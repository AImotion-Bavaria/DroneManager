import asyncio
import time
from collections import defaultdict

from dronemanager.plugin import Plugin


class RTCM3Plugin(Plugin):
    PREFIX = "rtcm3"
    DEPENDENCIES = []

    MAX_RTCM_PAYLOAD = 180
    
    # RTCM3 message types important for RTK
    #CRITICAL_MESSAGE_TYPES = {1004, 1005, 1006, 1008, 1012, 1019, 1020, 1033, 1042, 1045, 1046, 1077, 1087, 1097, 1107, 1127, 1230} # default RTKbase MSM7 messages
    CRITICAL_MESSAGE_TYPES = {1005, 1074, 1084, 1094, 1124, 1230} # reduced RTKbase MSM4 messages

    def __init__(self, dm, logger, name, host="192.168.1.40", port=5016,
                 idle_timeout=30.0, reconnect_delay=2.0, **kwargs):
        super().__init__(dm, logger, name, **kwargs)

        self.host = host
        self.port = int(port)
        self.idle_timeout = float(idle_timeout)
        self.reconnect_delay = float(reconnect_delay)
        self._rtcm_sequence_id = 0

        self.is_connected = False
        self.total_frames_parsed = 0
        self._should_run = True
        self._latest_packet: bytes | None = None
        self.forward_targets: set[str] = set()

        # Debugging metrics
        self.message_type_counters: dict[int, int] = defaultdict(int)  # msg_id -> count
        self.message_type_last_seen: dict[int, float] = {}  # msg_id -> timestamp
        self.message_type_sizes: dict[int, list[int]] = defaultdict(list)  # msg_id -> [sizes]
        self.bytes_received_count = 0
        self.last_receive_time = None
        self.connection_start_time = None
        self.total_forward_attempts = 0
        self.total_forward_failures = 0
        self.forward_per_drone: dict[str, dict] = defaultdict(lambda: {"sent": 0, "failed": 0})
        self.crc_errors = 0
        self.parse_errors = 0

        self.background_functions = []
        self.cli_commands = {
            "send": self.send_rtcm3_drone,
            "stop": self.stop_rtcm3_drone,
            "status": self.get_status,
            "debug": self.get_debug_status,
        }

    async def start(self):
        await super().start()
        if not self._running_tasks:
            self._running_tasks.add(asyncio.create_task(self._port_lifecycle_manager()))

    async def shutdown(self):
        """Explicit shutdown hook called by DroneManager framework (if supported)."""
        self.logger.info("Stopping RTCM3 background processing...")
        self._should_run = False
        self.is_connected = False
        await asyncio.sleep(0.1)

    def _extract_rtcm_packets(self, payload: bytes | bytearray) -> tuple[list[bytes], bytes]:
        """Pull all complete RTCM3 frames from a byte stream, leaving any partial tail behind."""
        buffer = bytearray(payload)
        packets: list[bytes] = []

        while len(buffer) >= 3:
            if buffer[0] != 0xD3:
                del buffer[0]
                continue

            length = ((buffer[1] & 0x03) << 8) | buffer[2]
            total_packet_size = length + 6

            if len(buffer) < total_packet_size:
                break

            packet_bytes = bytes(buffer[:total_packet_size])
            
            # Validate CRC24
            if not self._validate_crc24(packet_bytes):
                self.crc_errors += 1
                self.logger.debug(f"RTCM3 CRC error detected (total: {self.crc_errors})")
                del buffer[:total_packet_size]
                continue

            packets.append(packet_bytes)
            del buffer[:total_packet_size]

        return packets, bytes(buffer)
    
    def _validate_crc24(self, packet: bytes) -> bool:
        """Validate RTCM3 packet CRC24Q checksum."""
        if len(packet) < 6:
            return False
        
        # Extract the CRC from last 3 bytes
        received_crc = (packet[-3] << 16) | (packet[-2] << 8) | packet[-1]
        received_crc &= 0xFFFFFF  # Keep only 24 bits
        
        # Calculate CRC for payload (all bytes except last 3)
        calculated_crc = self._crc24q(packet[:-3])
        
        return received_crc == calculated_crc
    
    def _crc24q(self, data: bytes) -> int:
        """Calculate CRC24Q for RTCM3 data."""
        CRC24_POLY = 0x1864CFB
        crc = 0
        
        for byte in data:
            crc ^= (byte << 16)
            for _ in range(8):
                crc <<= 1
                if crc & 0x1000000:
                    crc ^= CRC24_POLY
        
        return crc & 0xFFFFFF

    async def _port_lifecycle_manager(self):
        """Async worker managing the TCP connection cycle."""
        self.logger.info(
            f"RTCM3 Plugin initialized. Listening for RTCM3 stream on {self.host}:{self.port} "
            f"(idle_timeout={self.idle_timeout}s, reconnect_delay={self.reconnect_delay}s)"
        )

        while self._should_run:
            reader = None
            writer = None
            try:
                reader, writer = await asyncio.open_connection(self.host, self.port)
                self.is_connected = True
                self.connection_start_time = time.time()
                self.bytes_received_count = 0
                self.logger.info(f"TCP link locked onto {self.host}:{self.port}.")

                byte_buffer = bytearray()
                while self._should_run:
                    try:
                        data = await asyncio.wait_for(reader.read(4096), timeout=self.idle_timeout)
                    except asyncio.TimeoutError:
                        self.logger.warning(
                            f"RTCM3 TCP idle timeout after {self.idle_timeout}s; closing connection and reconnecting."
                        )
                        break

                    if not data:
                        self.logger.warning("RTCM3 TCP socket closed by peer; reconnecting.")
                        break

                    self.bytes_received_count += len(data)
                    self.last_receive_time = time.time()
                    byte_buffer.extend(data)
                    packets, tail = self._extract_rtcm_packets(bytes(byte_buffer))
                    byte_buffer = bytearray(tail)

                    for rtcm_packet in packets:
                        msg_id = (rtcm_packet[3] << 4) | (rtcm_packet[4] >> 4)
                        self.total_frames_parsed += 1
                        self._distribute_packet(msg_id, rtcm_packet)

                    await asyncio.sleep(0)
            except (ConnectionResetError, ConnectionRefusedError, OSError) as exc:
                self.is_connected = False
                if self._should_run:
                    self.logger.warning(f"RTCM3 TCP connection lost ({exc}). Retrying in {self.reconnect_delay}s...")
                await asyncio.sleep(self.reconnect_delay)
            except Exception as exc:
                self.is_connected = False
                if self._should_run:
                    self.logger.error(f"RTCM3 parsing error occurred: {exc}")
                    self.parse_errors += 1
                await asyncio.sleep(self.reconnect_delay)
            finally:
                self.is_connected = False
                if writer is not None:
                    writer.close()
                    try:
                        await writer.wait_closed()
                    except Exception:
                        pass

    def _distribute_packet(self, msg_id: int, packet: bytes):
        """Pushes the isolated frame out into the DroneManager ecosystem and the connected drones."""
        self._latest_packet = packet
        packet_size = len(packet)
        
        # Track message type statistics
        self.message_type_counters[msg_id] += 1
        self.message_type_last_seen[msg_id] = time.time()
        self.message_type_sizes[msg_id].append(packet_size)
        
        # Log critical messages for debugging
        if msg_id in self.CRITICAL_MESSAGE_TYPES:
            self.logger.debug(
                f"RTCM3 Message Type {msg_id:4d}: Size={packet_size:3d}B, Total={self.message_type_counters[msg_id]:6d}"
            )
        
        if hasattr(self.dm, "publish"):
            self.dm.publish("gnss:rtcm3", {"id": msg_id, "raw": packet})
        elif hasattr(self.dm, "state"):
            self.dm.state.last_rtcm_frame = {
                "id": msg_id,
                "bytes": packet,
                "time": asyncio.get_event_loop().time(),
            }

        self._forward_to_drones(packet)

    def _chunk_packet(self, payload: bytes, max_payload: int | None = None) -> list[bytes]:
        if max_payload is None:
            max_payload = self.MAX_RTCM_PAYLOAD
        return [payload[i:i + max_payload] for i in range(0, len(payload), max_payload)]

    def _forward_to_drones(self, packet: bytes):
        """Send RTCM3 bytes strictly to drones explicitly added via 'send <drone>'."""
        if not self.forward_targets:
            return  # Standby mode: do not forward until targets are added via CLI

        drones = getattr(self.dm, "drones", {})

        for drone_name in list(self.forward_targets):
            if drone_name not in drones:
                continue

            drone = drones[drone_name]
            if not getattr(drone, "is_connected", False):
                continue

            mav_conn = getattr(drone, "mav_conn", None)
            if mav_conn is None:
                continue

            self._forward_to_drone(mav_conn, packet, drone_name)

    def _forward_to_drone(self, mav_conn, packet: bytes, drone_name: str = "unknown") -> bool:
        if mav_conn is None:
            return False

        mav = getattr(getattr(mav_conn, "con_drone_in", None), "mav", None)
        if mav is None or not hasattr(mav, "gps_rtcm_data_encode"):
            self.logger.error("RTCM3: MAV connection missing gps_rtcm_data_encode method.")
            self.forward_per_drone[drone_name]["failed"] += 1
            self.total_forward_failures += 1
            return False
            
        if not hasattr(mav_conn, "send_as_gcs"):
            self.logger.error("RTCM3: MAV connection missing send_as_gcs method.")
            self.forward_per_drone[drone_name]["failed"] += 1
            self.total_forward_failures += 1
            return False

        chunks = self._chunk_packet(packet)
        is_fragmented = len(chunks) > 1
        success = True

        for frag_id, chunk in enumerate(chunks):
            try:
                # Compute bitwise flags
                flags = (1 if is_fragmented else 0) | ((frag_id & 0x03) << 1) | ((self._rtcm_sequence_id & 0x1F) << 3)
                
                # FIX 1: Pad payload buffer strictly to 180 bytes for pymavlink
                chunk_len = len(chunk)
                padded_payload = chunk.ljust(180, b'\x00')
                
                # Encode MAVLink #233 message
                msg = mav.gps_rtcm_data_encode(flags, chunk_len, padded_payload)
                
                # Transmit frame as GCS
                mav_conn.send_as_gcs(msg)
                self.forward_per_drone[drone_name]["sent"] += 1
                self.total_forward_attempts += 1
            except Exception as e:
                self.logger.error(f"RTCM3: Failed to send chunk {frag_id} to {drone_name}: {e}")
                self.forward_per_drone[drone_name]["failed"] += 1
                self.total_forward_failures += 1
                success = False

        # Increment rolling sequence ID (0 to 31)
        self._rtcm_sequence_id = (self._rtcm_sequence_id + 1) % 32
        return success

    async def send_rtcm3_drone(self, name: str, packet: bytes | None = None):
        """Forward RTCM3 data to a specific drone by name using its MAV connection."""
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named '{name}'.")
            return False

        drone = self.dm.drones[name]
        mav_conn = getattr(drone, "mav_conn", None)
        if mav_conn is None:
            self.logger.warning(f"Drone '{name}' has no MAV connection configured.")
            return False

        if packet is None:
            packet = self._latest_packet
        if packet is None:
            self.logger.warning("No RTCM3 data is available to send.")
            return False

        self.forward_targets.add(name)
        self.logger.info(f"RTCM3 data forwarding locked explicitly to target drone: {name}.")
        return self._forward_to_drone(mav_conn, packet, name)

    async def stop_rtcm3_drone(self, name: str):
        """Remove a specific drone target from the forwarding filter registry."""
        if name in self.forward_targets:
            self.forward_targets.discard(name)
            self.logger.info(f"Stopped RTCM3 routing to drone '{name}'.")
            return True
        self.logger.warning(f"Drone '{name}' was not found.")
        return False

    async def get_status(self):
        """Inspect RTCM3 stream health."""
        connection_label = "CONNECTED" if self.is_connected else "DISCONNECTED/TCP UNAVAILABLE"
        uptime_str = "N/A"
        if self.connection_start_time:
            uptime = time.time() - self.connection_start_time
            uptime_str = f"{uptime:.1f}s"
        
        self.logger.info(f"--- [RTCM3 Interface Diagnostic] ---")
        self.logger.info(f"Target TCP endpoint : {self.host}:{self.port}")
        self.logger.info(f"Hardware Status     : {connection_label}")
        self.logger.info(f"Connection Uptime   : {uptime_str}")
        self.logger.info(f"Frames Extracted    : {self.total_frames_parsed}")
        self.logger.info(f"Bytes Received      : {self.bytes_received_count} bytes")
        self.logger.info(f"CRC Errors          : {self.crc_errors}")
        self.logger.info(f"Parse Errors        : {self.parse_errors}")
        self.logger.info(f"Forward Attempts    : {self.total_forward_attempts}")
        self.logger.info(f"Forward Failures    : {self.total_forward_failures}")
        self.logger.info(f"Active Targets      : {list(self.forward_targets)}")

    async def get_debug_status(self):
        """Comprehensive RTCM3 debug information for RTK fix troubleshooting."""
        self.logger.info("\n" + "="*70)
        self.logger.info("RTCM3 DEBUG STATUS - RTK FIX TROUBLESHOOTING")
        
        # Connection Status
        self.logger.info("\n[CONNECTION STATUS]")
        connection_label = "CONNECTED" if self.is_connected else "DISCONNECTED"
        self.logger.info(f"  Status: {connection_label}")
        self.logger.info(f"  Endpoint: {self.host}:{self.port}")
        if self.last_receive_time:
            time_since_last = time.time() - self.last_receive_time
            self.logger.info(f"  Time since last message: {time_since_last:.2f}s")
        
        # Data Reception
        self.logger.info("\n[DATA RECEPTION]")
        self.logger.info(f"  Total frames parsed: {self.total_frames_parsed}")
        self.logger.info(f"  Total bytes received: {self.bytes_received_count}")
        if self.bytes_received_count > 0 and self.total_frames_parsed > 0:
            avg_frame_size = self.bytes_received_count / self.total_frames_parsed
            self.logger.info(f"  Average frame size: {avg_frame_size:.1f} bytes")
        
        # Error Tracking
        self.logger.info("\n[ERROR TRACKING]")
        self.logger.info(f"  CRC errors: {self.crc_errors}")
        self.logger.info(f"  Parse errors: {self.parse_errors}")
        
        # Critical Message Types
        self.logger.info("\n[CRITICAL MESSAGE TYPES FOR RTK]")
        critical_received = False
        for msg_type in sorted(self.CRITICAL_MESSAGE_TYPES):
            if msg_type in self.message_type_counters:
                count = self.message_type_counters[msg_type]
                if count > 0:
                    last_seen = self.message_type_last_seen.get(msg_type, 0)
                    sizes = self.message_type_sizes.get(msg_type, [])
                    if sizes:
                        avg_size = sum(sizes) / len(sizes)
                        size_range = f"{min(sizes)}-{max(sizes)} bytes (avg: {avg_size:.1f})"
                    else:
                        size_range = "N/A"
                    
                    time_since = time.time() - last_seen if last_seen else 0
                    self.logger.info(
                        f"  Type {msg_type:4d}: Count={count:6d}, "
                        f"Size={size_range:30s}, Last={time_since:.2f}s ago"
                    )
                    critical_received = True
        
        if not critical_received:
            self.logger.warning("  ⚠️  NO CRITICAL MESSAGE TYPES RECEIVED!")
            self.logger.warning("  ⚠️  This will prevent RTK fix! Check your RTCM3 source.")
        
        # All Message Types
        self.logger.info("\n[ALL MESSAGE TYPES RECEIVED]")
        if self.message_type_counters:
            for msg_type in sorted(self.message_type_counters.keys()):
                if msg_type not in self.CRITICAL_MESSAGE_TYPES:
                    count = self.message_type_counters[msg_type]
                    self.logger.info(f"  Type {msg_type:4d}: Count={count:6d}")
        else:
            self.logger.info("  No messages received yet")
        
        # Forwarding Status
        self.logger.info("\n[DRONE FORWARDING STATUS]")
        if self.forward_per_drone:
            for drone_name, stats in self.forward_per_drone.items():
                sent = stats.get("sent", 0)
                failed = stats.get("failed", 0)
                total = sent + failed
                success_rate = (sent / total * 100) if total > 0 else 0
                status_icon = "✓" if failed == 0 else "✗"
                self.logger.info(
                    f"  {status_icon} {drone_name:15s}: Sent={sent:6d}, Failed={failed:6d} ({success_rate:.1f}% success)"
                )
        else:
            self.logger.info("  No active forwarding targets")
        
        self.logger.info(f"\nTotal forward attempts: {self.total_forward_attempts}")
        self.logger.info(f"Total forward failures: {self.total_forward_failures}")
        if self.total_forward_attempts > 0:
            overall_success = ((self.total_forward_attempts - self.total_forward_failures) 
                              / self.total_forward_attempts * 100)
            self.logger.info(f"Overall success rate: {overall_success:.1f}%")
        
        self.logger.info("\n" + "="*70 + "\n")


    