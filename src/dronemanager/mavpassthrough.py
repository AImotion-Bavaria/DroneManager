import logging
import datetime
import math
import time
import os
import asyncio
import typing
import struct

from pymavlink import mavutil

from dronemanager.utils import COMMON_FORMATTER, coroutine_awaiter, LOG_DIR

# TODO: Routing between multiple GCS so we can have my app and QGroundControl connected at the same time
# TODO: Implement sending as drone/drone components
# TODO: Some kind of generic message matching method for callbacks to use
# TODO: This module also suffers from a memory leak due to pymavlink. The amount of leakage, about 200MB per hour, is
#  quite small compared to the duration of most missions but still.


class MAVPassthrough:
    def __init__(self, dialect=None, loggername="passthrough", log_messages=True):
        self.dialect = dialect
        self.source_system = 246
        self.source_component = 201
        self.con_drone_in: mavutil.mavudp | mavutil.mavserial | None = None
        self.con_gcs: mavutil.mavudp | None = None

        self.drone_system = 0
        self.drone_component = 0
        self.drone_autopilot = None
        self.gcs_system = 0
        self.gcs_component = 0

        self.time_of_last_gcs = 0
        self.time_of_last_drone = 0
        self.disconnected_thresh = 2e9

        self.logger = logging.getLogger(loggername)
        self.logger.setLevel(logging.DEBUG)
        filename = f"{loggername}_{datetime.datetime.now()}"
        filename = filename.replace(":", "_").replace(".", "_") + ".log"
        os.makedirs(LOG_DIR, exist_ok=True)
        file_handler = logging.FileHandler(os.path.join(LOG_DIR, filename))
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(COMMON_FORMATTER)
        self.logger.addHandler(file_handler)
        self.logging_handlers = []
        self.logging_handlers.append(file_handler)
        self.log_messages = log_messages

        self.running_tasks = set()
        self.should_stop = False

        self._drone_receive_callbacks: dict[int, set[typing.Callable[[any], typing.Coroutine]]] = {}
        self._ack_waiters: dict[tuple[int, int, int, int, int], list[asyncio.Future]] = {}
        self._msg_waiters: dict[tuple[int, int, int], list[asyncio.Future]] = {}

        self._sockets = set()  # A set with all active mavlink connections. Closed when we shut down

    def connect_gcs(self, address):
        self.running_tasks.add(asyncio.create_task(self._connect_gcs(address)))

    async def _connect_gcs(self, address):
        self.con_gcs = mavutil.mavlink_connection("udpout:" + address,
                                                  source_system=self.source_system,
                                                  source_component=self.source_component, dialect=self.dialect)
        self._sockets.add(self.con_gcs)
        self.running_tasks.add(asyncio.create_task(self._send_heartbeats(self.con_gcs, "GCS", mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID)))
        await asyncio.sleep(0)
        self.logger.debug("Waiting for GCS heartbeat")
        gcs_heartbeat = self.con_gcs.wait_heartbeat(blocking=False)
        while not gcs_heartbeat:
            await asyncio.sleep(0.05)
            gcs_heartbeat = self.con_gcs.wait_heartbeat(blocking=False)
        self.gcs_system = gcs_heartbeat.get_srcSystem()
        self.gcs_component = gcs_heartbeat.get_srcComponent()
        self.logger.debug(f"Got GCS {self.gcs_system, self.gcs_component} heartbeat.")
        self.time_of_last_gcs = time.time_ns()
        self.running_tasks.add(asyncio.create_task(self._send_pings(self.con_gcs, "GCS")))

        handshake_task = asyncio.create_task(self._do_version_handshake(self.con_gcs, "GCS"))
        self.running_tasks.add(handshake_task)
        await handshake_task

        self.running_tasks.add(asyncio.create_task(self._listen_gcs()))

    def connect_drone(self, loc, appendix, scheme="udp"):
        if scheme == "udp":
            self.running_tasks.add(asyncio.create_task(self._connect_drone_udp(loc, appendix)))
        elif scheme == "serial":
            self.running_tasks.add(asyncio.create_task(self._connect_drone_serial(loc, appendix)))

    async def _connect_drone_serial(self, path, baud):
        try:
            self.logger.debug(f"Connecting to drone @{path}:{baud}")

            tmp_con_drone_in = mavutil.mavlink_connection(f"{path}",
                                                          baud=baud,
                                                          source_system=self.source_system,
                                                          source_component=self.source_component,
                                                          dialect=self.dialect)
            self._sockets.add(tmp_con_drone_in)
            await self._connect_drone(tmp_con_drone_in)
        except Exception as e:
            self.logger.debug(f"Error during connection to drone: {repr(e)}", exc_info=True)

    async def _connect_drone_udp(self, ip, port):
        self.logger.debug(f"Connecting to drone @{ip}:{port}")
        if ip == "":
            tmp_con_out = mavutil.mavlink_connection(f"udp::{port}",
                                                     input=True,
                                                     source_system=self.source_system,
                                                     source_component=self.source_component, dialect=self.dialect)
            tmp_con_local = None
        else:
            # Sometimes, FCs don't answer on the port from which we send our heartbeats, so we have to listen on the
            # same port as the destination as well.
            tmp_con_out = mavutil.mavlink_connection(f"udp:{ip}:{port}",
                                                     input=False,
                                                     source_system=self.source_system,
                                                     source_component=self.source_component, dialect=self.dialect)
            tmp_con_local = mavutil.mavlink_connection(device=f"udp::{port}",
                                                       input = True,
                                                       source_system=self.source_system,
                                                       source_component=self.source_component, dialect=self.dialect)
            self._sockets.add(tmp_con_local)
        self._sockets.add(tmp_con_out)
        await self._connect_drone(tmp_con_out, tmp_con_local)

    async def _connect_drone(self, con, backup = None):
        # Backup is a secondary connection (usually local) on which we listen for heartbeats as well.
        # Used with UDP as some FCs are configured to answer NOT on the port on which they are pinged, but on a hard
        # coded one.
        try:
            init_con_task = asyncio.create_task(self._process_initial_drone_connection(con, local = backup))
            self.running_tasks.add(init_con_task)
            await init_con_task

            # Start continuously sending heartbeats and pings once we have system and component id from drone
            self.running_tasks.add(asyncio.create_task(self._send_heartbeats(self.con_drone_in, "drone", mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID)))
            self.running_tasks.add(asyncio.create_task(self._send_pings(self.con_drone_in, "drone")))

            # Do version handshake
            handshake_task = asyncio.create_task(self._do_version_handshake(self.con_drone_in, "Drone"))
            self.running_tasks.add(handshake_task)
            await handshake_task

             # Start listening and transferring messages
            self.running_tasks.add(asyncio.create_task(self._listen_drone()))
        except Exception as e:
            self.logger.debug(f"Error during connection to drone: {repr(e)}", exc_info=True)

    async def _process_initial_drone_connection(self, con, local = None):
        self.logger.debug("Sending initial drone heartbeats")
        hb_received_con = False
        hb_received_local = False
        use_local = False
        while not hb_received_local and not hb_received_con:
            con.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            # Listen for heartbeats on the two ports
            hb_received_con = self._check_heartbeat(con)
            if local is not None:
                hb_received_local = self._check_heartbeat(local)
            # Use the local port if we get a heartbeat on that, use the remote if we get a heartbeat on that but not
            # local, keep listening if we get no heartbeats at all
            if hb_received_local:
                use_local = True
            if not hb_received_local and not hb_received_con:
                await asyncio.sleep(0.5)
        if use_local:
            self.con_drone_in = local
            con.close()
        else:
            self.con_drone_in = con
            if local is not None:
                local.close()
                self._sockets.remove(local)

    def _check_heartbeat(self, con):
        m = con.wait_heartbeat(blocking=False)
        hb_received = False
        if m is not None:
            if m.get_srcSystem() != self.source_system and m.get_srcComponent() == 1:
                hb_received = True
                self.drone_system = m.get_srcSystem()
                self.drone_component = m.get_srcComponent()
                self.logger.debug(f"Got drone {self.drone_system, self.drone_component} heartbeat!")
                # Check whether we are connecting to ardupilot or PX4 and change dialect accordingly
                if self.dialect is None:
                    match m.autopilot:
                        case 3:
                            self.drone_autopilot = "ArduPilot"
                            self.dialect = "ardupilotmega"
                        case 12:
                            self.drone_autopilot = "PX4"
                            self.dialect = "cubepilot"
                    self.logger.debug(f"Identified drone as {self.drone_autopilot}")
        return hb_received

    async def _do_version_handshake(self, con, con_name):
        try:
            self.logger.info(f"MAV Version {con_name}: {con.mavlink20()}")

            # Version handshake, have to do this explicitly to avoid breaking messages.
            # Send protocol version request
            msg = con.mav.command_long_encode(self.drone_system, self.drone_component, 512, 0,
                                                            300, math.nan, math.nan, math.nan, math.nan, math.nan, 1)
            con.mav.send(msg)
            self.logger.debug(f"Sent Protocol message request {msg.get_srcSystem(), msg.get_srcComponent()}: {msg.to_dict()}")

            # In theory we should listen to the response, but the main issue currently is getting two links that support mavlink 2 to actually use it, for which the response is pointless.
            # pymavlink autoupgrades us, as soon as the mavlink 2 response is sent, and we can't receive it anyway since we're on mavlink 1 until then, which doesn't support the message.
            # If we ever have a system that is mavlink 1 only, or there is a newer mavlink version, we should look into doing this "properly"

            return True
        except Exception as e:
            self.logger.debug(repr(e), exc_info=True)


    def connected_to_drone(self):
        if time.time_ns() - self.time_of_last_drone < self.disconnected_thresh:
            return True
        else:
            return False

    def connected_to_gcs(self):
        if time.time_ns() - self.time_of_last_gcs < self.disconnected_thresh:
            return True
        else:
            return False

    def send_as_gcs(self, msg):
        try:
            self.con_drone_in.mav.srcSystem = self.gcs_system
            self.con_drone_in.mav.srcComponent = self.gcs_component
            self.con_drone_in.mav.send(msg)
            self.logger.debug(f"Sent Message as GCS {msg.get_srcSystem(), msg.get_srcComponent()}: {msg.to_dict()}")
            self.con_drone_in.mav.srcSystem = self.source_system
            self.con_drone_in.mav.srcComponent = self.source_component
        except Exception as e:
            self.logger.debug(repr(e), exc_info=True)

    def listen_ack(self, command_id, target_component) -> asyncio.Future:
        received = asyncio.Future()
        msg_tuple = (command_id, self.drone_system, target_component, self.gcs_system, self.gcs_component)
        if msg_tuple in self._ack_waiters:
            self._ack_waiters[msg_tuple].append(received)
        else:
            self._ack_waiters[msg_tuple] = [received]
        return received

    def listen_message(self, message_id, target_component) -> asyncio.Future:
        received = asyncio.Future()
        msg_tuple = (message_id, self.drone_system, target_component)
        if msg_tuple in self._msg_waiters:
            self._msg_waiters[msg_tuple].append(received)
        else:
            self._msg_waiters[msg_tuple] = [received]
        return received

    def send_cmd_long(self, target_component, cmd, param1=math.nan, param2=math.nan, param3=math.nan,
                      param4=math.nan, param5=math.nan, param6=math.nan, param7=math.nan) -> asyncio.Future:
        msg = self.con_drone_in.mav.command_long_encode(self.drone_system, target_component, cmd, 0,
                                                        param1, param2, param3, param4, param5, param6, param7)
        self.send_as_gcs(msg)
        return self.listen_ack(cmd, target_component)

    def send_request_message(self, target_component, message_id, param1=math.nan, param2=math.nan,
                             param3=math.nan, param4=math.nan, param5=math.nan, response_target=1):
        request_ack = self.send_cmd_long(target_component, 512, message_id, param1, param2, param3, param4, param5,
                                         response_target)
        message = self.listen_message(message_id, target_component)
        return message, request_ack

    async def request_message(self, target_component, message_id, param1=math.nan, param2=math.nan,
                              param3=math.nan, param4=math.nan, param5=math.nan, response_target=1, timeout=5):
        message, request_ack = self.send_request_message(target_component, message_id, param1=param1, param2=param2,
                                                         param3=param3, param4=param4, param5=param5,
                                                         response_target=response_target)
        ack_wait = asyncio.wait_for(request_ack, timeout)
        msg_wait = asyncio.wait_for(message, timeout)
        ack_good, msg = await asyncio.gather(ack_wait, msg_wait, return_exceptions=True)
        if isinstance(ack_good, Exception):
            self.logger.debug(repr(ack_good), exc_info=True)
            ack_good = False
        if isinstance(msg, Exception):
            self.logger.debug(repr(msg), exc_info=True)
            msg = False
        if msg:
            if not ack_good:
                self.logger.debug("Received the requested message, but not the request acknowledgement.")
            return msg
        else:
            if ack_good:
                self.logger.debug("Received the request acknowledgement, but not the message.")
            return msg

    def send_param_ext_request_list(self, target_component):
        msg = self.con_drone_in.mav.param_ext_request_list_encode(self.drone_system, target_component)
        self.send_as_gcs(msg)

    def send_param_ext_set(self, target_component, param_id, param_value: int | float, param_type: int):
        if param_type in [1, 3, 5, 7]:
            encoded_param_value = int.to_bytes(param_value, length=2 ** int(param_type / 2),
                                               byteorder="little", signed=False)
        elif param_type in [2, 4, 6, 8]:
            encoded_param_value = int.to_bytes(param_value, length=2 ** int(param_type / 2),
                                               byteorder="little", signed=True)
        elif param_type == 9:
            encoded_param_value = struct.pack("<f", param_value)
        elif param_type == 10:
            encoded_param_value = struct.pack("<d", param_value)
        else:
            raise NotImplementedError("Custom parameter types are not supported!")
        msg = self.con_drone_in.mav.param_ext_set_encode(self.drone_system, target_component,
                                                         param_id.encode("ascii"), encoded_param_value, param_type)
        self.send_as_gcs(msg)

    def send_param_ext_request_read(self, target_component, param_id: str, param_index: int = None):
        if param_index is None:
            msg = self.con_drone_in.mav.param_ext_request_read_encode(self.drone_system, target_component,
                                                                      param_id.encode("ascii"), -1)
        else:
            msg = self.con_drone_in.mav.param_ext_request_read_encode(self.drone_system, target_component, b"\x00", param_index)
        self.send_as_gcs(msg)

    def _process_message_for_return(self, msg):
        msg_id = msg.get_msgId()  # msg.id is sometimes not set correctly.
        if msg_id == -1:
            self.logger.debug(f"Message with BAD_DATA id, can't resend: {msg_id}, {msg.to_dict()}")
            return False
        if msg_id == -2:
            self.logger.debug(f"Message with unkown MAVLink ID, can't resend: {msg_id}, {msg.get_type()} "
                              f"{msg.fieldnames}, {msg.fieldtypes}, {msg.orders}, {msg.lengths}, "
                              f"{msg.array_lengths}, {msg.crc_extra}, {msg.unpacker}")
            return False
        return True

    async def _listen_gcs(self):
        self.logger.debug("Starting to listen to GCS")
        while not self.should_stop:
            try:
                if self.con_gcs is not None:
                    if await asyncio.get_running_loop().run_in_executor(None, self.con_gcs.select, 1):
                        # Receive and log all messages from the GCS
                        msg = self.con_gcs.recv_match(blocking=False)
                        if self.log_messages:
                            self.logger.debug(f"Message from GCS {msg.get_srcSystem(), msg.get_srcComponent()}, "
                                              f"{msg.to_dict()}")
                        self.time_of_last_gcs = time.time_ns()
                        if self.con_drone_in is not None and self.connected_to_gcs():  # Send onward to the drone
                            self.con_drone_in.mav.srcSystem = msg.get_srcSystem()
                            self.con_drone_in.mav.srcComponent = msg.get_srcComponent()
                            if self._process_message_for_return(msg):
                                try:
                                    self.con_drone_in.mav.send(msg)
                                except Exception as e:
                                    self.logger.debug(f"Encountered an exception sending message to drone: "
                                                      f"{repr(e)}", exc_info=True)
                            self.con_drone_in.mav.srcSystem = self.source_system
                            self.con_drone_in.mav.srcComponent = self.source_component
                else:
                    await asyncio.sleep(1)
            except Exception as e:
                self.logger.debug(f"Exception in the drone connection function: {repr(e)}", exc_info=True)

    async def _wait_for_drone_data(self) -> bool:
        if getattr(self.con_drone_in, "fd", None) is None:
            # pymavlink can't select on serial ports on Windows, its select just sleeps for half a second. Poll the
            # (non-blocking) port instead.
            await asyncio.sleep(0.005)
            return True
        return await asyncio.get_running_loop().run_in_executor(None, self.con_drone_in.select, 1)

    async def _listen_drone(self):
        self.logger.debug("Starting to listen to drone")
        while not self.should_stop:
            try:
                if self.con_drone_in is not None:
                    if await self._wait_for_drone_data():
                        # Process everything that arrived, not just a single message per wait.
                        while (msg := self.con_drone_in.recv_match(blocking=False)) is not None:
                            self._process_drone_message(msg)
                            await asyncio.sleep(0)
                else:
                    await asyncio.sleep(1)
            except Exception as e:
                self.logger.debug(f"Exception in the drone connection function: {repr(e)}", exc_info=True)

    def _process_drone_message(self, msg: mavutil.mavlink.MAVLink_message):
        if self.log_messages:
            self.logger.debug(f"Message from Drone {msg.get_srcSystem(), msg.get_srcComponent()}, "
                              f"{msg.to_dict()}")
        self.time_of_last_drone = time.time_ns()
        if self.con_gcs is not None and self.connected_to_drone():
            self.con_gcs.mav.srcSystem = msg.get_srcSystem()
            self.con_gcs.mav.srcComponent = msg.get_srcComponent()
            if self._process_message_for_return(msg):
                try:
                    self.con_gcs.mav.send(msg)
                except Exception as e:
                    self.logger.debug(f"Encountered an exception sending message to GCS: {repr(e)}",
                                      exc_info=True)
                # Do callbacks
                msg_id = msg.get_msgId()
                if msg_id in self._drone_receive_callbacks:
                    callbacks = list(self._drone_receive_callbacks[msg.get_msgId()])
                    for coro in callbacks:
                        self.logger.debug(f"Doing callback {coro} for message "
                                          f"with ID {msg.get_msgId()}")
                        task = asyncio.create_task(coro(msg))
                        self.running_tasks.add(task)
                        self.running_tasks.add(asyncio.create_task(coroutine_awaiter(task,
                                                                                     self.logger)))
                # Check acks
                if msg_id == 77:
                    msg_tuple = (msg.command, msg.get_srcSystem(), msg.get_srcComponent(),
                                 msg.target_system, msg.target_component)
                    if msg_tuple in self._ack_waiters:
                        futs = self._ack_waiters[msg_tuple]
                        if len(futs) > 0:
                            fut = futs.pop(0)
                            if msg.result == 0:
                                fut.set_result(True)
                            else:
                                fut.set_result(False)
                # Check other msgs
                else:
                    msg_tuple = (msg.get_msgId(), msg.get_srcSystem(), msg.get_srcComponent())
                    if msg_tuple in self._msg_waiters:
                        futs = self._msg_waiters.pop(msg_tuple)
                        for fut in futs:
                            fut.set_result(msg)
            self.con_gcs.mav.srcSystem = self.source_system
            self.con_gcs.mav.srcComponent = self.source_component

    async def _send_pings(self, con, name):
        while not self.should_stop:
            self.logger.debug(f"Pinging {name}")
            con.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            await asyncio.sleep(5)

    async def _send_heartbeats(self, con, name, mav_type, mav_autopilot):
        while not self.should_stop:
            self.logger.debug(f"Sending heartbeat to {name}")
            con.mav.heartbeat_send(mav_type, mav_autopilot, 0, 0, 0)
            await asyncio.sleep(0.5)

    def add_drone_message_callback(self, message_id: int, func: typing.Callable[[any], typing.Coroutine]):
        if message_id in self._drone_receive_callbacks:
            self._drone_receive_callbacks[message_id].add(func)
        else:
            self._drone_receive_callbacks[message_id] = {func}

    def remove_drone_message_callback(self, message_id: int, func: typing.Callable[[any], typing.Coroutine]):
        if message_id in self._drone_receive_callbacks:
            self._drone_receive_callbacks[message_id].remove(func)

    async def stop(self):
        self.logger.debug("Stopping")
        self.should_stop = True
        for sock in self._sockets:
            try:
                sock.close()
            except TypeError:
                pass
        for task in self.running_tasks:
            if isinstance(task, asyncio.Future):
                task.cancel()
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)


async def main():
    snoop = MAVPassthrough()
    snoop.connect_drone(loc="192.168.1.37", appendix=14567)
    snoop.connect_gcs("127.0.0.1:15567")
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
