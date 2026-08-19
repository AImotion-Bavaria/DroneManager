"""Plugins for communication to other software

Currently only features a basic UDP server which sends data on connected drones and running missions in a json format.
"""
import asyncio
import select
import threading
import socket
import errno
import json
import time
import math

from dronemanager.plugin import Plugin
from dronemanager.utils import coroutine_awaiter


SERVER_PORT = 31659
MAX_FREQUENCY = 100
MIN_FREQUENCY = 1
MAX_DURATION = 60


class UDPPlugin(Plugin):
    """Communication happens over port 31659. A client will send a json message with the desired frequency and duration
    (in seconds) to this port and the server starts answering. Frequency is capped between 1/60 and 20Hz.

    Example message from client::

        {
          "duration": 30,
          "frequency": 5
        }

    """

    PREFIX = "UDP"

    def __init__(self, dm, logger, name, server_port: int = SERVER_PORT, max_frequency: float = MAX_FREQUENCY,
                 min_frequency: float = MIN_FREQUENCY, max_duration: float = MAX_DURATION):
        super().__init__(dm, logger, name)
        self.inport = server_port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(("", self.inport))
        self.socket = sock

        self.max_frequency = max_frequency
        self.min_frequency = min_frequency
        self.max_duration = max_duration

        outsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.outsocket = outsock
        self.clients: dict[tuple[str, int], UDPClient] = {}
        self.background_functions = [
        ]

        self._stop_threads = False
        self._event_loop = asyncio.get_running_loop()
        self._listen_thread = threading.Thread(target=self._listen_for_clients, args=(lambda: self._stop_threads,))
        self._listen_thread.start()

    async def close(self):
        await super().close()
        self._stop_threads = True

    async def status(self):
        """Log currently connected clients."""
        self.logger.info(f"Clients: {' | '.join([str(client) for client in self.clients])}")

    def _listen_for_clients(self, stop):
        self.logger.debug("Listening for clients...")
        while not stop():
            try:
                try:
                    rlist, _, _ = select.select([self.socket], [], [], 1)
                    if len(rlist) == 1:
                        msg, addr = self.socket.recvfrom(1024)
                    else:
                        continue
                except socket.error as e:
                    if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                        continue
                    else:
                        raise
                self.logger.debug(f"Received message from {addr}")
                json_dict = json.loads(msg)
                if "frequency" not in json_dict or "duration" not in json_dict:
                    self.logger.debug(f"Invalid JSON message received from {addr}")
                    continue
                ip, port = addr
                frequency = json_dict["frequency"]
                if frequency > self.max_frequency:
                    frequency = self.max_frequency
                elif frequency < self.min_frequency:
                    frequency = self.min_frequency
                duration = json_dict["duration"]
                if duration <= 0:
                    duration = self.max_duration
                if (ip, port) not in self.clients:
                    client = UDPClient(ip, port, frequency, duration)
                    self.clients[(ip, port)] = client
                    send_task = asyncio.run_coroutine_threadsafe(self._client_sender(client), self._event_loop)
                    awaiter_task = asyncio.run_coroutine_threadsafe(coroutine_awaiter(send_task, self.logger),
                                                                    self._event_loop)
                    self.running_tasks.add(send_task)
                    self.running_tasks.add(awaiter_task)
                    self.logger.info(f"New client @{ip, port} with frequency {frequency} "
                                     f"and duration {math.inf if duration == 0 else duration}.")
                else:
                    self.logger.debug(f"Received repeat message from existing client {ip, port}, "
                                      f"updating parameters and resetting timer...")
                    client = self.clients[(ip, port)]
                    client.start_time = time.time()
                    client.duration = duration
                    client.frequency = frequency
            except Exception as e:
                self.logger.warning("Exception listening for incoming UDP!")
                self.logger.debug(repr(e), exc_info=True)
                self.logger.debug("Dummy")

    async def _client_sender(self, client: "UDPClient"):
        """Send data to the client.
        """
        # TODO: If we ever have a larger number of clients we should cache the jsons somehow
        # TODO: The OSError for the send command is raised at the recvfrom for some reason, which breaks this client
        #  closing. Not critical while we have a limited max duration.
        while client.duration == 0 or time.time() < (client.start_time + client.duration):
            try:
                data = self._make_json()
                self._send_msg(data, client.ip, client.port)
            except OSError as e:
                self.logger.info("Couldn't send information, closing connection...")
                self.logger.info(f"{e.errno}: {e.strerror}")
                break
            except Exception as e:
                self.logger.warning("Exception sending data out over UDP! Check the log for details.")
                self.logger.debug(repr(e), exc_info=True)
            await asyncio.sleep(1 / client.frequency)
        if (client.ip, client.port) in self.clients:
            self.clients.pop((client.ip, client.port))
        self.logger.info(f"Finished sending data to {client.ip, client.port}")

    def _make_json(self):
        drone_data = {}
        for drone_name in self.dm.drones:
            drone = self.dm.drones[drone_name]
            # Target Logic
            target_list = []
            current_target = drone.path_generator.target_position
            if current_target is not None:
                current_target = current_target.pos.tolist()
            target_list.append(current_target)

            # fence logic
            fence_list = []
            if getattr(drone, 'fence', None):  # Check if fence exists and is not None
                fence_list = [
                    drone.fence.north_lower,
                    drone.fence.north_upper,
                    drone.fence.east_lower,
                    drone.fence.east_upper,
                    drone.fence.down_lower,
                    drone.fence.down_upper,
                    drone.fence.safety_level
                ]
            drone_data[drone_name] = {
                "position": drone.position_ned.tolist(),
                "gps": drone.position_global.tolist(),
                "velocity": drone.velocity.tolist(),
                "attitude": drone.attitude.tolist(),
                "mode": drone.flightmode.name,
                "conn": drone.is_connected,
                "armed": drone.is_armed,
                "in_air": drone.in_air,
                "rtsp": drone.config.rtsp,
                "fence": fence_list,
                "target": target_list,
            }
        data = {"drones": drone_data}
        if hasattr(self.dm, "mission"):  # Check that the mission plugin is actually loaded
            mission_data = {}
            data["missions"] = mission_data
            for mission_name in self.dm.mission.loaded:
                mission = getattr(self.dm, mission_name)
                mission_data[mission.PREFIX] = {
                    "flight-area": mission.flight_area.bounding_box if mission.flight_area is not None else None,
                    "stage": mission.current_stage.name if mission.current_stage is not None else None,
                    "drones": list(mission.drones.keys()),
                }
                for info, item in mission.additional_info.items():
                    try:
                        mission_data[mission.PREFIX][info] = str(item)
                    except Exception as e:
                        self.logger.warning("Couldn't collect all mission information to send out due to an exception!")
                        self.logger.debug(repr(e), exc_info=True)
        return json.dumps(data)

    def _send_msg(self, msg: str, ip="localhost", port=None):
        if port is None:
            port = self.inport
        try:
            self.outsocket.sendto(msg.encode("utf-8"), (ip, port))
        except Exception as e:
            self.logger.warning("Exception sending out data! Check the log for details.")
            self.logger.debug(repr(e), exc_info=True)


class UDPClient:

    def __init__(self, ip, port, frequency, duration):
        self.start_time = time.time()
        self.ip = ip
        self.port = port
        self.frequency = frequency
        self.duration = duration

    def __str__(self):
        return (f"Client: {self.ip}:{self.port}, {self.frequency} Hz, "
                f"{self.start_time + self.duration - time.time()}s remaining.")
