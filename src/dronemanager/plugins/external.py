"""A plugin for generic communication with external components.

Currently only features a basic UDP server which sends data on connected drones and running missions in a json format.
The server listens on a fixed port for incoming requests from clients. Clients can request messages for a certain
duration and at a certain frequency. If clients want to extend the duration, they can send another request before the
duration expires and the duration will be extended. Clients can change their message frequency using the same method.

Messages contain core information, such as the position, velocity, whether the drone is armed or in the air,
battery status and more. They also carry information about loaded missions, such as flight area, participating drones
and custom information the missions define themselves with their
:py:attr:`~dronemanager.plugins.mission.Mission.additional_info` attribute.


This module also contains a minimal example client :py:class:`DummyUDPClient` and a simple script demonstrating the
behaviour of the server. Running this module as a script without commands launches an echo client that requests
messages from the server and prints them to console.

The full command line options are `python external.py <server_address?> --port <server_port> -e`:
* server_address: The IP address of the server.
* --port server_port: The port of the server.
* -e: A flag which changes the behaviour of the example script. Instead of indefinitely requesting messages
"""
import argparse
import asyncio
from concurrent.futures import Future
import errno
import json
import logging
import math
import select
import socket
import threading
import time
from types import TracebackType
from typing import Any, Callable, Coroutine

import dronemanager.core
from dronemanager.plugin import Plugin
from dronemanager.utils import coroutine_awaiter, cancel_running_tasks


DEFAULT_SERVER_PORT: int = 31659
"""The default port on which the server is listening for incoming client connections."""

DEFAULT_MAX_FREQUENCY: float = 100
"""The default maximum message frequency in Hz that clients can request."""

DEFAULT_MIN_FREQUENCY: float = 1
"""The default minimum message frequency in Hz that clients can request."""

DEFAULT_MAX_DURATION: float = 60
"""The default maximum duration in seconds for which clients can request messages."""


class UDPClient:
    """This class tracks properties for a single connected client."""

    def __init__(self, ip: str, port: int, frequency: float, duration: float):
        """Create the UDPClient object.

        Args:
            ip: The IP address of this client.
            port: The port from which this client requested messages.
            frequency: The requested message frequency for this client.
            duration: The requested transmission duration for this client.
        """
        self.start_time: float = time.time()  #: The time this client connected.
        self.ip = ip  #: The IP address of this client.
        self.port = port  #: The port from which this client requested messages.
        self.frequency = frequency  #: The requested message frequency for this client.
        self.duration = duration  #: The requested transmission duration for this client.

    def __str__(self) -> str:
        """A nice string representation of a connected client.

        Returns:
            A pretty string.
        """
        return (f"Client: {self.ip}:{self.port}, {self.frequency} Hz, "
                f"{self.start_time + self.duration - time.time()}s remaining.")


class UDPPlugin(Plugin):
    """A plugin for generic communication with external components using UDP and JSON.

    On startup, launches a server listening on a fixed port for incoming requests from clients.
    Clients can request messages for a certain duration and at a certain frequency by sending a JSON message to
    the server. If clients want to extend the duration, they can send another request before the duration expires and
    the duration will be extended. Clients can change their message frequency using the same method.

    The fixed port, the maximum and minimum message frequency and the maximum duration for which clients can request
    messages can be set in the configuration file, with defaults set in this module. Note that the maximum duration
    is the duration per client message, clients can extend the duration indefinitely by sending a new request before
    the old one expires.

    Example message from client::

        {
          "duration": 30,
          "frequency": 5
        }

    This plugin has one CLI command:

    * "status" - :py:meth:`status`: Log information about currently connected clients.
    """

    PREFIX = "UDP"
    """The prefix for the CLI commands, "UDP" by default."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str,
                 server_port: int = DEFAULT_SERVER_PORT, max_frequency: float = DEFAULT_MAX_FREQUENCY,
                 min_frequency: float = DEFAULT_MIN_FREQUENCY, max_duration: float = DEFAULT_MAX_DURATION):
        """Create a UDPPlugin instance.

        Args:
            dm: The DroneManager instance associated with this plugin.
            logger: The logger for errors and output.
            name: The name of the plugin.
            server_port: The port on which the server listens for incoming connections.
            max_frequency: The maximum message frequency which clients can request.
            min_frequency: The minimum message frequency which clients can request.
            max_duration: The maximum duration which clients can request.
        """
        super().__init__(dm, logger, name)
        self.server_port: int = server_port  #: The port for incoming clients requests.
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(("", self.server_port))
        self.socket: socket.socket = sock  #: The socket object for incoming messages.

        self.max_frequency: float = max_frequency  #: The maximum requestable frequency.
        self.min_frequency: float = min_frequency  #: The minimum requestable frequency.
        self.max_duration: float = max_duration  #: the maximum requestable duration for a single request.

        out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out_socket: socket.socket = out_sock  #: The socket for outgoing messages.
        self.clients: dict[tuple[str, int], UDPClient] = {}  #: A dictionary of clients by their addresses.
        self.background_functions: list[Coroutine] = [
        ]

        self._stop_threads = False  #: Set to True when we stop
        self._event_loop = asyncio.get_running_loop()  #: Reference to the event loop for callback coroutines.
        #: The thread listening for incoming messages.
        self._listen_thread = threading.Thread(target=self._listen_for_clients, args=(lambda: self._stop_threads,))
        self._listen_thread.start()

    async def close(self):
        """Closes the plugin.

        Cancels running coroutines and stops the thread.
        """
        self._stop_threads = True
        self.socket.close()
        await asyncio.to_thread(self._listen_thread.join)
        self.out_socket.close()
        await super().close()

    async def status(self):
        """Log currently connected clients."""
        self.logger.info(f"Clients: {' | '.join([str(client) for client in self.clients])}")

    def _listen_for_clients(self, stop: Callable[[], bool]):
        """Listen method for the listen thread.

        Listens for incoming messages on the server socket, parses the requests and instantiates the message emitting
        functions for each client. New clients are logged, updated requests from already connected clients are
        processed quietly.

        Args:
            stop: Callable to determine when the thread should end.

        Raises:
            socket.error: Potential connection errors.
        """
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
                    if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED, errno.ENOTSOCK]:
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

    async def _client_sender(self, client: UDPClient):
        """Message emitting coroutine.

        Creates data json and sends it to the client while their requested duration has not yet expired.

        Args:
            client: The client to which we send information.
        """
        next_msg_time = time.monotonic()
        while client.duration == 0 or time.monotonic() < (client.start_time + client.duration):
            prev_msg_time = next_msg_time
            msg_interval = 1 / client.frequency
            next_msg_time = prev_msg_time + msg_interval
            try:
                data = self._make_json()
                self._send_msg(data, client.ip, client.port)
            except OSError as e:
                # This doesn't seem to work, we just never get an exception if the client closes.
                self.logger.info("Couldn't send information, closing connection...")
                self.logger.info(f"{e.errno}: {e.strerror}")
                break
            except Exception as e:
                self.logger.warning("Exception sending data out over UDP! Check the log for details.")
                self.logger.debug(repr(e), exc_info=True)
            await asyncio.sleep(max(0, next_msg_time - time.monotonic()))
        if (client.ip, client.port) in self.clients:
            self.clients.pop((client.ip, client.port))
        self.logger.info(f"Finished sending data to {client.ip, client.port}")

    def _make_json(self) -> str:
        """Grabs data from DroneManager and constructs a JSON object.

        Returns:
            The JSON as a string.
        """
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
            fence_box: list[float] = []
            fence_type = None
            if getattr(drone, 'fence', None):  # Check if fence exists and is not None
                fence_type = drone.fence.__class__.__name__
                fence_box = drone.fence.bounding_box.tolist()
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
                "fence-type": fence_type,
                "fence-box": fence_box,
                "target": target_list,
            }
        data = {"drones": drone_data}
        if hasattr(self.dm, "mission"):  # Check that the mission plugin is actually loaded
            mission_data = {}
            data["missions"] = mission_data
            for mission_name in self.dm.mission.loaded:
                mission = self.dm.mission.loaded[mission_name]
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

    def _send_msg(self, msg: str, ip: str = "localhost", port: int = None):
        """Sends a single message to the target address using the outgoing socket.

        Args:
            msg: The message to be sent.
            ip: The address IP.
            port: The address port.
        """
        if port is None:
            port = self.server_port
        try:
            self.out_socket.sendto(msg.encode("utf-8"), (ip, port))
        except Exception as e:
            self.logger.warning("Exception sending out data! Check the log for details.")
            self.logger.debug(repr(e), exc_info=True)


class DummyUDPClient:
    """A dummy UDP client that starts a connection and then keeps it alive until shut down.

    Intended as an example of how the client side to the server might be implemented.
    Once started, automatically sends fresh requests once the previous one is about to expire. Requests can also be
    sent manually using :py:meth:`send_update_message`.
    This class can be used as a context manager.

    You can update the frequency of the messages by changing :py:attr:`frequency`, or the outgoing frequency by
    changing :py:attr:`duration`. It will be updated with the next hello message. You can also send one manually
    with :py:meth:`send_update_message`.
    """
    def __init__(self, server_ip: str, server_port: int, frequency: float = 1, duration: float = 30):
        """Create the DummyUDPClient.

        Args:
            server_ip: The IP of the server we are requesting messages from.
            server_port: The port on which the server is listening for requests.
            frequency: How often we would like to receive messages from the server, in Hz.
            duration: How long the server should send us messages after each request, in seconds.
        """
        self.frequency: float = frequency  #: How often should the server send messages to us.
        self.duration: float = duration  #: How long the server should send us info without a message from us.
        self.max_listen_time: float = 3  #: Timeout on the listening function in seconds. Necessary to prevent lock-ups.

        self.json_output: Any = None  #: The latest message from the server, as a json object.
        self.time_of_last = -1  #: The time we received the last message from the server.

        # Socket stuff
        self.socket: socket.socket | None = None  #: The socket we use. None until :py:meth:`start` is called.
        self.target: tuple[str, int] = (server_ip, server_port)  #: The address of the server.

        #: A set of running tasks which should be canceled on shutdown.
        self.running_tasks: set[Future | asyncio.Future] = set()
        #: Whether we are receiving messages. We send requests more often while waiting for the server to respond.
        self.receiving_messages: bool = False

    def start(self):
        """Start up the client.

        Initializes the socket, starts the receiving task and starts sending requests to the server.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 0))
        self.socket = sock
        hello_task = asyncio.create_task(self._send_hello_messages())
        self.running_tasks.add(hello_task)
        listen_task = asyncio.create_task(self.receive())
        self.running_tasks.add(listen_task)

    def __enter__(self) -> "DummyUDPClient":
        """Magic method to allow this to be used as a context manager.

        Returns:
            This instance.
        """
        return self

    def close(self):
        """Shutdown the client, canceling all tasks and closing the socket."""
        cancel_running_tasks(self.running_tasks)
        self.socket.close()

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_value: BaseException | None,
                 exc_tb: TracebackType | None):
        """Magic method for exiting the context manager.

        Calls :py:meth:`close`.

        Args:
            exc_type: ``None`` if we exited normally or the type of the exception if we exited due to an exception.
            exc_value: ``None`` if we exited normally or the exception instance if we exited due to an exception.
            exc_tb: ``None`` if we exited normally or the traceback for the exception.
        """
        self.close()

    def send_update_message(self):
        """Send a request to the server."""
        print(f"Sending update message. New frequency: {self.frequency} New keep-alive duration: {self.duration}")
        msg = json.dumps({"duration": self.duration, "frequency": self.frequency})
        self.socket.sendto(msg.encode("utf-8"), self.target)

    async def _send_hello_messages(self):
        """Automatically sends a new request if the previous one is about to expire."""
        next_msg_time = time.time()
        while True:
            prev_msg_time = next_msg_time
            try:
                print(f"Sending {'initial' if not self.receiving_messages else 'recurring'} hello message...")
                msg = json.dumps({"duration": self.duration, "frequency": self.frequency})
                self.socket.sendto(msg.encode("utf-8"), self.target)
            except Exception as e:
                print("Exception sending initial hello packet! ", repr(e))
            # Send every 1 second if we're not getting information yet, otherwise more rarely
            if not self.receiving_messages:
                await asyncio.sleep(1)
            else:
                msg_interval = self.duration * 0.8
                next_msg_time = prev_msg_time + msg_interval
                wait_to_next_msg = max(0, next_msg_time - time.time())
                await asyncio.sleep(wait_to_next_msg)

    async def receive(self):
        """Listens for messages from DroneManager.

        Receives messages from DroneManager, prints them to console and stores the content and updates
        :py:attr:`receiving_messages` and :py:attr:`time_of_last`.
        """
        while True:
            try:
                msg = await asyncio.wait_for(asyncio.get_running_loop().sock_recv(self.socket, 1024),
                                             self.max_listen_time)
                json_str = json.loads(msg)
                self.json_output = json_str
                self.receiving_messages = True
                self.time_of_last = time.time()
                print(time.time(), json_str)
            except (TimeoutError, asyncio.TimeoutError):
                self.receiving_messages = False
                print("No messages from DroneManager...")
                await asyncio.sleep(0)
            except Exception as e:
                print("Exception receiving data over UDP! ", repr(e))
                await asyncio.sleep(0)  # Prevent getting stuck in the loop when we get this exception


async def frequency_example(ip: str, port: int):
    """Example script showing the frequency of sent messages being updated.

    Args:
        ip: The IP of the server.
        port: The port of the server.
    """
    with DummyUDPClient(ip, port, frequency=1, duration=20) as receiver:
        receiver.start()
        await asyncio.sleep(10)
        print("Requesting updates with 2 Hz instead, waiting for natural update message.")
        print("Frequency will not update until the natural refresh in a few seconds.")
        receiver.frequency = 2
        await asyncio.sleep(15)
        print("Requesting updates with 5 Hz, sending update message immediately.")
        print("Messages are sent with new frequency immediately.")
        receiver.frequency = 5
        receiver.send_update_message()
        await asyncio.sleep(5)
    print("Done")


async def passive_listener(ip: str, port: int):
    """Example script just echoing the messages from DroneManager until shutdown.

    Args:
        ip: The IP of the server.
        port: The port of the server.
    """
    with DummyUDPClient(ip, port) as receiver:
        receiver.frequency = 5
        receiver.duration = 60
        receiver.start()
        while True:
            await asyncio.sleep(60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dummy UDP client to show functioning of 'External' plugin of DM")
    parser.add_argument("address", type=str, default="127.0.0.1", nargs="?",
                        help="IP address running DM. Default localhost.")
    parser.add_argument("--port", type=int, default=DEFAULT_SERVER_PORT, required=False,
                        help=f"Port on which the server is listening. Default {DEFAULT_SERVER_PORT}. The server port is"
                             f"fixed in DM, so this argument is only necessary if you changed the configuration.")
    parser.add_argument("-e", "--example", action="store_true",
                        help="If this flag is set, instead of trying to connect and request indefinitely, we run "
                             "through an example on the message frequencies and update messages.")
    args = parser.parse_args()

    if args.example:
        asyncio.run(frequency_example(args.address, args.port))
    else:
        asyncio.run(passive_listener(args.address, args.port))
