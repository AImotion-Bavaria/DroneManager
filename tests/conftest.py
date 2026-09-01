"""Module for pytest fixtures used by many tests."""
import asyncio
import logging
import numpy as np
import pytest
import pygame
import struct
from typing import AsyncGenerator, Any, Callable
from unittest.mock import Mock, AsyncMock

import cv2
from mavsdk import System

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK, DroneConfig, FlightMode, DroneParams
from dronemanager.navigation.core import PathGenerator, PathFollower, Waypoint, WayPointType
from dronemanager.navigation.rectlocalfence import RectLocalFence


pygame.init()


@pytest.fixture
async def dm() -> AsyncGenerator[DroneManager, Any]:
    """Create DroneManager object for tests.

    Yields:
        A DroneManager instance.
    """
    drone_type = DroneMAVSDK
    dm = DroneManager(drone_type, log_to_console=False)
    yield dm
    await dm.close()


class TCPStreamer:
    """A dummy TCP video streaming server.

    Listens for clients and sends random images of the set size to them at the set frequency.
    """
    def __init__(self):
        """Create the TCPStreamer instance."""
        self.ip: str = "127.0.0.1"  #: IP for the server.
        self.port: int = 5000  #: Port for the server
        self.frequency: float = 5  #: Frequency of the images.
        self.img_size = (360, 240, 3)  #: Size of the dummy image.

    async def start(self):
        """Start the TCP server."""
        logging.info("Starting TCPStreamer server.")
        server = await asyncio.start_server(self.send_images, self.ip, self.port)
        async with server:
            await server.serve_forever()

    async def send_images(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Send images at the set frequency.

        The reader is currently not used.

        Args:
            reader: Incoming stream.
            writer: Outgoing stream.
        """
        try:
            while True:
                img_data = np.random.randint(0, 256, self.img_size, dtype=np.uint8)
                res, encoded = cv2.imencode(".png", img_data)
                if not res:
                    logging.error("Couldn't encode dummy image.")
                img_bytes = encoded.tobytes()
                img_length = struct.pack("<I", len(img_bytes))

                writer.write(img_length)
                writer.write(img_bytes)
                await writer.drain()

                await asyncio.sleep(1/self.frequency)
        except (asyncio.CancelledError, ConnectionAbortedError, ConnectionResetError):
            pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass


@pytest.fixture
async def video_stream_source() -> AsyncGenerator[TCPStreamer, Any]:
    """Creates a dummy video stream source.

    Yields:
        The dummy tcp image streamer.
    """
    server = TCPStreamer()
    task = asyncio.create_task(server.start())
    yield server
    task.cancel()


@pytest.fixture
def mock_drone_getter() -> Callable[[int], Mock]:
    """Fixture that allows creating an arbitrary number of distinct mock drone object.

    Mocks a bunch of the individual components for each drone

    Returns:
        A Mock object specced to and wrapping DroneMAVSDK
    """
    def _mock_drone_creator(count: int) -> Mock:
        """Drone creation function.

        Args:
            count: A unique number for the mock drone. Used to ensure name uniqueness.

        Returns:
            A mock drone object with name "mock_<count>"
        """
        name = f"mock_{count}"
        # Core mocks
        mockdrone = Mock(spec=DroneMAVSDK)
        mockdrone.disconnect = AsyncMock()
        mockdrone.stop_execution = AsyncMock()
        mockdrone.system = Mock(spec=System)
        mockdrone.name = name

        # Optitrack mocks
        mockdrone.send_external_tracking_data = AsyncMock()
        mockdrone.system.mocap.set_vision_position_estimate = AsyncMock()

        # Drone property mocks
        mockdrone.path_generator = Mock(spec=PathGenerator)
        mockdrone.path_generator.target_position = Waypoint(WayPointType.POS_NED, pos=[2, 3, -6], yaw=90)
        mockdrone.path_follower = Mock(spec=PathFollower)
        mockdrone.config = DroneConfig(name, address="dummy_address")
        mockdrone.position_ned = np.asarray([1, 1, -2], dtype=np.float64)
        mockdrone.position_global = np.asarray([-1, -1, 300], dtype=np.float64)
        mockdrone.velocity = np.asarray([0.1, 0.2, 0.3], dtype=np.float64)
        mockdrone.attitude = np.asarray([5.0, 10.0, 20.0], dtype=np.float64)
        mockdrone.flightmode = FlightMode.HOLD
        mockdrone.is_connected = True
        mockdrone.is_armed = True
        mockdrone.in_air = True
        mockdrone.fence = RectLocalFence(0, 3, -1, 4, -10, -1)
        mockdrone.drone_params = DroneParams()
        mockdrone.drone_params.max_h_vel = 3
        mockdrone.drone_params.max_up_vel = 1
        mockdrone.drone_params.max_down_vel = 1
        mockdrone.drone_params.max_yaw_rate = 30

        # Function mocks
        mockdrone.arm = AsyncMock()

        def flight_mode_change_posctrl():
            """Mock function which changes the flight mode posctrl."""
            mockdrone.flightmode = FlightMode.POSCTL

        def change_flight_mode_side_effect(flightmode: FlightMode):
            """Mock flight mode chaning function.

            Args:
                flightmode: The new flight mode.
            """
            mockdrone.flightmode = flightmode

        mockdrone.manual_control_position.side_effect = flight_mode_change_posctrl
        mockdrone.set_manual_control_input = AsyncMock()
        mockdrone.execute_task = AsyncMock()
        mockdrone.change_flight_mode.side_effect = change_flight_mode_side_effect

        return mockdrone
    return _mock_drone_creator
