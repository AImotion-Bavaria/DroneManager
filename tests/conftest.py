"""Module for pytest fixtures used by many tests."""
import asyncio
from typing import AsyncGenerator, Any
from unittest.mock import Mock, AsyncMock

import cv2
import numpy as np
import pytest
import pygame
import struct

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK
from mavsdk import System

import logging


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
def mock_drone_object() -> Mock:
    """Fixture for a mock drone object.

    Mocks a bunch of the individual components

    Returns:
        A Mock object specced to and wrapping DroneMAVSDK
    """
    # Core mocks
    mockdrone = Mock(spec=DroneMAVSDK)
    mockdrone.disconnect = AsyncMock()
    mockdrone.stop_execution = AsyncMock()
    mockdrone.system = Mock(spec=System)

    # Optitrack mocks
    mockdrone.system.mocap = Mock()
    mockdrone.system.mocap.set_vision_position_estimate = AsyncMock()

    return mockdrone


@pytest.fixture
def mock_drone_connected(mock_drone_object: Mock) -> Mock:
    """Fixture for a connected mock drone object.

    Args:
        mock_drone_object: A mock object that has not been "instantiated".

    Returns:
        A Mock object specced to and wrapping DroneMAVSDK
    """
    # Core mocks
    mock_drone_object.is_connected = True
    return mock_drone_object
