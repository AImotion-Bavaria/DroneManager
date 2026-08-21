"""Tests for the external plugin."""
import asyncio
import math
import pytest
import time
from typing import AsyncGenerator, Any
from unittest.mock import Mock

from dronemanager.core import DroneManager
from dronemanager.plugins.external import DummyUDPClient, DEFAULT_SERVER_PORT, passive_listener, frequency_example


@pytest.fixture
async def dm_for_external(dm: DroneManager) -> AsyncGenerator[DroneManager, Any]:
    """Create a DroneManager object for the external tests.

    Args:
        dm: The DroneManager instance for the test.

    Yields:
        A DroneManager instance with the external plugin already loaded.
    """
    await dm.load("external")
    yield dm


async def test_passive_listener(dm_for_external: DroneManager):
    """Test the passive listener function, by just executing it for a little while.

    Args:
        dm_for_external: The DroneManager instance for the test.
    """
    listener_task = asyncio.create_task(passive_listener("127.0.0.1", DEFAULT_SERVER_PORT))
    try:
        await asyncio.wait_for(listener_task, timeout=5)
    except asyncio.TimeoutError:
        pass


async def test_frequency_example(dm_for_external: DroneManager):
    """Test the frequency example function by running it.

    Args:
        dm_for_external: The DroneManager instance for the test.
    """
    assert hasattr(dm_for_external, "external")
    await frequency_example("127.0.0.1", DEFAULT_SERVER_PORT)


async def test_dummy_client(dm_for_external: DroneManager, mock_drone: Mock):
    """Test the dummy client and UDP plugin in one.

    Starts a client, sends messages to the plugin and checks that the client is receiving messages and the plugin is
    registering the client. Also verifies that messages are sent with approximately the right frequency.
    Also loads a mission and adds a mock drone to verify that messages can be JSON serialized.

    Args:
        dm_for_external: The DroneManager instance for the test.
        mock_drone: A mock drone object for the test
    """
    # Load a mission and add a drone.
    await dm_for_external.load("mission")
    assert hasattr(dm_for_external, "mission")
    mission_plugin = getattr(dm_for_external, "mission")
    external_plugin = getattr(dm_for_external, "external")
    await mission_plugin.load("engel")
    assert hasattr(dm_for_external, "engel")
    dm_for_external.drones["mock"] = mock_drone

    with DummyUDPClient("127.0.0.1", DEFAULT_SERVER_PORT) as client:
        client.frequency = 5
        client.start()
        await asyncio.sleep(2)  # Short sleep for connection to happen
        assert client.receiving_messages
        assert len(external_plugin.clients) == 1
        # For 2 second, check that the frequency matches
        for _ in range(math.ceil(client.frequency * 2)):
            assert time.time() - client.time_of_last < 1.1 / client.frequency

        # Update frequency and check that actual frequency matches updated.
        client.frequency = 10
        client.send_update_message()
        await asyncio.sleep(0.5)
        for _ in range(math.ceil(client.frequency * 2)):
            assert time.time() - client.time_of_last < 1.1 / client.frequency
        assert len(external_plugin.clients) == 1

        # Test shutting down the UDPPlugin and checking DummyUDPClient behaviour.
        await dm_for_external.close()
        await asyncio.sleep(client.max_listen_time + 1)
        assert not client.receiving_messages
