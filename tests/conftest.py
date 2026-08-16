"""Module for pytest fixtures used by many tests."""
import pytest
import pygame
from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK


pygame.init()


@pytest.fixture
def dm() -> DroneManager:
    """Create DroneManager object for tests.

    Returns:
        A DroneManager instance.
    """
    drone_type = DroneMAVSDK
    dm = DroneManager(drone_type, log_to_console=False)
    return dm
