"""Module to test the terminal interface."""
from dronemanager.core import DroneManager
from dronemanager.app import DroneApp


def test_smoke(dm: DroneManager):
    """Test that the app loads.

    Args:
        dm: DroneManager instance.
    """
    app = DroneApp(dm, logger=dm.logger, smoke_test=True)
    app.run(headless=True)
