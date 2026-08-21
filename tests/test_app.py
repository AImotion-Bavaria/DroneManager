"""Module to test the terminal interface."""
from dronemanager.drone import DroneMAVSDK
from dronemanager.core import DroneManager
from dronemanager.app import DroneApp


def test_smoke():
    """Test that the app loads."""
    drone_type = DroneMAVSDK
    drone_manager = DroneManager(drone_type, log_to_console=False)
    app = DroneApp(drone_manager, logger=drone_manager.logger, smoke_test=True)
    app.run(headless=True)
