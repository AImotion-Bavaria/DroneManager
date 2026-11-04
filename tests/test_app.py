"""Module to test the terminal interface."""
import asyncio

from dronemanager.app import DroneApp
from dronemanager.core import DroneManager


def test_smoke(dm: DroneManager):
    """Test that the app loads.

    Args:
        dm: DroneManager instance.
    """
    app = DroneApp(dm, logger=dm.logger, smoke_test=True)
    app.run(headless=True)


async def test_control_screen(dm: DroneManager, mock_drone_getter):
    """Test the CLI and key commands.

    Args:
        dm: DroneManager instance.
    """
    app = DroneApp(dm, logger=dm.logger, smoke_test=False)
    mock_drone = mock_drone_getter(1)

    dm.drones["mock_1"] = mock_drone

    async with app.run_test(size=(120, 40)) as pilot:
        # The app has mounted and is running here.
        assert app.current_mode == "control"
        screen = app.screen

        async def enter_command(command: str):
            """Enter a command into the CLI.

            Args:
                command: Command to enter
            """
            for char in command:
                await pilot.press(char)
            await pilot.press("enter")
            await pilot.pause()

        # Test drone widget
        assert screen is not None
        assert "mock_1" not in screen.drone_widgets
        await screen._add_drone_object("mock_1", mock_drone)
        await pilot.pause()
        assert "mock_1" in screen.drone_widgets
        widget = screen.drone_widgets["mock_1"]
        assert widget.is_attached
        await pilot.pause()
        await screen._remove_drone_object("mock_1")
        await pilot.pause(1)
        assert "mock_1" not in screen.drone_widgets

        # Test CLI
        cli = screen.query_one("#cli")
        cli.focus()

        await enter_command("mission-load engel")
        await pilot.pause()
        await enter_command("fake-command which does not exist")
        mock_drone.batteries[0].remaining = 50
        await pilot.pause()
        await enter_command("connect")
        await pilot.pause()
        await enter_command("move mock_1 10 3 5 --fake-argument")
        await pilot.pause()
        await enter_command("move mock_1 shf seh sfhjd")
        mock_drone.batteries[0].remaining = 20
        await pilot.pause()
        await pilot.press("up")
        await pilot.pause()
        await pilot.press("up")
        await pilot.pause()
        await pilot.press("down")
        await pilot.pause()
        await pilot.press("enter")
        await pilot.pause()
        await enter_command("--help")
        await pilot.pause()

        # mockdrone disarmed and on the ground
        mock_drone.is_armed = False
        mock_drone.in_air = False
        await pilot.pause()

        # Exit
        await enter_command("exit")
        await pilot.pause(3)
        assert app.is_running is False
