"""Module to test the terminal interface."""
from dronemanager.core import DroneManager
from dronemanager.app import DroneApp
from dronemanager.drone import DroneMAVSDK


def test_smoke(dm: DroneManager):
    """Test that the app loads.

    Args:
        dm: DroneManager instance.
    """
    app = DroneApp(dm, logger=dm.logger, smoke_test=True)
    app.run(headless=True)


async def test_control_screen(dm: DroneManager):
    """Test the CLI and key commands.

    Args:
        dm: DroneManager instance.
    """
    app = DroneApp(dm, logger=dm.logger, smoke_test=False)

    drone = DroneMAVSDK("tom")
    dm.drones["tom"] = drone

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

        # Test rone widget
        assert screen is not None
        assert "tom" not in screen.drone_widgets
        await screen._add_drone_object("tom", drone)
        await pilot.pause()
        assert "tom" in screen.drone_widgets
        widget = screen.drone_widgets["tom"]
        assert widget.is_attached
        await pilot.pause()
        await screen._remove_drone_object("tom")
        await pilot.pause()
        assert "tom" not in screen.drone_widgets

        # Test CLI
        cli = screen.query_one("#cli")
        cli.focus()

        await enter_command("mission-load engel")
        await pilot.pause()
        await enter_command("fake-command which does not exist")
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

        # Exit
        await enter_command("exit")
        await pilot.pause(3)
        assert app.is_running is False
