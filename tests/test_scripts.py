"""Tests for the scripts plugin.

Executes the dummy script.
"""
import os

from dronemanager.core import DroneManager
from dronemanager.plugins.scripts import _script_function, SCRIPT_DIR


def test_script_function():
    """Test specifically the script function."""
    path = os.path.join(SCRIPT_DIR.as_posix(), "dummy_script.py")
    res = _script_function(path, ["--success"])
    assert res.returncode == 0
    assert res.stdout == "Success!\n"


async def test_script(dm: DroneManager):
    """Test scripts plugin.

    Args:
        dm: DroneManager instance.
    """
    await dm.load_plugin("scripts")
    arg_success = "--success"
    arg_fail = "--fail"
    res = await dm.scripts.execute_script("dummy_script.py", [arg_success])
    assert res.returncode == 0
    assert res.stdout == "Success!\n"

    res = await dm.scripts.execute_script("dummy_script.py", [arg_fail])
    assert res.returncode == 1
    assert res.stderr == "Failure!"
