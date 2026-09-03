"""Tests for the scripts plugin.

Executes the dummy script.
"""
import os

from dronemanager.core import DroneManager
from dronemanager.plugins.scripts import _script_function, SRC_SCRIPT_PATH


def test_script_function():
    """Test specifically the script function."""
    path = SRC_SCRIPT_PATH.joinpath("dummy_script.py").as_posix()
    res = _script_function(path, ["--success"])
    assert res.returncode == 0
    assert res.stdout == "Success!\n"


async def test_script(dm: DroneManager):
    """Test scripts plugin.

    Args:
        dm: DroneManager instance.
    """
    await dm.load("scripts")
    arg_success = "--success"
    arg_fail = "--fail"
    scripts = getattr(dm, "scripts")
    res = await scripts.execute_script("dummy_script.py", [arg_success])
    assert res.returncode == 0
    assert res.stdout == "Success!\n"

    res = await scripts.execute_script("dummy_script.py", [arg_fail])
    assert res.returncode == 1
    assert res.stderr == "Failure!"
