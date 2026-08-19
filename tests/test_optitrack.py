"""Tests for the optitrack plugin."""
from typing import AsyncGenerator, Any
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unittest.mock import Mock, patch

from dronemanager.core import DroneManager
from dronemanager.plugins.optitrack import CoordinateConversion

import logging


def test_coordinate_conversion():
    """Test for the coordinate conversion system.

    Note that generally drone attitude is presented as RPY but applied as YPR.
    """
    test_angles = [
        np.asarray([30, 0, 0]),
        np.asarray([0, -80, 0]),
        np.asarray([0, 0, 30]),
        np.asarray([10, 90, 30])
    ]
    test_pos = np.asarray([1, 2, 3])

    logging.info("First lab coordinate system test")
    conv = CoordinateConversion("z", "-x", "-y", "x", "z", "-y")
    target_angles = [
        np.asarray([30, 0, -90]),
        np.asarray([0, 0, -10]),
        np.asarray([0, 30, -90]),
        np.asarray([0, 40, -180])
    ]
    target_pos = np.asarray([3, -1, -2])
    for i in range(len(test_angles)):
        _test_conversion(conv, test_angles[i], target_angles[i], test_pos, target_pos)

    logging.info("Second lab version test")
    conv = CoordinateConversion("x", "z", "-y", "x", "z", "-y")
    target_angles = [
        np.asarray([30, 0, 0]),
        np.asarray([0, 0, 80]),
        np.asarray([0, 30, 0]),
        np.asarray([0, 40, -90])
    ]
    target_pos = np.asarray([1, 3, -2])
    for i in range(len(test_angles)):
        _test_conversion(conv, test_angles[i], target_angles[i], test_pos, target_pos)

    logging.info("Conversion unity test")
    conv = CoordinateConversion("x", "y", "z", "x", "y", "z")
    target_angles = [
        np.asarray([30, 0, 0]),
        np.asarray([0, -80, 0]),
        np.asarray([0, 0, 30]),
        np.asarray([90, 50, 90])
    ]
    target_pos = np.asarray([1, 2, 3])
    for i in range(len(test_angles)):
        _test_conversion(conv, test_angles[i], target_angles[i], test_pos, target_pos)


def _test_conversion(conversion: CoordinateConversion,
                     test_angles: np.ndarray, target_angles: np.ndarray,
                     test_pos: np.ndarray, target_pos: np.ndarray):
    """Performs a single coordinate conversion test.

    Converts the input position and angles and compares them against the target values. Optitrack uses quats, but
    euler angles are more intuitive to understand, so we start from euler angles and create a quat from them.
    All angles are in degrees.
    Tests fail if the absolute error is too large.

    Args:
        conversion: The CoordinateConversion object being tested.
        test_angles: The input angles.
        target_angles: The target output angles.
        test_pos: The input position.
        target_pos: The target output position.
    """
    rot: Rotation = Rotation.from_euler("XYZ", test_angles, degrees=True)
    rot_quat = rot.as_quat()
    conv_pos, conv_rot = conversion.convert_quat(test_pos, rot_quat, "xyz", degrees=True)
    pos_e = np.max(np.abs(conv_pos - target_pos))
    rot_e = np.max(np.abs(conv_rot - target_angles))
    logging.error(f"Testing angles {test_angles, target_angles, conv_rot}")
    assert pos_e < 1e-6, "Position error in coordinate conversion too large!"
    assert rot_e < 1e-6, "Rotation conversion error too large!"


@pytest.fixture
async def dm_with_optitrack_mock(dm: DroneManager) -> AsyncGenerator[tuple[DroneManager, Mock], Any]:
    """Create a DroneManager instance with the optitrack plugin already loaded

    Args:
        dm:

    Returns:

    """
    with patch("dronemanager.plugins.optitrack.NatNetClient") as NatNetMock:
        await dm.load_plugin("optitrack")
        client = NatNetMock.return_value
        out = dm, client
        yield out


async def test_optitrack_natnet_run_failure(dm_with_optitrack_mock):
    """Tests for the full optitrack plugin.

    Args:
        dm_with_optitrack_mock:

    Returns:

    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Test run not working
    client.run.return_value = False
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is False
    assert optitrack.client is None
    client.shutdown.assert_called_once()


async def test_optitrack_natnet_no_server(dm_with_optitrack_mock):
    """Tests for the full optitrack plugin.

    Args:
        dm_with_optitrack_mock:

    Returns:

    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Test run not working
    client.run.return_value = True
    client.connected.return_value = False
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is False
    assert optitrack.client is None
    client.shutdown.assert_called_once()


async def test_optitrack_natnet_exception(dm_with_optitrack_mock):
    """Tests for the full optitrack plugin.

    Args:
        dm_with_optitrack_mock:

    Returns:

    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Testing run exception
    client.run.side_effect = ConnectionResetError("Test exception, please ignore.")
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is False
    assert optitrack.client is None
    client.shutdown.assert_called_once()


async def test_optitrack_connect(dm_with_optitrack_mock):
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Test run not working
    client.run.return_value = True
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is True
    assert optitrack.client is client
