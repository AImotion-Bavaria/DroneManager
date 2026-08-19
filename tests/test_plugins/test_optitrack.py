"""Tests for the optitrack plugin."""
import asyncio
import logging
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from typing import AsyncGenerator, Any
from unittest.mock import Mock, patch

from dronemanager.core import DroneManager
from dronemanager.drone import MocapError
from dronemanager.plugins.NatNet.MoCapData import generate_mocap_data
from dronemanager.plugins.optitrack import CoordinateConversion


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
    """Create a DroneManager instance with the optitrack plugin already loaded.

    Args:
        dm: A plain DroneManager instance.

    Yields:
        A DroneManager instance with loaded optitrack plugin and the mocked NatNetClient object.
    """
    with patch("dronemanager.plugins.optitrack.NatNetClient") as NatNetMock:
        await dm.load_plugin("optitrack")
        client = NatNetMock.return_value
        out = dm, client
        yield out


async def test_optitrack_natnet_run_failure(dm_with_optitrack_mock: tuple[DroneManager, Mock]):
    """Tests for the server connection procedure optitrack plugin.

    Tests "run" function failure.

    Args:
        dm_with_optitrack_mock: The test DroneManager instance.
    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Test run not working
    client.run.return_value = False
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is False
    assert optitrack.client is None
    client.shutdown.assert_called_once()


async def test_optitrack_natnet_no_server(dm_with_optitrack_mock: tuple[DroneManager, Mock]):
    """Tests for the server connection procedure optitrack plugin.

    Tests behaviour if there is no server to connect to.

    Args:
        dm_with_optitrack_mock: The test DroneManager instance.
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


async def test_optitrack_natnet_exception(dm_with_optitrack_mock: tuple[DroneManager, Mock]):
    """Tests for the server connection procedure optitrack plugin.

    Tests a ConnectionResetException during connection.

    Args:
        dm_with_optitrack_mock: The test DroneManager instance.
    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Testing run exception
    client.run.side_effect = ConnectionResetError("Test exception, please ignore.")
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is False
    assert optitrack.client is None
    client.shutdown.assert_called_once()


async def test_optitrack_connect(dm_with_optitrack_mock: tuple[DroneManager, Mock]):
    """Tests for the server connection procedure optitrack plugin.

    Tests a successful connect.

    Args:
        dm_with_optitrack_mock: The test DroneManager instance.
    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Test run not working
    client.run.return_value = True
    res = await optitrack.connect_server(remote="127.0.0.1", local="127.0.0.1")
    assert res is True
    assert optitrack.client is client


async def test_rigid_body_processing(dm_with_optitrack_mock: tuple[DroneManager, Mock], mock_drone_connected: Mock):
    """Tests core functionality around processing Motive data.

    Args:
        dm_with_optitrack_mock: The DroneManager instance with loaded optitrack plugin.
        mock_drone_connected: The mocked drone object to be used during the tests.
    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    # Set specific coordinate conversion to not get messed up by config.
    await optitrack.set_coordinates_body("x", "z", "-y")
    await optitrack.set_coordinates_world("z", "-x", "-y")

    dm.drones["mock"] = mock_drone_connected

    assert len(optitrack.available_bodies) == 0
    frame_counter = 0

    def do_callback(frame_count: int) -> int:
        """Do a single callback with dummy mocap data.

        Args:
            frame_count: Frame count.

        Returns:
            frame count incremented by one.
        """
        data_dict = {"mocap_data": generate_mocap_data(frame_count)}
        frame_count += 1
        # Callback without drones
        optitrack._new_frame_callback(data_dict)
        return frame_count

    frame_counter = do_callback(frame_counter)
    assert len(optitrack.available_bodies) == 3

    # Test the check-conv function and check that conversion are happening properly from mocap data.
    await optitrack.check_conv(0)
    position, rotation = optitrack.available_bodies[0]
    target_pos = np.asarray([position[2], -position[0], -position[1]])
    target_rotation = np.asarray([-180, 0, -90])
    out_pos, out_rot = optitrack.coordinate_transform.convert_quat(position, rotation, out_sequence="xyz", degrees=True)
    assert np.max(np.abs(out_pos - target_pos)) < 1e-6 and np.max(np.abs(out_rot - target_rotation)) < 1e-6

    # Callback with drone
    await optitrack.add_drone("mock", 0)
    assert len(optitrack._drone_id_mapping) == 1
    frame_counter = do_callback(frame_counter)
    await asyncio.sleep(0.1)  # Need a little sleep so the rigid frame processing can happen.
    mock_drone_connected.send_external_tracking_data.assert_called_once()

    # Remove drone and check callback
    await optitrack.remove_drone("mock")
    assert len(optitrack._drone_id_mapping) == 0
    frame_counter = do_callback(frame_counter)
    await asyncio.sleep(0.1)  # Need a little sleep so the rigid frame processing can happen.
    mock_drone_connected.send_external_tracking_data.assert_called_once()


async def test_optitrack_errors(dm_with_optitrack_mock: tuple[DroneManager, Mock], mock_drone_connected: Mock):
    """Tests assorted error handling components of the plugin.

    Args:
        dm_with_optitrack_mock: The DroneManager instance with loaded optitrack plugin.
        mock_drone_connected: The mocked drone object to be used during the tests.
    """
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    dm.drones["mock"] = mock_drone_connected
    frame_counter = 0

    def do_callback(frame_count: int) -> int:
        """Do a single callback with dummy mocap data.

        Args:
            frame_count: Frame count.

        Returns:
            frame count incremented by one.
        """
        data_dict = {"mocap_data": generate_mocap_data(frame_count)}
        frame_count += 1
        # Callback without drones
        optitrack._new_frame_callback(data_dict)
        return frame_count

    frame_counter = do_callback(frame_counter)  # Do the callback once so we have tracks
    await optitrack.add_drone("mock", 0)

    # Test Mocap errors.
    mock_drone_connected.system.mocap.set_vision_position_estimate.side_effect = \
        MocapError(2, "Test exception, please ignore")
    frame_counter = do_callback(frame_counter)
    await asyncio.sleep(0.1)  # Need a little sleep so the rigid frame processing can happen.
    assert optitrack._err_count == 1

    # Remove the drone and test that callback removes it properly.
    await dm.disconnect("mock")
    assert len(optitrack._drone_id_mapping) == 1
    frame_counter = do_callback(frame_counter)
    await asyncio.sleep(0.1)
    assert len(optitrack._drone_id_mapping) == 0
