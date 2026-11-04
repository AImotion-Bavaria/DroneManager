"""Tests for the MAVSDK comm protocol and related implementing classes."""
import asyncio
from unittest.mock import Mock

from mavsdk.asyncio.plugins.mocap import MocapError

from dronemanager.core import DroneManager
from dronemanager.plugins.NatNet.MoCapData import generate_mocap_data


async def test_optitrack_errors(dm_with_optitrack_mock: tuple[DroneManager, Mock],
                                mock_drone_getter):
    """Tests assorted error handling components of the plugin.

    Args:
        dm_with_optitrack_mock: The DroneManager instance with loaded optitrack plugin.
        mock_mavsdkdrone_getter: A function to get mocked drone objects for tests.
    """
    mock_drone = mock_drone_getter(1)
    dm, client = dm_with_optitrack_mock
    optitrack = getattr(dm, "optitrack")

    dm.drones[mock_drone.name] = mock_drone
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
    await optitrack.add_drone(mock_drone.name, 0)

    # Test Mocap errors.
    mock_drone.system.mocap.set_vision_position_estimate.side_effect = \
        MocapError(2, "Test exception, please ignore")
    frame_counter = do_callback(frame_counter)
    await asyncio.sleep(0.1)  # Need a little sleep so the rigid frame processing can happen.
    assert optitrack._err_count == 1

    # Remove the drone and test that callback removes it properly.
    await dm.disconnect(mock_drone.name)
    assert len(optitrack._drone_id_mapping) == 1
    do_callback(frame_counter)
    await asyncio.sleep(0.1)
    assert len(optitrack._drone_id_mapping) == 0
