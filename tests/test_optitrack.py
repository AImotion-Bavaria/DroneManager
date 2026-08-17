"""Tests for the optitrack plugin."""
import numpy as np
from scipy.spatial.transform import Rotation

from dronemanager.core import DroneManager
from dronemanager.plugins.optitrack import CoordinateConversion


def test_coordinate_conversion():
    conv = CoordinateConversion("z", "-x", "-y")

    angles = [0, 45, 45]
    pos = [1, 2, 3]
    rot: Rotation = Rotation.from_euler(conv.rotation_sequence, angles, degrees=True)
    rot_quat = rot.as_quat()

    conv_pos_e, conv_rot_e = conv.convert_euler(pos, angles, "ZYX", degrees=True, in_degrees=True)
    conv_pos_q, conv_rot_q = conv.convert_quat(pos, rot_quat, "ZYX", degrees=True)
    assert False, f"{pos, conv_pos_e, conv_pos_q}, {angles, conv_rot_e, conv_rot_q}"


async def test_optitrack_plugin(dm: DroneManager):
    res = await dm.load_plugin("optitrack")
    assert res is not None
