"""Module to test core DroneManager functionality."""
from dronemanager.utils import get_config, heading_ned, heading_gps, offset_from_gps, ned_from_gps, get_free_port, \
    parse_address
from dronemanager.core import DMConfig, DroneManager


def test_config():
    """Test that the config can be loaded."""
    conf = DMConfig.from_file(get_config().as_posix())
    assert conf.mav_system_id is not None


async def test_dm_plugin_loading(dm: DroneManager):
    """Test loading each plugin.

    Args:
        dm: DroneManager instance.
    """
    available_but_not_loaded = [item for item in dm.plugin_options() if item not in dm.currently_loaded_plugins()]
    for plugin in available_but_not_loaded:
        await dm.load_plugin(plugin)
    await dm.close()


def test_heading_ned():
    """Test heading ned."""
    assert abs(heading_ned([10, 10, 3], [10, 20, 5]) - 90) < 1e-6
    assert abs(heading_ned([10, 10, 3], [20, 10, 5]) - 0) < 1e-6
    assert abs(heading_ned([10, 10, 3], [5, 15, 5]) - 135) < 1e-6
    assert abs(heading_ned([10, 10, 3], [0, 0, 5]) + 135) < 1e-6
    assert abs(heading_ned([10, 10, 3], [15, 5, 5]) + 45) < 1e-6


def test_heading_gps():
    """Test gps heading."""
    assert abs(heading_gps([0, 0, 3], [0, 0.1, 5]) - 90) < 1e-3
    assert abs(heading_gps([0, 0, 3], [0.1, 0, 5]) - 0) < 1e-3
    assert abs(heading_gps([0, 0, 3], [-0.1, 0.1, 5]) - 135) < 1e-3
    assert abs(heading_gps([0, 0, 3], [-0.1, -0.1, 5]) + 135) < 1e-3
    assert abs(heading_gps([0, 0, 3], [0.1, -0.1, 5]) + 45) < 1e-3
    # Test break points
    assert abs(heading_gps([0, 359.95, 3], [0, 0.05, 5]) - 90) < 1e-3
    assert abs(heading_gps([89.9, 0, 3], [89.9, 180, 5])) < 1e-3


def test_offset_gps():
    """Test gps offsetting function."""
    new_point = offset_from_gps([0, 0, 0], [0, 0, 100], [1, 1, 200])
    assert abs(new_point[0] - 1) < 1e6 and abs(new_point[1] - 1) < 1e6 and abs(new_point[2] - 100) < 1e6
    new_point = offset_from_gps([89.9, 0, 0], [0, 0, 100], [1, 0, 200])
    assert abs(new_point[0] - 89.1) < 1e6 and abs(new_point[1] % 180 - 180) < 1e6 and abs(new_point[2] - 100) < 1e6


def test_ned_from_gps():
    """Test computing NED distances from GPS coordinates."""
    ned = ned_from_gps([0, 0, 0], [1, 1, 100])
    assert abs(ned[0] - 111100)/111100 < 1e6 and abs(ned[1] - 111100)/111100 < 1e6 and abs(ned[2] - 100) < 1e6
    ned = ned_from_gps([45, 0, 0], [46, 1, 100])
    assert abs(ned[0] - 78567)/78567 < 1e6 and abs(ned[1] - 111100)/111100 < 1e6 and abs(ned[2] - 100) < 1e6
    ned = ned_from_gps([75, 0, 0], [76, 1, 100])
    assert abs(ned[0] - 28758)/28758 < 1e6 and abs(ned[1] - 111100)/111100 < 1e6 and abs(ned[2] - 100) < 1e6


def test_free_port():
    """Test function to get a free port."""
    port = get_free_port()
    assert port is not None


def test_parse_address():
    """Test address parsing function."""
    scheme, adr, loc = parse_address("udp://192.168.0.10:14540")
    assert scheme == "udp" and adr == "192.168.0.10" and loc == 14540
    scheme, adr, loc = parse_address("udp://:14540")
    assert scheme == "udp" and adr == "" and loc == 14540
    scheme, adr, loc = parse_address("udp://127.0.0.1:14540")
    assert scheme == "udp" and adr == "" and loc == 14540
    scheme, adr, loc = parse_address("serial://COM5:57600")
    assert scheme == "serial" and adr == "COM5" and loc == 57600
