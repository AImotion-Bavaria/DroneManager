"""Module to test core DroneManager functionality."""
import json
import pathlib

import pytest

from dronemanager.core import DMConfig, DroneManager
from dronemanager.utils import get_config, heading_ned, heading_gps, offset_from_gps, ned_from_gps, get_free_port, \
    parse_address, ConfigError, DEFAULT_CONFIG_FILE


def test_config():
    """Test that the config can be loaded."""
    conf = DMConfig.from_file(get_config().as_posix())
    assert conf.mav_system_id is not None


async def test_installed_config_matches_plugins(dm: DroneManager):
    """The installed configuration fits this version of DroneManager.

    Fails with the invalid entries if the configuration is from an older version, for example after an upgrade.

    Args:
        dm: DroneManager instance, using the installed configuration.
    """
    errors = dm.check_plugin_settings()
    assert not errors, "\n".join(str(error) for error in errors)


def _write_config(path: pathlib.Path, change: dict | None = None, remove: list[str] | None = None,
                  text: str | None = None) -> str:
    """Write a modified copy of the default configuration.

    Args:
        path: Directory for the file.
        change: Top-level entries to add or replace.
        remove: Top-level entries to remove.
        text: Raw file content, replaces the default configuration.

    Returns:
        The path of the configuration file.
    """
    config_file = path.joinpath("config.json")
    if text is None:
        config = json.loads(DEFAULT_CONFIG_FILE.read_text())
        config.update(change or {})
        for key in remove or []:
            config.pop(key)
        text = json.dumps(config)
    config_file.write_text(text)
    return config_file.as_posix()


@pytest.mark.parametrize("kwargs, message", [
    ({"text": '{"drones": [}'}, "JSON: Expecting value at line 1, column 13"),
    ({"remove": ["mav_system_id"]}, "mav_system_id: missing"),
    ({"change": {"mav_system_id": "246"}}, 'mav_system_id: must be an integer, not "246"'),
    ({"change": {"testing": 1}}, "testing: must be true or false, not 1"),
    ({"change": {"drones": {}}}, "drones: must be a list of drone configurations"),
    ({"change": {"drones": [{"drone_name": "tom"}]}}, "drones[0]: missing 'address'"),
    ({"change": {"plugin_settings": {"optitrack": ["x", "z", "-y"]}}},
     "plugin_settings.optitrack: must be an object with the plugin's arguments"),
])
def test_invalid_config_file(tmp_path: pathlib.Path, kwargs: dict, message: str):
    """An invalid configuration file raises a ConfigError naming the file and the problem.

    Args:
        tmp_path: Temporary directory.
        kwargs: How to change the default configuration.
        message: Expected part of the error message.
    """
    config_file = _write_config(tmp_path, **kwargs)
    with pytest.raises(ConfigError) as e:
        DMConfig.from_file(config_file)
    assert message in str(e.value)
    assert config_file in str(e.value)
    assert str(DEFAULT_CONFIG_FILE) in str(e.value)


@pytest.mark.parametrize("settings, message", [
    ({"optitrack": {"axes": ["z", "-x", "-y"]}},
     "plugin_settings.optitrack: unknown entry axes (valid: server_ip, local_ip, world_axes, body_axes, log_frames)"),
    ({"controllers": {"auto_drone": True, "auto_set": True}},
     "plugin_settings.controllers: unknown entries auto_drone, auto_set"),
    ({"controllers": {"mappings": {"PS4": {"name": "PS4"}}}}, "plugin_settings.controllers.mappings.PS4: missing"),
    ({"controllers": {"default_mappings": {"PS4 Controller": "PS4"}}},
     "plugin_settings.controllers.default_mappings.PS4 Controller: no mapping named 'PS4'"),
])
async def test_invalid_plugin_settings(dm: DroneManager, settings: dict, message: str):
    """Plugin settings that don't match the plugin are reported by the config check and when loading the plugin.

    Args:
        dm: DroneManager instance.
        settings: The invalid plugin settings.
        message: Expected part of the error message.
    """
    dm.config.plugin_settings = settings
    errors = dm.check_plugin_settings()
    assert len(errors) == 1
    assert message in str(errors[0])
    plugin = next(iter(settings))
    assert not await dm.load(plugin)
    assert plugin not in dm.plugins


async def test_unknown_default_plugin(dm: DroneManager):
    """A default plugin that doesn't exist is reported by the config check.

    Args:
        dm: DroneManager instance.
    """
    dm.config.default_plugins = ["missions"]
    dm.config.plugin_settings = {}
    errors = dm.check_plugin_settings()
    assert len(errors) == 1
    assert "default_plugins: no plugin named 'missions'" in str(errors[0])


async def test_dm_plugins(dm: DroneManager):
    """Test loading each plugin.

    Tests loading and common function for all plugins.

    Args:
        dm: DroneManager instance.
    """
    try:
        available_but_not_loaded = [item for item in dm.plugin_options if item not in dm.plugins]
        for plugin in available_but_not_loaded:
            res = await dm.load(plugin)
            assert res, f"Failed to load {plugin}!"
        for plugin in dm.plugins:
            plugin_obj = getattr(dm, plugin, None)
            if plugin_obj:
                if hasattr(plugin_obj, "status"):
                    await plugin_obj.status()
    finally:
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
    assert abs(new_point[0] - 1) < 1e-6 and abs(new_point[1] - 1) < 1e-6 and abs(new_point[2] - 100) < 1e-6
    new_point = offset_from_gps([89.9, 1, 0], [0, 0, 100], [1, 0, 200])
    assert abs(new_point[0] - 89.1) < 1e-6 and abs(new_point[1] - 181) < 1e-6 and abs(new_point[2] - 100) < 1e-6


def test_ned_from_gps():
    """Test computing NED distances from GPS coordinates."""
    ned = ned_from_gps([0, 0, 0], [1, 1, 100])
    assert abs(ned[0] - 111139)/111139 < 1e-3 and abs(ned[1] - 111139)/111139 < 1e-3 and abs(ned[2] - 100) < 1e-6
    ned = ned_from_gps([45, 0, 0], [45.1, 0.1, 100])
    assert abs(ned[0] - 11113.9)/11113.9 < 1e-2 and abs(ned[1] - 7862.67)/7862.67 < 1e-2 and abs(ned[2] - 100) < 1e-6
    ned = ned_from_gps([75, 0, 0], [75.1, 0.1, 100])
    assert abs(ned[0] - 11113.9)/11113.9 < 1e-2 and abs(ned[1] - 2877.94)/2877.94 < 1e-2 and abs(ned[2] - 100) < 1e-6


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
