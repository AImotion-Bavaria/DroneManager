"""Module to test core DroneManager functionality."""
from dronemanager.utils import get_config
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
