"""Tests for the controller plugin."""
import copy
from collections import OrderedDict
import pathlib
import pytest
from typing import Any, AsyncGenerator

from dronemanager.core import DroneManager, DMConfig
from dronemanager.plugins.controllers import InputMapping, ActionInputType


def dummy_func(axes_values: list[float]) -> list[float]:
    """A dummy function that just returns the input axes values.

    Args:
        axes_values: Input values.

    Returns:
        The unchanged input values.
    """
    return axes_values


@pytest.fixture
def ps4_mapping() -> InputMapping:
    """Create an Input mapping for tests.

    Returns:
        The input mapping.
    """
    ps4_button_labels = {
        1: "Cross",
        2: "Circle",
        3: "Square",
        4: "Triangle",
        5: "Share button",
        6: "Playstation button",
        7: "Options button",
        8: "Left stick press",
        9: "Right stick press",
        10: "Left bumper",
        11: "Right bumper",
        12: "D-Pad Up",
        13: "D-Pad Down",
        14: "D-Pad Left",
        15: "D-Pad Right",
        16: "Touch pad press",
    }
    ps4_axis_labels = {
        1: "Left stick right",
        2: "Left stick down",
        3: "Right stick right",
        4: "Right stick down",
        5: "Left trigger",
        6: "Right trigger",
    }
    mapping = InputMapping("PS4 Controller", 2, 1, -4, 3, 6, 1, 2, 12, 13,
                           axes_labels=ps4_axis_labels, button_labels=ps4_button_labels, hold_duration=2)
    return mapping


@pytest.fixture
async def dm_with_controller(dm: DroneManager) -> AsyncGenerator[DroneManager, Any]:
    """A DroneManager instance with a loaded controllers plugin for testing.

    Args:
        dm: A plain DroneManager instance.

    Yields:
        The modified DroneManager instance.
    """
    await dm.load("controllers")
    yield dm


def test_mapping_functions(ps4_mapping: InputMapping):
    """Test the input mapping functions.

    Args:
        ps4_mapping: A known input mapping for testing.
    """
    # Adding a valid button action
    ps4_mapping.add_action("Test_action", action_type=ActionInputType.Button, input_id=-14)
    assert "Test_action" in ps4_mapping._actions
    assert 14 in ps4_mapping._button_actions_by_button
    assert ps4_mapping._button_actions_by_button[14][0].label == "Test_action"

    # Adding a valid button action without binding
    ps4_mapping.add_action("Test_action2", action_type=ActionInputType.Button, input_id=None)
    assert "Test_action2" in ps4_mapping._actions
    assert None in ps4_mapping._button_actions_by_button
    assert ps4_mapping._button_actions_by_button[None][0].label == "Test_action2"

    # Adding a valid axis action
    ps4_mapping.add_action("Test_action_axis", action_type=ActionInputType.Axis, input_id=[5], func=dummy_func)
    assert "Test_action_axis" in ps4_mapping._actions
    assert dummy_func in ps4_mapping._axis_actions_by_function
    assert ps4_mapping._axis_actions_by_function[dummy_func].label == "Test_action_axis"

    # Binding a button
    ps4_mapping.bind_action("Test_action2", 5)
    assert ps4_mapping._actions["Test_action2"].input_id == 5

    # Changing binding a button
    ps4_mapping.bind_action("Test_action2", 3)
    assert ps4_mapping._actions["Test_action2"].input_id == 3

    # Adding a function to existing action
    ps4_mapping.add_action("Test_action2", func=dummy_func)
    assert ps4_mapping._actions["Test_action2"].func is not None
    assert ps4_mapping._actions["Test_action2"].input_id == 3

    # Unbinding a button
    ps4_mapping.unbind_action("Test_action2")
    assert ps4_mapping._actions["Test_action2"].input_id is None

    # Binding an axis
    ps4_mapping.bind_action("Test_action_axis", [5])
    assert ps4_mapping._actions["Test_action_axis"].input_id == [5]

    # Unbinding an axis
    ps4_mapping.unbind_action("Test_action_axis")
    assert ps4_mapping._actions["Test_action_axis"].input_id is None

    # Removing an action
    ps4_mapping.remove_action("Test_action_axis")
    ps4_mapping.remove_action("Test_action2")

    # An action with multiple input axes
    ps4_mapping.add_action("Multi_axes_action", action_type=ActionInputType.Axis, input_id=[-1, -2])
    # An action on the same button as another action
    ps4_mapping.add_action("Doubled button", action_type=ActionInputType.Button, input_id=6)
    # A hold action on the button as another action
    ps4_mapping.add_action("Doubled button inverted", action_type=ActionInputType.Button, input_id=-6)

    # Test json saving and loading:
    json_str = ps4_mapping.to_json()
    recovered_ps4_mapping = InputMapping.from_json(json_str)
    assert recovered_ps4_mapping == ps4_mapping

    # Testing error behaviour
    # Adding actions with invalid inputs
    with pytest.raises(ValueError):
        ps4_mapping.add_action("Invalid_action_1", action_type=ActionInputType.Button, input_id=[5])
    with pytest.raises(ValueError):
        ps4_mapping.add_action("Invalid_action_2", action_type=ActionInputType.Axis, input_id=5)
    with pytest.raises(ValueError):
        ps4_mapping.add_action("Invalid_action_3", action_type=None, input_id=12)
    with pytest.raises(ValueError):
        ps4_mapping.add_action("Invalid_action_4", action_type=ActionInputType.Button, input_id=31)
    with pytest.raises(ValueError):
        ps4_mapping.add_action("Invalid_action_5", action_type=ActionInputType.Axis, input_id=31)

    # Trying to alter or remove core actions
    with pytest.raises(RuntimeError):
        ps4_mapping.add_action("Control", action_type=ActionInputType.Button, input_id=6, func=dummy_func)
    with pytest.raises(RuntimeError):
        ps4_mapping.remove_action("Control")

    # Trying to unbind core or non-existent actions
    with pytest.raises(RuntimeError):
        ps4_mapping.unbind_action("Control")
    with pytest.raises(KeyError):
        ps4_mapping.unbind_action("Action which does not exist")

    # Trying to bind core or non-existent actions
    with pytest.raises(RuntimeError):
        ps4_mapping.bind_action("Control", 8)
    with pytest.raises(KeyError):
        ps4_mapping.bind_action("Action which does not exist", 3)

    ps4_mapping.add_action("Test_action_axis", action_type=ActionInputType.Axis, input_id=[5], func=dummy_func)
    assert "Test_action_axis" in ps4_mapping._actions
    assert dummy_func in ps4_mapping._axis_actions_by_function
    assert ps4_mapping._axis_actions_by_function[dummy_func].label == "Test_action_axis"

    with pytest.raises(ValueError):
        ps4_mapping.bind_action("Test_action", None)
    with pytest.raises(ValueError):
        ps4_mapping.bind_action("Test_action", [5])
    with pytest.raises(ValueError):
        ps4_mapping.bind_action("Test_action_axis", 5)

    action_input_mapping = ps4_mapping.action_input_mapping()
    target_action_mapping = OrderedDict()
    target_action_mapping["Thrust"] = "Left stick down"
    target_action_mapping["Yaw"] = "Left stick right"
    target_action_mapping["Forward"] = "Right stick down - Inverted"
    target_action_mapping["Right"] = "Right stick right"
    target_action_mapping["Control"] = "Playstation button"
    target_action_mapping["Arm"] = "Cross"
    target_action_mapping["Disarm"] = "Circle"
    target_action_mapping["Takeoff"] = "D-Pad Up"
    target_action_mapping["Land"] = "D-Pad Down"
    target_action_mapping["Test_action"] = "Hold D-Pad Left"
    target_action_mapping["Multi_axes_action"] = ["Left stick right - Inverted", "Left stick down - Inverted"]
    target_action_mapping["Doubled button"] = "Playstation button"
    target_action_mapping["Doubled button inverted"] = "Hold Playstation button"
    target_action_mapping["Test_action_axis"] = ["Left trigger"]
    assert action_input_mapping == target_action_mapping

    input_action_mapping = ps4_mapping.input_action_mapping()
    target_input_mapping = OrderedDict()
    target_input_mapping["Left stick right"] = ["Yaw", "Multi_axes_action - Inverted"]
    target_input_mapping["Left stick down"] = ["Thrust", "Multi_axes_action - Inverted"]
    target_input_mapping["Right stick right"] = ["Right"]
    target_input_mapping["Right stick down"] = ["Forward - Inverted"]
    target_input_mapping["Left trigger"] = ["Test_action_axis"]
    target_input_mapping["Right trigger"] = " - "
    target_input_mapping["Cross"] = ["Arm"]
    target_input_mapping["Circle"] = ["Disarm"]
    target_input_mapping["Square"] = " - "
    target_input_mapping["Triangle"] = " - "
    target_input_mapping["Share button"] = " - "
    target_input_mapping["Playstation button"] = ["Control", "Doubled button", "Doubled button inverted - on Hold"]
    target_input_mapping["Options button"] = " - "
    target_input_mapping["Left stick press"] = " - "
    target_input_mapping["Right stick press"] = " - "
    target_input_mapping["Left bumper"] = " - "
    target_input_mapping["Right bumper"] = " - "
    target_input_mapping["D-Pad Up"] = ["Takeoff"]
    target_input_mapping["D-Pad Down"] = ["Land"]
    target_input_mapping["D-Pad Left"] = ["Test_action - on Hold"]
    target_input_mapping["D-Pad Right"] = " - "
    target_input_mapping["Touch pad press"] = " - "
    assert target_input_mapping == input_action_mapping


async def test_mapping_saving_and_loading(dm_with_controller: DroneManager, ps4_mapping: InputMapping):
    """Test adding a mapping, saving it, and then loading it again.

    Args:
        dm_with_controller: A DroneManager instance with loaded external plugin.
        ps4_mapping: An InputMapping matching a PS4 controller used for testing.
    """
    controller_plugin = getattr(dm_with_controller, "controllers")
    test_config_file_path = pathlib.Path("tests/test_config.json")

    # Adding a new mapping
    test_mapping = copy.deepcopy(ps4_mapping)
    test_mapping.name = "Test_mapping"
    n_mappings_initial = len(controller_plugin.mappings)
    controller_plugin.mappings["Test_mapping"] = test_mapping
    assert len(controller_plugin.mappings) == n_mappings_initial + 1

    # Saving the mapping
    await controller_plugin.save_current_mappings()
    dm_with_controller.save_config(test_config_file_path.as_posix())

    # Removing the new mapping
    controller_plugin.mappings.pop("Test_mapping")
    assert len(controller_plugin.mappings) == n_mappings_initial

    # Loading the new config and the mappings from that config
    new_config = DMConfig.from_file(test_config_file_path.as_posix())
    dm_with_controller.config = new_config
    controller_plugin.load_mappings_from_config()
    assert len(controller_plugin.mappings) == n_mappings_initial + 1
    assert "Test_mapping" in controller_plugin.mappings
    controller_plugin.mappings["Test_mapping"].name = ps4_mapping.name
    assert ps4_mapping == controller_plugin.mappings["Test_mapping"]

    # Remove test config file.
    test_config_file_path.unlink()
