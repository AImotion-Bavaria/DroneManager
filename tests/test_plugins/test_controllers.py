"""Tests for the controller plugin."""
import asyncio
import copy
from collections import OrderedDict
import pathlib
import pytest
from queue import Queue
from typing import Any, AsyncGenerator
from unittest.mock import patch, MagicMock, Mock

from dronemanager.core import DroneManager, DMConfig
from dronemanager.drone import FlightMode
from dronemanager.plugins.controllers import InputMapping, ActionInputType


def dummy_func(axes_values: list[float] | None = None) -> list[float] | None:
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
    mapping = InputMapping("PS4 Controller", 2, 1, -4, 3, -6, 1, 2, 12, 13,
                           axes_labels=ps4_axis_labels, button_labels=ps4_button_labels, hold_duration=2)
    return mapping


class FakeJoystick:
    """A fake joystick class to test pygame with."""
    def __init__(self, instance_id: int = 0, name: str = "Test Controller"):
        """Create the fake joystick object.

        Args:
            instance_id: The instance ID for this fake joystick.
            name: The name of this fake joystick.
        """
        self.instance_id: int = instance_id  #: Instance ID for the mock joystick.
        self.name: str = name  #: Instance ID for the mock joystick.
        self.axes: list[float] = [0.0, 0.64, -0.91, 0.0, 0.0, 0.0]  #: Mock axes output. Should be between -1 and 1.
        self.rumbles: list[tuple[float, float, float]] = []  #: A list of all calls to the rumble function.
        self.quit_called: bool = False  #: If quit was called for this joystick.

    def get_instance_id(self) -> int:
        """Replacement function.

        Returns:
            The set fake instance id.
        """
        return self.instance_id

    def get_name(self) -> str:
        """Replacement function.

        Returns:
            The set fake name.
        """
        return self.name

    def get_numaxes(self) -> int:
        """Replacement function.

        Returns:
            The number of axes that we have defined outputs for.
        """
        return len(self.axes)

    def get_axis(self, axis: int) -> float:
        """Replacement function.

        Args:
            axis: The axis ID whose fake output we get.

        Returns:
            The fake output for the provided axis.
        """
        return self.axes[axis]

    def rumble(self, low_frequency: float, high_frequency: float, duration: float):
        """Replacement function, appends calls with arguments to :py:attr:`rumbles`.

        Args:
            low_frequency: Low end of fake rumble.
            high_frequency: High end of fake rumble.
            duration: Duration for fake rumble.
        """
        self.rumbles.append((low_frequency, high_frequency, duration))

    def quit(self):
        """Replacement function, just tracks whether it was called for this object or not."""
        self.quit_called = True


@pytest.fixture
async def dm_with_controller(dm: DroneManager) -> AsyncGenerator[tuple[DroneManager, MagicMock], Any]:
    """A DroneManager instance with a loaded controllers plugin for testing.

    Args:
        dm: A plain DroneManager instance.

    Yields:
        The modified DroneManager instance.
    """
    with patch("dronemanager.plugins.controllers.pygame") as mock_pygame:
        mock_pygame.JOYAXISMOTION = 1
        mock_pygame.JOYBUTTONUP = 2
        mock_pygame.JOYBUTTONDOWN = 3
        mock_pygame.JOYHATMOTION = 4
        mock_pygame.JOYBALLMOTION = 5
        mock_pygame.JOYDEVICEADDED = 6
        mock_pygame.JOYDEVICEREMOVED = 7

        mock_pygame.joystick.get_count.return_value = 0

        await dm.load("controllers")
        yield dm, mock_pygame


def test_mapping_functions(ps4_mapping: InputMapping):
    """Test the input mapping functions.

    Args:
        ps4_mapping: A known input mapping for testing.
    """
    # Adding a valid button action
    ps4_mapping.add_action("Test_action", action_type=ActionInputType.Button, input_id=-14)
    assert "Test_action" in ps4_mapping._actions
    str(ps4_mapping._actions["Test_action"])
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
    with pytest.raises(ValueError):
        ps4_mapping.add_action("Invalid_action_6", action_type=ActionInputType.Axis, input_id=[None, 42])

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

    ps4_mapping.add_action("Test_action_axis", action_type=ActionInputType.Axis, input_id=[5],
                           func=dummy_func)
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
    target_action_mapping["Control"] = "Hold Playstation button"
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
    target_input_mapping["Playstation button"] = ["Control - on Hold", "Doubled button",
                                                  "Doubled button inverted - on Hold"]
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


async def test_mapping_saving_and_loading(dm_with_controller: tuple[DroneManager, MagicMock],
                                          ps4_mapping: InputMapping):
    """Test adding a mapping, saving it, and then loading it again.

    Args:
        dm_with_controller: A DroneManager instance with loaded external plugin.
        ps4_mapping: An InputMapping matching a PS4 controller used for testing.
    """
    manager, mock_pygame = dm_with_controller
    controller_plugin = getattr(manager, "controllers")
    test_config_file_path = pathlib.Path("tests/test_config.json")

    # Adding a new mapping
    test_mapping = copy.deepcopy(ps4_mapping)
    test_mapping.name = "Test_mapping"
    n_mappings_initial = len(controller_plugin.mappings)
    controller_plugin.mappings["Test_mapping"] = test_mapping
    assert len(controller_plugin.mappings) == n_mappings_initial + 1

    # Saving the mapping
    await controller_plugin.save_current_mappings()
    manager.save_config(test_config_file_path.as_posix())

    # Removing the new mapping
    controller_plugin.mappings.pop("Test_mapping")
    assert len(controller_plugin.mappings) == n_mappings_initial

    # Loading the new config and the mappings from that config
    new_config = DMConfig.from_file(test_config_file_path.as_posix())
    manager.config = new_config
    controller_plugin.load_mappings_from_config()
    assert len(controller_plugin.mappings) == n_mappings_initial + 1
    assert "Test_mapping" in controller_plugin.mappings
    controller_plugin.mappings["Test_mapping"].name = ps4_mapping.name
    assert ps4_mapping == controller_plugin.mappings["Test_mapping"]

    # Remove test config file.
    test_config_file_path.unlink()


async def test_plugin(dm_with_controller: tuple[DroneManager, Mock], mock_drone: Mock, ps4_mapping: InputMapping):
    """Big ugly test function for most parts of the plugin class.

    Args:
        dm_with_controller: A DroneManager instance with the controller plugin already loaded.
        mock_drone: A mock drone object.
        ps4_mapping: A fixed InputMapping.
    """
    manager, mock_pygame = dm_with_controller
    controller_plugin = getattr(manager, "controllers")
    ps4_mapping.name = "Test Mapping"

    mock_pygame.init.assert_called_once()
    mock_pygame.joystick.init.assert_called_once()

    mock_joystick1 = FakeJoystick(instance_id=31, name="Test Controller")
    mock_joystick2 = FakeJoystick(instance_id=5, name="PS4 Controller")

    def joystick_return_function(dev_id: int) -> FakeJoystick:
        """Dummy function returning fake joystick objects for given "device" IDs.

        Args:
            dev_id: The fake device ID.

        Returns:
            One of two fixed fake joystick objects.
        """
        if dev_id == 0:
            return mock_joystick1
        else:
            return mock_joystick2

    # Add first mock controller
    mock_pygame.joystick.Joystick.side_effect = joystick_return_function
    mock_pygame.joystick.get_count.return_value = 1
    await asyncio.sleep(2)
    assert len(mock_joystick1.rumbles) == 3
    assert len(controller_plugin.controllers) == 1
    assert controller_plugin.controllers[31].id == 31
    assert controller_plugin.controllers[31].input_mapping is None

    mock_func = Mock()
    mock_func.side_effect = dummy_func

    mock_axis_func = Mock()
    mock_axis_func.side_effect = dummy_func

    # Add a test action func to mapping
    ps4_mapping.add_action("Dummy button action", action_type=ActionInputType.Button, func=mock_func, input_id=1)
    ps4_mapping.add_action("Dummy axis action", action_type=ActionInputType.Axis, func=mock_axis_func, input_id=[2, -3])

    # Setting a mapping
    controller_plugin.mappings["Test Mapping"] = ps4_mapping
    await controller_plugin.set_mapping(5, "Test Mapping")
    assert controller_plugin.controllers[31].input_mapping is None
    await controller_plugin.set_mapping(31, "Fake mapping")
    assert controller_plugin.controllers[31].input_mapping is None
    controller_plugin.controllers[31].in_control = True
    await controller_plugin.set_mapping(31, "Test Mapping")
    assert controller_plugin.controllers[31].input_mapping is None
    controller_plugin.controllers[31].in_control = False
    await controller_plugin.set_mapping(31, "Test Mapping")
    assert controller_plugin.controllers[31].input_mapping.name == "Test Mapping"

    # Status
    await controller_plugin.status()

    # Toggle input logging
    await controller_plugin.check_controller_inputs(31)

    # Identify
    await controller_plugin.identify()
    await asyncio.sleep(0.2)
    assert len(mock_joystick1.rumbles) == 4

    # Mapping logging
    _ = await controller_plugin.view_mapping("Test Mapping")
    _ = await controller_plugin.view_inputs("Test Mapping")

    # Add a mock_drone
    mock_drone.flightmode = FlightMode.HOLD
    manager.drones["mock"] = mock_drone
    await asyncio.sleep(0.3)
    # Check that the drone and controller were assigned
    assert len(controller_plugin.drones) == 1
    assert controller_plugin.controllers[31].drone == "mock"
    # Check that the drone flightmode posctrl function was called
    mock_drone.manual_control_position.assert_called_once()
    # Check that we called the stick input function.
    mock_drone.set_manual_control_input.assert_called()
    # Check that the axis input was called
    mock_axis_func.assert_called_with([0.6, 0.9])

    mock_axis_func.side_effect = RuntimeError("Test error")

    class MockEvent:
        """A mock pygame event class."""
        def __init__(self, event_type: int, button_id: int, controller_id: int):
            """Create this MockEvent.

            Args:
                event_type: The type of pygame event to mock.
                button_id: The button ID this event refers to. Ignored for non-button events.
                controller_id: The mock controller ID that the event will be faked from.
            """
            self.type = event_type
            self.dict = {
                "button": button_id,
                "instance_id": controller_id,
            }

    class MockEventQueue:
        """Mock event queue for use with pygame.event.get()."""

        def __init__(self, queue: Queue):
            """Create the mock event queue.

            Args:
                queue: Queue object for the internal use. Add mock events to this object to make them available.
            """
            self.queue = queue

        def get(self) -> list[MockEvent]:
            """Grabs an item from the internal queue or returns an empty list if the queue is empty.

            Returns:
                An item from the queue as a list for use with pygame.event.get() or an empty list if the queue is empty.
            """
            if self.queue.empty():
                return []
            else:
                return [self.queue.get()]

    test_event_queue = Queue()
    mock_pygame.event.get.side_effect = MockEventQueue(test_event_queue).get
    # Press and hold the control_button for a short time seconds, test nothing happens
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONDOWN, 5, 31))
    await asyncio.sleep(0.1)
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONUP, 5, 31))
    await asyncio.sleep(0.1)
    assert not controller_plugin.controllers[31].in_control

    # Press the arm button, test that arm action was not executed, but our dummy function was.
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONDOWN, 0, 31))
    await asyncio.sleep(0.1)
    mock_drone.execute_task.assert_not_called()
    mock_func.assert_called_once()

    # Press and hold the control button for 2.5 seconds, test that we take control
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONDOWN, 5, 31))
    await asyncio.sleep(2.5)
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONUP, 5, 31))
    await asyncio.sleep(0.1)
    assert controller_plugin.controllers[31].in_control

    # Press the arm button, test that arm action was executed.
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONDOWN, 0, 31))
    await asyncio.sleep(0.1)
    mock_drone.execute_task.assert_called_once()

    # "Plug" second controller in
    mock_pygame.joystick.get_count.return_value = 2
    await asyncio.sleep(0.1)
    assert len(controller_plugin.controllers) == 2
    assert controller_plugin.controllers[5].input_mapping.name == "PS4 Controller"

    # Try to reassign drone being controlled to other controller
    assert controller_plugin.controllers[31].in_control and controller_plugin.controllers[31].drone == "mock"
    await controller_plugin.assign_drone("mock", 5)
    assert controller_plugin.controllers[31].drone == "mock" and controller_plugin.controllers[5].drone is None
    # Try to assign controller that does not exist
    await controller_plugin.assign_drone("mock", 7)
    assert controller_plugin.controllers[31].drone == "mock" and controller_plugin.controllers[5].drone is None

    await controller_plugin.status()

    # Press and hold the control button for 2.5 seconds, test that we released control
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONDOWN, 5, 31))
    await asyncio.sleep(2.5)
    test_event_queue.put(MockEvent(mock_pygame.JOYBUTTONUP, 5, 31))
    await asyncio.sleep(0.1)
    assert not controller_plugin.controllers[31].in_control

    # Try to reassign now
    assert not controller_plugin.controllers[31].in_control and controller_plugin.controllers[31].drone == "mock"
    await controller_plugin.assign_drone("mock", 5)
    assert controller_plugin.controllers[31].drone is None and controller_plugin.controllers[5].drone == "mock"

    # "Unplug" the test controllers
    mock_pygame.joystick.get_count.return_value = 0
    test_event_queue.put(MockEvent(mock_pygame.JOYDEVICEREMOVED, 0, 31))
    test_event_queue.put(MockEvent(mock_pygame.JOYDEVICEREMOVED, 0, 5))
    await asyncio.sleep(0.1)
    assert len(controller_plugin.controllers) == 0

    assert mock_joystick1.quit_called and mock_joystick2.quit_called
