"""Plugin for using controllers and joysticks to control drones with DM.

# TODO: Extensive documentation
# TODO: Description of actions and controller objects, how bindings and physical controllers work with them, etc...

# TODO: Describe how bindings can be set in the CLI.

TODO: Controller plugin description.
TODO: Actions description.
TODO: Mapping description.
"""
import asyncio
from collections import OrderedDict
import copy
import enum
import json
import math
import os
import pygame
import time
from typing import Any, Callable

from dronemanager.drone import FlightMode
from dronemanager.plugin import Plugin
from dronemanager.utils import coroutine_awaiter


os.environ['SDL_JOYSTICK_HIDAPI_PS4_RUMBLE'] = '1'


DEFAULT_FREQUENCY: float = 50
"""Default frequency for the control and event loops."""

DEFAULT_HOLD_DURATION: float = 2
"""Default duration for which a button must be pressed continuously to count as a "long press"."""


class ActionInputType(enum.Enum):
    """Possible input types for actions."""

    Button = enum.auto()
    """Type for actions on button presses. """

    Axis = enum.auto()
    """Type for actions taking axis positions."""


class Action:
    """Actions, i.e. functions, which can or should be bound to controls.

    There are two types of actions:

    * Button actions: Called when their bound button is activated. Button combinations are not allowed.
    * Axis actions: Called every step in the control loop. The functions registered to these actions should take a
      drone name and a list of axes as their arguments.

    An action can be inverted. For axis actions, this means that the axes inputs are inverted. For button actions, this
    means that the button must be pressed for a certain amount of time, and the action is executed when the button is
    released.

    Note that axis actions are not called if any of their input axes are unbound.
    """
    def __init__(self, label: str, input_type: ActionInputType, input_id: int | list[int | None] | None = None,
                 func: Callable | None = None):
        """Create the action object.

        Args:
            label: The name or label for this action. Used as a reference and to ensure uniqueness.
            input_type: The type of input for this action.
            input_id: The IDs of the input(s) used for this action.
            func: A callable associated with this action that will be called when appropriate inputs are received.
        """
        self.label = label
        self.input_type = input_type
        self.input_id = input_id
        self.func = func

    def __eq__(self, other: Any) -> bool:
        """Equality comparison.

        Two actions are equal if they have the same label, input type, input_id and function.

        Args:
            other: The other object being compared against.

        Returns:
            Whether this and the other object are equal.
        """
        if not isinstance(other, self.__class__):
            return NotImplemented
        if self.label == other.label \
                and self.input_type is other.input_type \
                and self.input_id == other.input_id \
                and self.func == other.func:
            return True
        return False

    def __str__(self) -> str:
        """Pretty string representation.

        Returns:
            A string showing the key attributes.
        """
        return f"{self.label}: {self.input_type, self.input_id}, {self.func}"

    @property
    def controller_id(self) -> int | list[int] | None:
        """The actual ID used by the controller directly.

        DroneManager IDs are incremented by 1 to allow negatives to be used unambiguously.

        Returns:
            The raw ID for use by the controller.
        """
        if self.input_id is None:
            return None
        if isinstance(self.input_id, list):
            return [abs(single_id) - 1 if single_id is not None else single_id for single_id in self.input_id]
        else:
            return abs(self.input_id) - 1


_CORE_ACTIONS = [
    Action("Thrust", ActionInputType.Axis, None, None),
    Action("Yaw", ActionInputType.Axis, None, None),
    Action("Forward", ActionInputType.Axis, None, None),
    Action("Right", ActionInputType.Axis, None, None),
    Action("Control", ActionInputType.Button, None, None),
    Action("Arm", ActionInputType.Button, None, None),
    Action("Disarm", ActionInputType.Button, None, None),
    Action("Takeoff", ActionInputType.Button, None, None),
    Action("Land", ActionInputType.Button, None, None),
]
"""A list of core actions that every mapping must provide.

:meta hide-value:
"""


class InputMapping:
    """Mapping that assigns controller axes and buttons to actions.

    This class tracks both the available axis and button inputs and a list of actions that can or should be bound to
    these inputs.

    The available axis or buttons are tracked in the attributes :py:attr:`axes_labels` and :py:attr:`button_labels`.
    They also hold readable string representations for the inputs. Actions can't be bound to axis or buttons not
    included in these properties.

    Actions can be bound, i.e. have assigned inputs, or unbound. They can also have a function assigned or not.
    Actions without functions exist as placeholders, so bindings can be defined and then a function added later.

    Axes or buttons are assigned by integer IDs.These IDs are not the raw IDs from the controller, but incremented by
    one to avoid "0" and "-0" issues. This conversion is done automatically.
    Input IDs can be negative to indicate a special behaviour. For axes, negative IDs indicate an inverted response,
    e.g. a "-2" for the forward action means that moving the stick for axis "2" in the positive direction moves the
    drone backward.
    For buttons, a negative ID indicates that the button should be pressed for a long duration to trigger the action.
    These long press functions are executed when the button is released after they were held for the required duration.

    Axis actions can have multiple axes as inputs. They can be partially bound, i.e. only some of their inputs assigned
    to actual axes. In this case their assigned function is only called when all inputs are bound.

    We define a number of core actions which InputMapping must provide. The constructor sets their bindings.
    The exact meaning of the axis actions varies depending on the flight mode of the drone. The default is a position
    hold mode. In this mode, the core axis actions are as follows:

    * "Thrust": Represents vertical movement in a body FRD frame, so a positive input moves the drone down.
    * "Yaw": Rotation about the vertical axis, with positive being clockwise.
    * "Forward": Horizontal movement "forward", based on the current orientation of the drone.
    * "Right": Horizontal movement "right", based on the current orientation of the drone.
    """

    UNBOUND_STRING: str = " - "
    """String representation for missing bindings."""

    def __init__(self, name: str, thrust_axis: int, yaw_axis: int, forward_axis: int, right_axis: int,
                 control_button: int, arm_button: int, disarm_button: int, takeoff_button: int, land_button: int,
                 axes_labels: dict[int, str], button_labels: dict[int, str],
                 hold_duration: float = DEFAULT_HOLD_DURATION):
        """Create the input mapping object.

        Args:
            name: The name for this mapping. Used to determine uniqueness.
            thrust_axis: The controller axis for the "thrust" input. Controls thrust or altitude depending on flight
              mode.
            yaw_axis: The controller axis for the "yaw" input.
            forward_axis: The controller axis for the "forward" input. A positive input pitches or moves the drone
              forward.
            right_axis: The controller axis for the "right" input. A positive input rolls or moves the drone to the
              right.
            control_button: Pressing this button toggles control of the drone to and away the controller.
            arm_button: Pressing this button arms the drone.
            disarm_button: Pressing this button disarms the drone.
            takeoff_button: Pressing this button performs a take off.
            land_button: Pressing this button lands the drone.
            axes_labels: A dictionary listing all available axis inputs with human-readable labels.
            button_labels: A dictionary listing all available button inputs with human-readable labels.
            hold_duration: How long a button should be pressed to count as a "hold".
        """
        self.name: str = name  #: Name for this mapping
        self._actions: OrderedDict[str, Action] = OrderedDict()  #: A dictionary listing actions by their label.
        self.axes_labels: dict[int, str] = axes_labels  #: A dictionary of all axes with human-readable labels.
        self.button_labels: dict[int, str] = button_labels  #: A dictionary of all axes with human-readable labels.
        self.hold_duration: float = hold_duration  #: How long a button must be pressed to count as a long press.

        # Check that axes and button labels are unique
        input_label_list = list(self.axes_labels.values()) + list(self.button_labels.values())
        input_label_set = set(input_label_list)
        assert len(input_label_list) == len(input_label_set), "Duplicate labels for some buttons or axes!"

        # Assign core actions and their inputs
        core_inputs = [thrust_axis,
                       yaw_axis,
                       forward_axis,
                       right_axis,
                       control_button,
                       arm_button,
                       disarm_button,
                       takeoff_button,
                       land_button]
        core_actions = copy.deepcopy(_CORE_ACTIONS)
        for i, core_input in enumerate(core_inputs):
            action = core_actions[i]
            action.input_id = core_input
            self._actions[action.label] = action
            # Check that the input axes or buttons are defined for this controller.
            if action.input_type is ActionInputType.Button:
                assert abs(action.input_id) in self.button_labels, (f"Invalid controller mapping! Action {action.label}"
                                                                    f" is bound to button {abs(action.input_id)}, which"
                                                                    f" does not exist in mapping {self.name}! "
                                                                    f"Available buttons: {self.button_labels}")
            else:
                assert abs(action.input_id) in self.axes_labels, (f"Invalid controller mapping! Action {action.label} "
                                                                  f"is bound to axis {abs(action.input_id)}, which does"
                                                                  f" not exist in mapping {self.name}! "
                                                                  f"Available axes: {self.axes_labels}")

        # Check that base axes actions are unique. Inverted axes are not allowed as duplicates.
        base_axis_inputs: set[int] = set([action.controller_id for action in core_actions
                                          if action.input_type is ActionInputType.Axis])
        assert len(base_axis_inputs) == 4, ("Duplicate or missing controller bindings for base axes! Inverted and"
                                            "normal inputs referring to the same input axis are not allowed!")

        # Check that core button actions are unique. Unlike actions, holds are allowed.
        base_button_inputs: set[int] = set([action.input_id for action in core_actions
                                            if action.input_type is ActionInputType.Button])
        assert len(base_button_inputs) == 5, "Duplicate or missing controller bindings for base actions!"

        #: A dictionaries for faster access: Lists button actions by their button.
        self._button_actions_by_button: dict[int | None, list[Action]] = {}
        #: A dictionaries for faster access: Lists axis actions by their function.
        self._axis_actions_by_function: dict[Callable, Action] = {}

        #: A set of action names with the core actions. These actions are protected from being changed.
        self._core_actions: set[str] = set()

        for action_label, action in self._actions.items():
            self._core_actions.add(action_label)
            if action.input_type is ActionInputType.Button:
                self._button_actions_by_button[abs(action.input_id)] = [action]
            else:
                if action.func is not None:
                    self._axis_actions_by_function[action.func] = action

    def __eq__(self, other: Any) -> bool:
        """Equality comparison.

        Two InputMappings are equal if they have the same name, set of actions, core actions, possible axes or buttons
        and hold duration.

        Args:
            other: The other object being compared against.

        Returns:
            Whether this and the other object are equal.
        """
        if type(other) is not type(self):
            return False
        if self.name == other.name \
                and self._actions == other._actions \
                and self.axes_labels == other.axes_labels \
                and self.button_labels == other.button_labels \
                and self.hold_duration == other.hold_duration \
                and self._core_actions == other._core_actions:
            return True
        return False

    def action_input_mapping(self) -> OrderedDict[str, str | list[str]]:
        """Get a mapping from actions to button or axis labels.

        Returns:
            A dictionary listing the button, axis or axes labels for each registered action.
        """
        output = OrderedDict()
        # Add base axis and button actions
        for action_name, action in self._actions.items():
            if action.input_type is ActionInputType.Button:
                input_label = self._get_button_label(action.input_id)
            else:
                input_label = self._get_axes_label(action.input_id)
            output[action_name] = input_label
        return output

    def input_action_mapping(self) -> OrderedDict[str, list[str]]:
        """Get a mapping from button or axis labels to corresponding actions.

        Returns:
            A dictionary listing the registered action for each button or axis.
        """
        output = OrderedDict()
        for axis_id, axis_label in self.axes_labels.items():
            output[axis_label] = []
        for button_id, button_label in self.button_labels.items():
            output[button_label] = []

        # Go through each registered action and add them to the dictionary by their inputs.
        for action_name, action in self._actions.items():
            if action.input_type is ActionInputType.Button:
                input_label = self._get_button_label(abs(action.input_id))
                if action.input_id < 0:
                    action_name += " - on Hold"
                if input_label in output:
                    output[input_label].append(action_name)
            else:
                if action.input_id is None:
                    continue
                elif isinstance(action.input_id, int):
                    input_label = self._get_axes_label(abs(action.input_id))
                    if action.input_id < 0:
                        action_name += " - Inverted"
                    if input_label in output:
                        output[input_label].append(action_name)
                else:
                    for i, single_axis in enumerate(action.input_id):
                        axis_name = action_name
                        input_label = self._get_axes_label(abs(single_axis))
                        if action.input_id[i] < 0:
                            axis_name += " - Inverted"
                        if input_label in output:
                            output[input_label].append(axis_name)
        # Any inputs without actions are not bound to anything.
        for label, action_labels in output.items():
            if len(action_labels) == 0:
                output[label] = self.UNBOUND_STRING
        return output

    def _get_axes_label(self, axis_ids: int | list[int | None] | None) -> str | list[str]:
        """Get the label string for a given axis ID or list of axis IDs.

        Args:
            axis_ids: Input axis ID.

        Returns:
            Label string(s) for the axes corresponding to the axis ID(s).
        """
        if isinstance(axis_ids, list):
            axis_label = [self._get_axis_label(axis_id) for axis_id in axis_ids]
        else:
            axis_label = self._get_axis_label(axis_ids)
        return axis_label

    def _get_axis_label(self, axis_id: int | None) -> str:
        """Get the label string for a single axis ID.

        Args:
            axis_id: Input axis id.

        Returns:
            The label string.
        """
        if axis_id is None:
            axis_label = self.UNBOUND_STRING
        elif abs(axis_id) not in self.axes_labels:
            axis_label = f"Invalid axes {abs(axis_id)}, does not exist in this mapping!"
        else:
            axis_label = self.axes_labels[abs(axis_id)]
            if axis_id < 0:
                axis_label += " - Inverted"
        return axis_label

    def _get_button_label(self, button_id: int | None) -> str:
        """Get the label string for a single button ID.

        Args:
            button_id: Input button ID.

        Returns:
            The label string.
        """
        if button_id is None:
            button_label = self.UNBOUND_STRING
        elif abs(button_id) not in self.button_labels:
            button_label = f"Invalid button {abs(button_id)}, does not exist in this mapping!"
        else:
            button_label = self.button_labels[abs(button_id)]
            if button_id < 0:
                button_label = f"Hold {button_label}"
        return button_label

    def add_action(self, action_name: str, action_type: ActionInputType | None = None, func: Callable | None = None,
                   input_id: int | list[int | None] | None = None):
        """Add an action to this binding.

        Can be used to perform two different actions: Either to add a completely new action, or to add a function to
        an already existing action which doesn't have one.
        If no action with the provided name exists, a new one is created with the passed function and input ids.
        if an action with the name already exists, but doesn't have a function, insert the passed function into that
        action. In this case the arguments ``action_type`` and ``input_id`` are ignored.

        If the new action is a button action, ``input_id`` should be either an integer or ``None``.
        If the new action is an axis action, ``input_id`` must be a list containing integers or ``None`` values.

        Args:
            action_name: The name for the action.
            action_type: The type of action.
            func: The function associated with the action. Can be ``None``, unless updating an existing action.
            input_id: The inputs bound for this action, or ``None``.

        Raises:
            RuntimeError: When trying to overwrite the function of an existing action or a core action.
            ValueError: If the combination of inputs is invalid as described above.
        """
        if action_name in self._actions:
            if action_name in self._core_actions:
                raise RuntimeError(f"Can't overwrite core action {action_name}!")
            action = self._actions[action_name]
            if action.func is not None:
                raise RuntimeError(f"Can't overwrite function of already registered action {action_name}!")
            if func is None:
                raise ValueError(f"Must provide a function if updating existing action {action_name}!")
            action.func = func
            if action.input_type is ActionInputType.Axis:
                self._axis_actions_by_function[func] = action
        else:
            if action_type is None:
                raise ValueError("Must provide an action input type if creating new actions!")
            if action_type is ActionInputType.Axis and input_id is not None:
                if not isinstance(input_id, list):
                    raise ValueError("Must provide a list of integers or Nones for axis actions!")
                for input_item in input_id:
                    if not isinstance(input_item, int) or input_item is None:
                        raise ValueError("Must provide a list of integers or Nones for axis actions!")
                    if isinstance(input_item, int) and abs(input_item) not in self.axes_labels:
                        raise ValueError(f"Couldn't add action {action_name} since input {input_item} is not available"
                                         f" for this mapping!")
            elif action_type is ActionInputType.Button and input_id is not None:
                if not isinstance(input_id, int):
                    raise ValueError("Must provide a single integer or None for button actions!")
                else:
                    if abs(input_id) not in self.button_labels:
                        raise ValueError(f"Couldn't add action {action_name} since input {input_id} is not available"
                                         f" for this mapping!")
            action = Action(action_name, action_type, input_id=input_id, func=func)
            self._actions[action_name] = action
            if action_type is ActionInputType.Button:
                button_dict_id = input_id
                if button_dict_id is not None:
                    button_dict_id = abs(button_dict_id)
                if button_dict_id in self._button_actions_by_button:
                    self._button_actions_by_button[button_dict_id].append(action)
                else:
                    self._button_actions_by_button[button_dict_id] = [action]
            else:
                if func is not None:
                    self._axis_actions_by_function[func] = action

    def remove_action(self, action_name: str):
        """Remove an existing action.

        Does nothing if no action with this name exists. Core actions can't be removed.

        Args:
            action_name: The name of the action to be removed.

        Raises:
            RuntimeError: When trying to remove core actions.
        """
        if action_name in self._actions:
            if action_name in self._core_actions:
                raise RuntimeError(f"Can't remove core action {action_name}!")
            action = self._actions.pop(action_name)
            if action.input_type is ActionInputType.Button:
                if action.input_id is not None:
                    self._button_actions_by_button[action.input_id].remove(action)
            else:
                if action.func is not None:
                    self._axis_actions_by_function.pop(action.func)

    def bind_action(self, action_name: str, input_id: int | list[int | None]):
        """Add or change the control bindings of an existing action.

        Core actions are protected from being changed. Button actions must receive a single integer. Axes actions must
        receive a list of input ids, which may only have a single entry and contain ``None``. Axes actions are not
        executed if they are partially bound.

        Args:
            action_name: The name of the action.
            input_id: The new bindings for the actions.

        Raises:
            RuntimeError: When trying to change the binding for core actions.
            KeyError: When trying to change the bindings for actions which do not exist.
            ValueError: When the input id does not match the action type as described above.
        """
        if action_name not in self._actions:
            raise KeyError(f"No action with name {action_name}")
        if action_name in self._core_actions:
            raise RuntimeError(f"Can't change the bindings for core action {action_name}!")
        action = self._actions[action_name]
        if action.input_type is ActionInputType.Button:
            if not isinstance(input_id, int):
                raise ValueError("Button actions must be bound to a single button.")
            # Update button_dict
            self._button_actions_by_button[action.input_id].remove(action)
            action.input_id = input_id
            if action.input_id in self._button_actions_by_button:
                self._button_actions_by_button[action.input_id].append(action)
            else:
                self._button_actions_by_button[action.input_id] = [action]
        elif action.input_type is ActionInputType.Axis:
            if not isinstance(input_id, list):
                raise ValueError(f"Axis action {action_name} must receive a list of input IDs or Nones!")
            else:
                for input_entry in input_id:
                    if input_entry is not None and not isinstance(input_entry, int):
                        raise ValueError(f"Must provide a list of axes IDs or None for axis action {action_name}!")
                action.input_id = input_id

    def unbind_action(self, action_name: str):
        """Remove bindings from an action.

        Core actions are protected from being unbound.

        Args:
            action_name: The name of the action.

        Raises:
            RuntimeError: When trying to unbind core actions.
            KeyError: When trying to unbind actions which do not exist.
        """
        if action_name not in self._actions:
            raise KeyError(f"No action with name {action_name}")
        if action_name in self._core_actions:
            raise RuntimeError(f"Can't change the bindings for core action {action_name}!")
        self._actions[action_name].input_id = None

    def to_json(self) -> str:
        """Create a json string for saving this mapping.

        Note that this loses registered functions.

        Returns:
            A json string representation of this mapping.
        """
        output_dict = self.to_dict()
        return json.dumps(output_dict)

    def to_dict(self) -> dict:
        """Create a serializable dictionary of this mapping.

        Note that this loses registered functions.

        Returns:
            A dictionary with key information for this mapping.
        """
        output_dict = {
            "name": self.name,
            "axes_labels": list(self.axes_labels.items()),
            "button_labels": list(self.button_labels.items()),
            "hold_duration": self.hold_duration,
            "actions": {}
        }
        for action_name, action in self._actions.items():
            output_dict["actions"][action_name] = {
                "input_type": action.input_type.name,
                "input_id": action.input_id
            }
        return output_dict

    @classmethod
    def from_json(cls, json_string: str) -> "InputMapping":
        """Create an InputMapping object from a json string.

        Args:
            json_string: An input mapping encoded as a json string.

        Returns:
            The new InputMapping object.
        """
        input_dict = json.loads(json_string)
        return cls.from_dict(input_dict)

    @classmethod
    def from_dict(cls, input_dict: dict) -> "InputMapping":
        """Create an InputMapping object from a dictionary with key entries.

        Intended to be used with the structure from the DroneManager configuration file.

        Args:
            input_dict: The source dictionary.

        Returns:
            The new InputMapping object.
        """
        name = input_dict["name"]
        axes_labels = dict(input_dict["axes_labels"])
        button_labels = dict(input_dict["button_labels"])
        hold_duration = input_dict["hold_duration"]
        core_actions = [action.label for action in _CORE_ACTIONS]
        core_action_inputs = [None for _ in _CORE_ACTIONS]
        non_core_actions = []
        for action_name in input_dict["actions"]:
            # Core actions added as argument
            old_action = input_dict["actions"][action_name]
            if action_name in core_actions:
                indx = core_actions.index(action_name)
                core_action_inputs[indx] = old_action["input_id"]
            else:
                # Non core actions added later
                input_type = ActionInputType[old_action["input_type"]]
                input_id = old_action["input_id"]
                non_core_actions.append(Action(action_name, input_type=input_type, input_id=input_id))

        assert None not in core_action_inputs, "Couldn't load all core action bindings!"
        out = cls(name, *core_action_inputs, axes_labels=axes_labels, button_labels=button_labels,
                  hold_duration=hold_duration)
        for action in non_core_actions:
            out.add_action(action.label, action.input_type, None, action.input_id)
        return out


class Controller:
    def __init__(self, controller_id: int, joystick: pygame.joystick.JoystickType):
        self.id = controller_id  #: ID for this controller, matches  instance ID of the physical controller.
        self.joystick: pygame.joystick.JoystickType = joystick
        self.drone: str | None = None
        self.in_control: bool = False
        self.input_mapping: InputMapping | None = None
        self.not_connected: bool = False
        self.control_mode: FlightMode = FlightMode.POSCTL
        #: A dictionary with buttons being currently pressed and when they were first pressed.
        self.held_buttons: dict[int, float] = {}
        self.log_input_ids = False

    @property
    def name(self) -> str:
        """Get the name of the joystick object.

        Returns:
            The name of the physical joystick.
        """
        return self.joystick.get_name()

    def input_ids(self, action_name: str):
        if action_name not in self.input_mapping._actions:
            raise KeyError(f"No action {action_name} in mapping {self.input_mapping.name}")
        return self.input_mapping._actions[action_name].input_id

    def axis_output(self, action_name: str) -> float | list[float]:
        """Return the axis outputs for a given axis action."""
        action = self.input_mapping._actions[action_name]
        axes_ids = action.input_id
        axes = action.controller_id
        if isinstance(axes, int):
            return self.stick_response(axes) * math.copysign(1, axes_ids)
        else:
            return [self.stick_response(axes[i]) * math.copysign(1, axes_ids[i]) for i in range(len(axes_ids))]

    def stick_response(self, raw_axis_id: int):
        value = self.joystick.get_axis(abs(raw_axis_id))
        dz = 0.1
        if abs(value) < dz:
            return 0.0
        elif value > 0:
            value = (value - dz) / (1 - dz)
        else:
            value = (value + dz) / (1 - dz)
        return value


class ControllerPlugin(Plugin):

    PREFIX = "control"

    # TODO: Functions to print action -> button/axis label for a given controller/binding
    # TODO: Functions to print button/axis label > action for a given controller/binding
    # TODO: Functions to print input id -> button/axis label for a given controller/binding

    def __init__(self, dm, logger, name, auto_assign=False, event_frequency: float = DEFAULT_FREQUENCY,
                 control_frequency: float = DEFAULT_FREQUENCY, default_mappings: dict[str, str] = None, **kwargs):
        """

        Args:
            dm:
            logger:
            name:
            auto_assign:
            event_frequency:
            control_frequency:
            default_mappings:
            **kwargs:
        """
        super().__init__(dm, logger, name)
        self.background_functions = [
            self.event_processor(),
            self.control_loop(),
        ]
        self.cli_commands = {
            "view": self.available_controllers,
            "identify": self.identify,
            "check": self.check_controller_inputs,
            "set-binds": self.set_mapping,
            "view-binds": self.view_mapping,
            "view-inputs": self.view_inputs,
            "assign": self.assign_drone,
            "unassign": self.unassign_drone,
            "status": self.status,
        }

        pygame.init()
        pygame.joystick.init()
        #: A dictionary with available physical controllers by their instance ID.
        self.controllers: dict[int, Controller] = {}
        #: A dictionary of drones managed by this plugin and their assigned controllers.
        self.drones: dict[str, Controller] = {}

        self._relevant_events = [pygame.JOYAXISMOTION,
                                 pygame.JOYBUTTONUP,
                                 pygame.JOYBUTTONDOWN,
                                 pygame.JOYHATMOTION,
                                 pygame.JOYBALLMOTION,
                                 pygame.JOYDEVICEADDED,
                                 pygame.JOYDEVICEREMOVED]
        self.event_frequency = event_frequency
        self.control_frequency = control_frequency

        self.mappings: dict[str, InputMapping] = {}
        if default_mappings is None:
            default_mappings = {}
        self.default_mappings = default_mappings  #: Controller names as keys and default mapping as values.
        self.load_mappings_from_config()

        self.dm.add_remove_func(self._drone_disconnected_callback)

        #: If True and there are exactly one drone and one controller connected, assign them automatically.
        self.auto_assign = auto_assign

    async def status(self):
        """Log current configuration of the controller plugin."""
        drone_info_str = []
        for name, controller in self.drones.items():
            mapping_name = controller.input_mapping
            if controller.input_mapping is not None:
                mapping_name = controller.input_mapping.name
            drone_info_str.append(f"Drone: {name}\tController: {controller.id, controller.name}"
                                  f"\tActive control: {controller.in_control}\tMapping: {mapping_name}")
        if len(drone_info_str) == 0:
            drone_config = "\tNo drones assigned!"
        else:
            drone_config = "\n\t".join(drone_info_str)
        self.logger.info(f"Configured drones:\n\t{drone_config}")
        await self.available_controllers()
        self.logger.info(f"Available control schemes: {list(self.mappings.keys())}")

    async def available_controllers(self):
        """Log available controllers."""
        if len(self.controllers) == 0:
            available_controllers = "\tNo Controllers connected!"
        else:
            c_info_strings = []
            for c_id, controller in self.controllers.items():
                mapping_name = "None" if controller.input_mapping is None else controller.input_mapping.name
                c_info_strings.append(f"ID: {c_id}\tName: {controller.name}\tConnected: {not controller.not_connected}"
                                      f"\tMapping: {mapping_name}")
            available_controllers = "\n\t".join(c_info_strings)
        self.logger.info(f"Available controllers:\n\t{available_controllers}")

    async def identify(self):
        """Identify all connected controllers by rumbling them turn by turn."""
        self.logger.info("Identifying controllers by rumbling!")
        for controller_id, controller in self.controllers.items():
            self.logger.info(f"Rumbling controller {controller_id} for about 2 second.")
            controller.joystick.rumble(0.1, 0.9, 2000)
            await asyncio.sleep(4)

    async def check_controller_inputs(self, controller_id: int):
        """Toggle logging of pressed button and used axis IDs.

        Logs the IDs of any pressed buttons, or any axis which are tilted significantly.
        Using the same function for the same controller toggles the logging on and off.
        """
        if controller_id not in self.controllers:
            self.logger.warning(f"No controller with ID {controller_id}.")
        else:
            self.controllers[controller_id].log_input_ids = not self.controllers[controller_id].log_input_ids

    def load_mappings_from_config(self):
        """Populates :py:attr:`mappings` from the entries in the DroneManager config."""
        plugin_setting = self.dm.config.plugin_settings
        if "controllers" in plugin_setting:
            if "mappings" in plugin_setting["controllers"]:
                for mapping_name in plugin_setting["controllers"]["mappings"]:
                    mapping = InputMapping.from_dict(plugin_setting["controllers"]["mappings"][mapping_name])
                    self.mappings[mapping_name] = mapping

    async def save_current_mappings(self):
        """Save the current mappings to the DroneManager config. The config itself has to be saved separately!"""
        # Overwrites existing ones if they differ!
        plugin_settings = self.dm.config.plugin_settings
        if "controllers" not in plugin_settings:
            plugin_settings["controllers"] = {}
        if "mappings" not in plugin_settings["controllers"]:
            plugin_settings["controllers"]["mappings"] = {}
        for mapping_name in self.mappings:
            plugin_settings["controllers"]["mappings"][mapping_name] = self.mappings[mapping_name].to_dict()

    async def set_mapping(self, controller_id: int, mapping_name: str):
        """Set an input mapping for a given controller.

        The new InputMapping must already be loaded and stored in :py:attr:`mappings`. The controller also can't be
        actively controlling a drone.

        Args:
            controller_id: The ID of the controller.
            mapping_name: The name of the new mapping.
        """
        if mapping_name not in self.mappings:
            self.logger.warning(f"Couldn't set mapping {mapping_name} for controller {controller_id}, no such mapping "
                                f"loaded! Loaded mappings: {list(self.mappings.keys())}")
        elif controller_id not in self.controllers:
            self.logger.warning(f"Couldn't set mapping {mapping_name} for controller {controller_id}, no controller "
                                f"with that ID!")
            await self.available_controllers()
        elif self.controllers[controller_id].in_control:
            self.logger.warning("Couldn't set mapping as the controller is actively controlling a drone!")
        else:
            self.controllers[controller_id].input_mapping = self.mappings[mapping_name]
            self.logger.info(f"Set mapping {mapping_name} for controller {controller_id}")

    async def view_mapping(self, mapping_name: str):
        if mapping_name not in self.mappings:
            self.logger.warning(f"No mapping named {mapping_name}")
        else:
            mapping = self.mappings[mapping_name]
            name_length = 20
            rows = []
            rows.append(f"    {'Action':<{name_length}.{name_length}} :  {'Input IDs'}")
            rows.append("================================================")
            for key, value in mapping.action_input_mapping().items():
                rows.append(f"{key:<{name_length}.{name_length}} :  {value}")
            bind_str = "\n    ".join(rows)
            self.logger.info(f"Action binds for {mapping_name}:\n{bind_str}")

    async def view_inputs(self, mapping_name: str):
        if mapping_name not in self.mappings:
            self.logger.warning(f"No mapping named {mapping_name}")
        else:
            mapping = self.mappings[mapping_name]
            name_length = 20
            rows = []
            rows.append(f"    {'Input':<{name_length}.{name_length}} :  {'Actions IDs'}")
            rows.append("================================================")
            for key, value in mapping.input_action_mapping().items():
                rows.append(f"{key:<{name_length}.{name_length}} :  {value}")
            bind_str = "\n    ".join(rows)
            self.logger.info(f"Input binds for {mapping_name}:\n{bind_str}")

    async def assign_drone(self, drone: str, controller_id: int):
        """Assign a drone to a controller.

        Args:
            drone: The drone to be assigned.
            controller_id: The controller the drone will be assigned to.
        """
        if await self._can_assign_drone_controller(drone, controller_id):
            # Unassign from current controller if any.
            if drone in self.drones:
                self.drones[drone].drone = None
            controller = self.controllers[controller_id]
            controller.drone = drone
            self.drones[drone] = self.controllers[controller_id]
            self.logger.info(f"Controller {controller_id} now assigned to drone {drone}!")

    async def _can_assign_drone_controller(self, drone: str, controller_id: int):
        can_assign_drone = True
        # Check that a drone with that name is connected.
        if drone not in self.dm.drones:
            self.logger.warning(f"No drone named {drone}")
            can_assign_drone = False
        # Check that the drone is assignable: Either not assigned, or assigned but not controlled by a different
        # controller.
        if drone in self.drones:
            if self.drones[drone].in_control:
                self.logger.warning(f"Can't reassign drone {drone}, it is actively controlled by "
                                    f"controller {self.drones[drone].id}")
                can_assign_drone = False
        # Check that the controller exists
        if controller_id not in self.controllers:
            self.logger.warning(f"Can't assign controller {controller_id}, no such controller connected!")
            await self.available_controllers()
            can_assign_drone = False
        if can_assign_drone:
            controller = self.controllers[controller_id]
            # Check that the controller is not currently controlling a drone
            if controller.in_control:
                self.logger.warning(f"Can't assign controller {controller_id}, as it is already controlling another "
                                    f"drone, {controller.drone}!")
                can_assign_drone = False
            # Check that the controller has bindings and is connected
            if controller.input_mapping is None:
                self.logger.warning(f"Can't assign controller {controller_id} as it does not have a mapping!")
                can_assign_drone = False
            if controller.not_connected:
                self.logger.warning(f"Can't assign controller {controller_id} as it is disconnected!")
                can_assign_drone = False
        return can_assign_drone

    async def unassign_drone(self, drone: str):
        """Unassign a drone from a controller.

        Args:
            drone: The drone to be unassigned.
        """
        if drone not in self.drones:
            self.logger.warning(f"Can't unassign drone {drone}, as it is not assigned to any controllers!")
        else:
            controller = self.drones[drone]
            if controller.in_control:
                self.logger.warning(f"Can't unassign drone {drone}, as it is currently actively controlled!")
            else:
                controller = self.drones.pop(drone)
                controller.drone = None

    async def control_loop(self):
        """Main control loop.

        Takes controller inputs, performs some processing, like sending them past the fence safety functions, and then
        forwards the inputs to the drone. The frequency of the loop is determined by :py:attr:`control_frequency`.
        The loop also performs controller discovery, automatic assignment if :py:attr:`auto_assign` is set, and
        logging of depressed axes for controllers with that set.
        """
        next_loop_time = time.monotonic()
        counter = 0
        while True:
            try:
                # Determine time for next loop
                prev_loop_time = next_loop_time
                next_loop_time = prev_loop_time + 1 / self.control_frequency
                loop_interval = max(0.0, next_loop_time - time.monotonic())
                await asyncio.sleep(loop_interval)
                counter += 1
                if counter % 1000 == 0:
                    counter = 0
                pygame.event.pump()
                # Do controller discovery
                self._discover_controllers()
                # Process inputs
                for drone, controller in self.drones.items():
                    if drone in self.dm.drones:
                        self._process_stick_inputs(drone)
                    else:
                        await self._drone_disconnected_callback(drone)  # Make sure we stop if a drone get removed
                # Log axis ids for controller with that enabled
                for controller in self.controllers.values():
                    if controller.log_input_ids:
                        self._log_input_ids(controller, counter)
                # If auto_drone is True, there is one controller and there is one drone in drone manager, and we're not
                # already controlling the drone, try to assign controller and drone automatically
                if self.auto_assign and len(self.dm.drones) == 1 and len(self.controllers) == 1 \
                        and len(self.drones) == 0:
                    drone_name = list(self.dm.drones.keys())[0]
                    controller_id, controller = list(self.controllers.items())[0]
                    if await self._can_assign_drone_controller(drone_name, controller_id):
                        self.logger.info(f"Auto-assigning controller {controller_id} to drone {drone_name}")
                        assign_task = asyncio.create_task(self.assign_drone(drone_name, controller_id))
                        assign_wait_task = asyncio.create_task(coroutine_awaiter(assign_task, self.logger))
                        self.running_tasks.add(assign_task)
                        self.running_tasks.add(assign_wait_task)
            except Exception as e:
                self.logger.warning("Error in main controller loop!")
                self.logger.debug(repr(e), exc_info=True)

    def _discover_controllers(self):
        """Discovers controllers connected to the machine."""
        current_controllers = pygame.joystick.get_count()
        for i in range(current_controllers):
            joystick = pygame.joystick.Joystick(i)
            cid = joystick.get_instance_id()
            if cid not in self.controllers:
                self.running_tasks.add(asyncio.create_task(self.add_controller(cid, joystick)))

    async def add_controller(self, controller_id: int, joystick: pygame.joystick.JoystickType):
        """Add a controller to the plugin.

        Shouldn't have to be called manually, as connected controllers are discovered and added automatically.
        Controllers rumble when connected. A single rumble indicates a controller that is ready to be assigned. A
        series of rumbles indicates missing input bindings.

        Args:
            controller_id: The ID of the pygame controller to be added.
            joystick: The joystick object associated with the physical controller.
        """
        controller = Controller(controller_id, joystick)
        if controller.name in self.default_mappings:
            mapping_name = self.default_mappings[controller.name]
            if mapping_name in self.mappings:
                controller.input_mapping = self.mappings[mapping_name]
            else:
                self.logger.warning(f"Controller {controller_id, controller.name} "
                                    f"has default mapping {mapping_name}, but that mapping isn't loaded! "
                                    f"Loaded mappings: {list(self.mappings.keys())}")
        self.controllers[controller_id] = controller
        if controller.input_mapping is None:
            self.logger.info(f"Connected to controller {controller.name}, no binding currently "
                             f"configured!")
            await asyncio.sleep(0.1)
            controller.joystick.rumble(0.5, 0.9, 300)
            await asyncio.sleep(0.6)
            controller.joystick.rumble(0.5, 0.9, 300)
            await asyncio.sleep(0.6)
            controller.joystick.rumble(0.5, 0.9, 300)
            await asyncio.sleep(0.3)
        else:
            self.logger.info(f"Connected to controller {controller.name}, using "
                             f"{controller.input_mapping.name} bindings!")
            await asyncio.sleep(0.1)
            controller.joystick.rumble(0.5, 0.9, 1000)
            await asyncio.sleep(1.0)

    async def remove_controller(self, controller_id: int):
        """Remove a connected controller.

        Shouldn't be necessary, as disconnected controllers do not need to be shut down, and all connected controllers
        are disconnected automatically when the plugin is shut down.

        Args:
            controller_id: The ID of the controller to be removed.
        """
        if controller_id not in self.controllers:
            self.logger.warning(f"Can't remove controller {controller_id}, no such controller!")
            await self.available_controllers()
        else:
            controller = self.controllers[controller_id]
            if controller.in_control:
                self.logger.warning(f"Can't remove controller {controller_id}, as it is controlling drone "
                                    f"{controller.drone}!")
            else:
                # Remove the controller
                controller = self.controllers.pop(controller_id)
                if controller.drone is not None:
                    self.drones.pop(controller.drone, None)
                controller.joystick.quit()

    async def _force_remove_controller(self, controller_id: int):
        controller = self.controllers[controller_id]
        if controller.in_control:
            await self._release_control(controller_id)
        await self.remove_controller(controller_id)

    def _process_stick_inputs(self, drone_name: str):
        """Process the axes inputs for a single controlled drone.

        Args:
            drone_name: The name of the drone.
        """
        drone_obj = self.dm.drones[drone_name]
        controller = self.drones[drone_name]
        vertical_input = controller.axis_output("Thrust")
        yaw_input = controller.axis_output("Yaw")
        right_input = controller.axis_output("Right")
        forward_input = controller.axis_output("Forward")

        # If we have non-zero inputs, and we aren't in the appropriate mode, put us into appropriate mode
        if abs(vertical_input) > 0.01 or abs(yaw_input) > 0.01 \
                or abs(right_input) > 0.01 or abs(forward_input) > 0.01:
            if drone_obj.flightmode != controller.control_mode:
                if controller.control_mode is FlightMode.POSCTL:
                    swap_to_manual_task = asyncio.create_task(drone_obj.manual_control_position())
                else:
                    raise NotImplementedError
                self.running_tasks.add(swap_to_manual_task)
                self.running_tasks.add(asyncio.create_task(coroutine_awaiter(swap_to_manual_task, self.logger)))

        # If we are connected and armed, send stick inputs to drone
        if drone_obj.is_connected:
            if drone_obj.fence is not None:
                try:
                    forward_input, right_input, vertical_input, yaw_input = \
                        drone_obj.fence.controller_safety(drone_obj, forward_input, right_input, vertical_input,
                                                          yaw_input)
                except AttributeError as e:
                    self.logger.warning("Fence constraints could not be applied due to missing fence attributes.")
                    self.logger.debug(repr(e), exc_info=True)
                except Exception as e:
                    self.logger.error("Error applying controller fence logic")
                    self.logger.debug(repr(e), exc_info=True)

            # Scale vertical from -1/1 to 0/1 and flip because MAVLink manual control has up as positive
            vertical_input = (-vertical_input + 1) / 2
            manual_input_task = asyncio.create_task(drone_obj.set_manual_control_input(forward_input, right_input,
                                                                                       vertical_input, yaw_input))
            self.running_tasks.add(manual_input_task)
            self.running_tasks.add(asyncio.create_task(coroutine_awaiter(manual_input_task, self.logger)))

        # Also perform whatever other functions are bound to any other axis
        for func, action in controller.input_mapping._axis_actions_by_function.items():
            axis_values = controller.axis_output(action.label)
            if func is not None:
                try:
                    func(axis_values)
                except Exception as e:
                    self.logger.warning(f"Encountered an exception processing function {func} controllers")
                    self.logger.debug(repr(e), exc_info=True)

    def _log_input_ids(self, controller, counter):
        # Go through all axes, get their outputs, and log if any of them exceed -0.5, 0.5
        # Only do this approximately once per second to avoid spamming the log.
        # For example, some axes start at a limit
        sps = math.ceil(self.control_frequency)
        if counter % sps == 0:
            for axis in range(controller.joystick.get_numaxes()):
                axis_output = controller.joystick.get_axis(axis)
                if abs(axis_output) > 0.5:
                    self.logger.info(f"Axis {axis + 1} output: {axis_output}")

    async def event_processor(self):
        next_loop_time = time.monotonic()
        while True:
            try:
                # Determine time for next loop
                prev_loop_time = next_loop_time
                next_loop_time = prev_loop_time + 1 / self.event_frequency
                loop_interval = max(0.0, next_loop_time - time.monotonic())
                await asyncio.sleep(loop_interval)
                for event in pygame.event.get():
                    if event.type in self._relevant_events:
                        event_dict = event.dict
                        if "instance_id" in event_dict \
                                and event_dict["instance_id"] in self.controllers.keys():
                            controller_id = event_dict["instance_id"]
                            controller = self.controllers[controller_id]
                            # Button presses
                            if event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP]:
                                released = event.type == pygame.JOYBUTTONUP
                                # Add 1 to match what actions use.
                                pressed_button_id = event_dict["button"] + 1
                                # Log inputs if set
                                if controller.log_input_ids:
                                    if released:
                                        self.logger.info(f"Released button {pressed_button_id}")
                                    else:
                                        self.logger.info(f"Pressed button {pressed_button_id}")
                                # Perform actions if pressed button matches event.
                                if controller.input_mapping is not None:
                                    if pressed_button_id in controller.input_mapping.button_labels:
                                        pressed_button_label = controller.input_mapping.button_labels[pressed_button_id]
                                        try:
                                            self._process_button_press(controller, pressed_button_id, released=released)
                                        except Exception as e:
                                            self.logger.error(f"Exception processing button press for "
                                                              f"button {pressed_button_label}!")
                                            self.logger.debug(repr(e), exc_info=True)
                            # Controller disconnected
                            elif event.type == pygame.JOYDEVICEREMOVED:
                                self.logger.warning(f"Controller {controller_id} disconnected!")
                                controller.not_connected = True
                                # Reconnects are not discovered as the same controller, have to remove it
                                remove_task = asyncio.create_task(self._force_remove_controller(controller_id))
                                self.running_tasks.add(remove_task)
                                self.running_tasks.add(asyncio.create_task(coroutine_awaiter(remove_task, self.logger)))
                            # Anything else: Log it if set.
                            else:
                                if controller.log_input_ids and event.type != pygame.JOYAXISMOTION:
                                    self.logger.info(f"Event ID: {event.type}, entries: {event_dict}")
                        else:
                            self.logger.debug(f"{event.type}, {event_dict}")
            except Exception as e:
                self.logger.warning("Exception processing controller event!")
                self.logger.debug(repr(e), exc_info=True)

    def _process_button_press(self, controller: Controller, button_id: int, released: float = False):
        can_do_actions = (controller.in_control and controller.drone is not None
                          and self.dm.drones[controller.drone].is_connected)
        toggle_control = False

        # For hold actions: Save the time this button was pressed:
        if released:
            try:
                time_held = time.monotonic() - controller.held_buttons[button_id]
                controller.held_buttons.pop(button_id)
            except KeyError as e:
                self.logger.warning(f"Released button {button_id} was never pressed, skipping...!")
                self.logger.debug(repr(e), exc_info=True)
                return
        else:
            time_held = 0
            controller.held_buttons[button_id] = time.monotonic()

        # Get the action for this button id, then perform it. Core actions are handled differently than others.
        # Core actions can only be performed if we are actively controlling a drone.
        if button_id in controller.input_mapping._button_actions_by_button:
            registered_actions = controller.input_mapping._button_actions_by_button[button_id]
            button_label = controller.input_mapping.button_labels[button_id]
            for action in registered_actions:
                hold_action = action.input_id < 0
                # Check that button activation matches requirement:
                # If this is a hold action, the button was released after enough time, do action
                # If this is a press action and the button was pressed, do action.
                # Else skip
                if not ((hold_action and released and time_held > controller.input_mapping.hold_duration)
                    or (not hold_action and not released)):
                    continue
                action_coro = None
                action_coro_args = {}
                if action.label == "Control":
                    if controller.drone is not None:
                        self.logger.info(f"Trying to toggling manual control for drone {controller.drone} ...")
                        action_coro_args = {"controller_id": controller.id}
                        if controller.in_control:
                            action_coro= self._release_control
                        else:
                            action_coro= self._take_control
                        toggle_control = True
                    else:
                        self.logger.info(f"Can't take control, not assigned to any drone!")
                elif action.label == "Arm":
                    self.logger.debug(f"Arm button {button_label} pressed")
                    action_coro_args = {"names": controller.drone}
                    action_coro = self.dm.arm
                elif action.label == "Disarm":
                    self.logger.debug(f"Disarm button {button_label} pressed")
                    action_coro_args = {"names": controller.drone}
                    action_coro = self.dm.disarm
                elif action.label == "Takeoff":
                    self.logger.debug(f"Takeoff button {button_label} pressed")
                    action_coro_args = {"names": controller.drone,
                                        "altitude": 1.5,
                                        "allow_in_air": False}
                    action_coro = self.dm.takeoff
                elif action.label == "Land":
                    self.logger.debug(f"Land button {button_label} pressed")
                    action_coro_args = {"names": controller.drone}
                    action_coro = self.dm.land
                else:
                    # Do non-core actions
                    if action.func is not None:
                        try:
                            action.func()
                        except Exception as e:
                            self.logger.warning(f"Encountered an exception processing function {action.func} for "
                                                f"action {action.label} on button {button_label}")
                            self.logger.debug(repr(e), exc_info=True)

                # Log information for user about current control state
                if action_coro is not None and not can_do_actions and not toggle_control:
                    if not controller.in_control:
                        self.logger.info("Received control inputs, but not in control of drone!")

                # Do the action if we have an action and either can do it, or are toggling control (which is checked separately)
                if action_coro is not None and controller.drone is not None and (can_do_actions or toggle_control):
                    coro = action_coro(**action_coro_args)
                    # Cancel anything the drone might be doing
                    self.dm.drones[controller.drone].clear_queue()
                    self.dm.drones[controller.drone].cancel_action()
                    action_task = asyncio.create_task(coro)
                    action_awaiter = asyncio.create_task(coroutine_awaiter(action_task, self.logger))
                    self.running_tasks.add(action_task)
                    self.running_tasks.add(action_awaiter)

    async def _take_control(self, controller_id: int):
        """The specified controller starts controlling the drone.

        Args:
            controller_id: The ID of the controller.

        Raises:
            NotImplementedError: When trying to fly with unsupported control modes.
        """
        controller = self.controllers[controller_id]
        if controller.drone is None:
            self.logger.warning(f"Can't take control, no drone assigned to controller {controller_id}!")
        elif not self.dm.drones[controller.drone].is_connected:
            self.logger.warning(f"Can't take control, assigned drone {controller.drone} is disconnected!")
        else:
            if controller.control_mode is FlightMode.POSCTL:
                await self.dm.drones[controller.drone].manual_control_position()
            else:
                raise NotImplementedError
            self.logger.info(f"Controller {controller_id} took control of {controller.drone}")
            controller.in_control = True

    async def _release_control(self, controller_id: int):
        """Release control of a drone, putting it into HOLD mode.

        Args:
            controller_id: The controller which is releasing control.
        """
        controller = self.controllers[controller_id]
        controller.in_control = False
        await self.dm.change_flightmode(controller.drone, "hold")
        self.logger.info(f"Controller {controller_id} released control of {controller.drone}.")

    async def _drone_disconnected_callback(self, name: str):
        """Callback when a drone gets disconnected from DroneManager.

        Args:
            name: The name of the drone that was disconnected.
        """
        # If a drone we were controlling got disconnected
        controller = self.drones.pop(name, None)
        if controller is not None:
            self.logger.info(f"Drone {name} assigned to controller {controller} was disconnected.")
            controller.drone = None
            controller.in_control = False

    async def close(self):
        """Close the plugin.

        Disconnects all controllers and stops pygame.
        """
        await super().close()
        for _, controller in self.controllers.items():
            controller.joystick.quit()
        pygame.quit()
