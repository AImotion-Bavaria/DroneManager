"""Custom textual widgets."""
import argparse
import asyncio
import logging
import math
from rich.text import Text
from textual.binding import Binding
from textual.widgets import Input, Log, Static
from typing import IO

from dronemanager.drone import FlightMode, FixType, Drone


class ArgumentParserError(Exception):
    """Special exception class for errors not otherwise covered."""
    pass


# Dummy error to stop parsing
class PrintHelpInsteadOfParsingError(Exception):
    """Dummy exception to stop argument parsing when help is printed."""
    pass


class ArgParser(argparse.ArgumentParser):
    """Adjusted ArgumentParser that raises Exceptions instead of calling sys.exit."""

    def __init__(self, *args, logger: logging.Logger = None, **kwargs):
        """Create ArgParser.

        Args:
            *args: Passed to base class.
            logger: The logger used for parsing errors and help output.
            **kwargs: Passed to base class.
        """
        self.logger: logging.Logger = logger  #: The logger used for parsing errors and help output.
        super().__init__(*args, **kwargs)

    def error(self, message: str):
        """Changed default behaviour to raise Exceptions instead of calling sys.exit.

        Args:
            message: The error message.

        Raises:
            ValueError: Raised with a number of invalid-input-type errors.
            ArgumentParserError: Fallback for other types of argparse exceptions.
        """
        if "invalid choice" in message:
            raise ValueError(message)
        elif "arguments are required" in message:
            raise ValueError(message)
        elif "unrecognized argument" in message:
            raise ValueError(message)
        elif "invalid" in message:  # Likely an invalid argument, i.e. a string instead of float
            raise ValueError(message)
        else:
            raise ArgumentParserError(message)

    def print_help(self, file: IO[str] | None = None):
        """Print help function that stops parsing after the help is printed.

        This prevents partially parsed arguments from being executed after the help string is printed.

        Args:
            file: Where to print the help.

        Raises:
            PrintHelpInsteadOfParsingError: Dummy exception to stop parsing on help.
        """
        if file is None:
            file = (self.logger, logging.INFO)
        self._print_message(self.format_help(), file)
        raise PrintHelpInsteadOfParsingError()

    def _print_message(self, message: str, file: IO[str] | None = None):
        if message:
            file = file or (self.logger, logging.ERROR)
            try:
                if isinstance(file, tuple):
                    file[0].log(file[1], message)
                else:
                    file.write(message)
            except (AttributeError, OSError):
                pass

    def exit(self, status: int = 0, message: str = None):
        """Changes the exit behaviour of the ArgumentParser to not call sys.exit().

        Args:
            status: Status code.
            message: Exit message.

        Raises:
            ArgumentParserError: Raised when this function is called with a non-zero status code.
        """
        if status != 0:
            raise ArgumentParserError(message)
        else:
            pass


class InputWithHistory(Input):
    """Input widget with a traversable history.

    The history has a maximum size set by :py:attr:`history_max_length` and can be traversed using the arrow up and
    down keys. If new entries are added after the maximum history size is reached, old entries are overwritten using
    a rolling zero-index mechanism.
    """

    BINDINGS = Input.BINDINGS.copy()
    """Key bindings for the input field.

    Compared to standard textual input, arrow up selects the previous item in the history, arrow down selects the next.

    :meta hide-value:"""
    BINDINGS.append(Binding("up", "history_prev", "Previous item from history", show=False))
    BINDINGS.append(Binding("down", "history_rec", "Next item in history", show=False))

    def __init__(self, *args: any, **kwargs: any):
        """Create the widget.

        Args:
            *args: Passed to the base class.
            **kwargs: Passed to the base class.
        """
        super().__init__(*args, **kwargs)
        self._history: list[str] = []  #: The history list
        self.history_cursor: int = -1  #: Shows where in the history we are.
        self.rolling_zero: int = 0  #: The current position of the "zeroth" entry in the list.
        self.history_max_length: int = 50  #: The maximum size of the history.
        self._submitted_historical: bool = False  #: Whether the last submitted entry came from the history.

    @property
    def _current_history_cursor(self) -> int:
        return (self.rolling_zero - (self.history_cursor + 1)) % min(len(self._history), self.history_max_length)

    def _increase_history_cursor(self):
        if self.history_cursor < self.history_max_length - 1 and self.history_cursor < len(self._history) - 1:
            self.history_cursor += 1

    def _decrease_history_cursor(self):
        if self.history_cursor >= 0:
            self.history_cursor -= 1

    def _increase_history_rolling_pos(self):
        self.rolling_zero = (self.rolling_zero + 1) % self.history_max_length

    def action_history_prev(self):
        """Moves the history cursor backwards."""
        if self._history:
            self._submitted_historical = False
            self._increase_history_cursor()
            self.value = self._history[self._current_history_cursor]

    def action_history_rec(self):
        """Moves the history cursor forwards.

        If we have reached the end of the history, clears the field.
        """
        if self._history:
            if not self._submitted_historical:
                self._decrease_history_cursor()
            else:
                self._submitted_historical = False
            if self.history_cursor == -1:
                self.value = ""
            else:
                self.value = self._history[self._current_history_cursor]
        else:
            self.value = ""

    def add_to_history(self, item: str):
        """Add an item to the history.

        If the history is full, the item at the current rolling zero position is overwritten. Otherwise, it is simply
        appended.

        Args:
            item: The new item.
        """
        # If we add extra entries, overwrite old ones
        if len(self._history) == self.history_max_length:
            # Entry to be overridden is at rolling_zero
            self._history[self.rolling_zero] = item
            self._increase_history_rolling_pos()
        else:
            self._history.append(item)

    async def action_submit(self):
        """Handle a submit action.

        Triggered when the user hits enter while the input field is active. Compared to the base class, this also
        adds the input field to the history.
        """
        submitted_value = self.value
        await super().action_submit()  # Submit the action
        # If we are at some point in our input history and the submission is identical to that history, keep our
        # position in the history. When pressing up we get the same command again, but pressing down gets us the next
        # command in the sequence, rather than two without this special behaviour.
        if self.history_cursor != -1 and submitted_value == self._history[self._current_history_cursor]:
            self.add_to_history(submitted_value)  # Store the action in the history
            self._submitted_historical = True
        # Otherwise reset our position in the history
        else:
            self._submitted_historical = False
            self.add_to_history(submitted_value)
            self.history_cursor = -1


class DroneOverview(Static):
    """Widget to show key values for a drones."""

    COLUMN_NAMES: list[str] = ["Name", "Status", "Modes", "GPS", "Local", "Vel", "Yaw/Bat"]
    """(class attribute) The names for each column of the overview."""

    COLUMN_WIDTHS: list[int] = [10, 11, 11, 16, 9, 9, 8]
    """(class attribute) The target widths for each column. Shorter strings are padded, longer ones truncated."""

    COLUMN_ALIGN: list[str] = ["<", ">", ">", ">", ">", ">", ">"]
    """(class attribute) The alignment strings for each column."""

    COLUMN_SPACING: int = 3
    """(class attribute) The spacing between columns."""

    def __init__(self, drone: Drone, update_frequency: float, logger: logging.Logger, *args, **kwargs):
        """Create DroneOverview.

        Args:
            drone: The drone whose info this overview should show.
            update_frequency: How often this widget updates.
            logger: The logger for errors.
            *args: Passed to the base class.
            **kwargs: Passed to the base class.
        """
        super().__init__(*args, **kwargs)
        self.drone: Drone = drone  #: The drone which this overview is showing
        self.update_frequency: float = update_frequency  #: How often this widget updates
        self.logger: logging.Logger = logger  #: The logger for errors
        #: A list of format strings for each column
        self.column_formats: list[str] = [f"{{:{self.COLUMN_ALIGN[i]}{self.COLUMN_WIDTHS[i]}}}"
                                          for i in range(len(self.COLUMN_NAMES))]
        self.spacer: str = " " * self.COLUMN_SPACING  #: Spacing string
        self.format_string: str = self.spacer.join(self.column_formats)  #: The full formatted string to be filled.

    @classmethod
    def header_string(cls) -> str:
        """Creates the header string for the drone overview, with appropriate spacing.

        Returns:
            The header string.
        """
        return (" " * cls.COLUMN_SPACING).join([f"{cls.COLUMN_NAMES[i]:{cls.COLUMN_ALIGN[i]}{cls.COLUMN_WIDTHS[i]}}"
                                                for i
                                                in range(len(cls.COLUMN_NAMES))])

    @classmethod
    def gadget_width(cls) -> int:
        """Get the width of this widget, for calculating screen sizes.

        Returns:
            The width of this widget.
        """
        return (len(cls.COLUMN_NAMES) - 1) * cls.COLUMN_SPACING + sum(cls.COLUMN_WIDTHS)

    def on_mount(self):
        """Called when the widget is created, starts the update function."""
        asyncio.create_task(self.update_display())

    def _text_name(self) -> Text:
        string = self.column_formats[0].format(self.drone.name)
        return Text(string, style="bold")

    def _text_empty(self, column: int) -> Text:
        string = self.column_formats[column].format("")
        return Text(string, style="bold")

    def _text_connect(self) -> Text:
        color = "green" if self.drone.is_connected else "red"
        string = self.column_formats[1].format(f"Conn: {str(self.drone.is_connected):>{self.COLUMN_WIDTHS[1]-6}}")
        return Text(string, style=f"bold {color}")

    def _text_flightmode(self) -> Text:
        color = "green" if self.drone.flightmode == FlightMode.OFFBOARD else "red"
        string = self.column_formats[2].format(str(self.drone.flightmode))
        return Text(string, style=f"bold {color}")

    def _text_fixtype(self) -> Text:
        color = "yellow"
        if self.drone.fix_type == FixType.NO_FIX:
            color = "red"
        elif self.drone.fix_type in [FixType.RTK_FIXED, FixType.RTK_FLOAT]:
            color = "green"
        string = self.column_formats[2].format(str(self.drone.fix_type))
        return Text(string, style=f"bold {color}")

    def _text_armed(self) -> Text:
        color = "green" if self.drone.is_armed else "yellow"
        string = self.column_formats[1].format(f"Arm: {str(self.drone.is_armed):>{self.COLUMN_WIDTHS[1]-5}}")
        return Text(string, style=f"bold {color}")

    def _text_airborne(self) -> Text:
        color = "green" if self.drone.in_air else "yellow"
        string = self.column_formats[1].format(f"Air: {str(self.drone.in_air):>{self.COLUMN_WIDTHS[1]-5}}")
        return Text(string, style=f"bold {color}")

    def _text_lat(self) -> Text:
        string = self.column_formats[3].format(f"LAT: {self.drone.position_global[0]:{self.COLUMN_WIDTHS[3]-6}.6f}")
        return Text(string, style="bold")

    def _text_long(self) -> Text:
        string = self.column_formats[3].format(f"LONG: {self.drone.position_global[1]:{self.COLUMN_WIDTHS[3] - 6}.6f}")
        return Text(string, style="bold")

    def _text_amsl(self) -> Text:
        string = self.column_formats[3].format(f"AMSL: {self.drone.position_global[2]:{self.COLUMN_WIDTHS[3] - 6}.2f}")
        return Text(string, style="bold")

    def _text_p_north(self) -> Text:
        string = self.column_formats[4].format(f"N: {self.drone.position_ned[0]:{self.COLUMN_WIDTHS[4]-3}.3f}")
        return Text(string, style="bold")

    def _text_p_east(self) -> Text:
        string = self.column_formats[4].format(f"E: {self.drone.position_ned[1]:{self.COLUMN_WIDTHS[4]-3}.3f}")
        return Text(string, style="bold")

    def _text_p_down(self) -> Text:
        string = self.column_formats[4].format(f"D: {self.drone.position_ned[2]:{self.COLUMN_WIDTHS[4]-3}.3f}")
        return Text(string, style="bold")

    def _text_v_north(self) -> Text:
        string = self.column_formats[5].format(f"N: {self.drone.velocity[0]:{self.COLUMN_WIDTHS[5]-3}.3f}")
        return Text(string, style="bold")

    def _text_v_east(self) -> Text:
        string = self.column_formats[5].format(f"E: {self.drone.velocity[1]:{self.COLUMN_WIDTHS[5]-3}.3f}")
        return Text(string, style="bold")

    def _text_v_down(self) -> Text:
        string = self.column_formats[5].format(f"D: {self.drone.velocity[2]:{self.COLUMN_WIDTHS[5]-3}.3f}")
        return Text(string, style="bold")

    def _text_yaw(self) -> Text:
        string = self.column_formats[6].format(f"Y: {self.drone.attitude[2]:{self.COLUMN_WIDTHS[6]-3}.1f}")
        return Text(string, style="bold")

    def _text_bat_remain(self) -> Text:
        color = "white"
        battery_remaining = math.nan
        try:
            battery_remaining = self.drone.batteries[0].remaining
            if battery_remaining > 66:
                color = "green"
            elif battery_remaining > 33:
                color = "yellow"
            else:
                color = "red"
        except KeyError:
            pass
        string = self.column_formats[6].format(f"{battery_remaining:{self.COLUMN_WIDTHS[6]-1}.0f}%")
        return Text(string, style=f"bold {color}")

    def _text_bat_volt(self) -> Text:
        battery_voltage = math.nan
        try:
            battery_voltage = self.drone.batteries[0].voltage
        except KeyError:
            pass
        string = self.column_formats[6].format(f"{battery_voltage:{self.COLUMN_WIDTHS[6]-1}.2f}V")
        return Text(string, style="bold")

    async def update_display(self):
        """Update the overview screen with current information.

        Continuously updates the screen with the frequency set by :py:attr:`update_frequency`.
        """
        while True:
            try:
                text_output = Text.assemble(self._text_empty(0), self.spacer,
                                            self._text_connect(), self.spacer,
                                            self._text_flightmode(), self.spacer,
                                            self._text_lat(), self.spacer,
                                            self._text_p_north(), self.spacer,
                                            self._text_v_north(), self.spacer,
                                            self._text_yaw(), "\n",
                                            self._text_name(), self.spacer,
                                            self._text_armed(), self.spacer,
                                            self._text_fixtype(), self.spacer,
                                            self._text_long(), self.spacer,
                                            self._text_p_east(), self.spacer,
                                            self._text_v_east(), self.spacer,
                                            self._text_bat_remain(), "\n",
                                            self._text_empty(0), self.spacer,
                                            self._text_airborne(), self.spacer,
                                            self._text_empty(2), self.spacer,
                                            self._text_amsl(), self.spacer,
                                            self._text_p_down(), self.spacer,
                                            self._text_v_down(), self.spacer,
                                            self._text_bat_volt(), "\n",
                                            )
                self.update(text_output)
            except Exception as e:
                self.logger.debug(f"Exception updating status pane for drone {self.drone.name}: {repr(e)}",
                                  exc_info=True)
            await asyncio.sleep(1 / self.update_frequency)


class RTCM3Status(Static):
    """Display RTCM3 plugin status including connection, frame count, and forwarding targets."""
    
    def __init__(self, rtcm3_plugin, update_frequency, logger, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rtcm3_plugin = rtcm3_plugin
        self.update_frequency = update_frequency
        self.logger = logger
        self._update_task: asyncio.Task | None = None

    def on_mount(self) -> None:
        self._update_task = asyncio.create_task(self.update_display())

    def on_unmount(self) -> None:
        """Stop refreshing once this status widget is no longer displayed."""
        if self._update_task is not None:
            self._update_task.cancel()
            self._update_task = None

    def _text_connection_status(self):
        """Display RTCM3 connection status."""
        color = "green" if self.rtcm3_plugin.is_connected else "red"
        status_text = "CONNECTED" if self.rtcm3_plugin.is_connected else "DISCONNECTED"
        string = f"RTCM3 Connection: {status_text:15} ({self.rtcm3_plugin.host}:{self.rtcm3_plugin.port})"
        return Text(string, style=f"bold {color}")

    def _text_frame_count(self):
        """Display total frames parsed."""
        string = f"Frames Parsed   : {self.rtcm3_plugin.total_frames_parsed:>10}"
        return Text(string, style="bold")

    def _text_forward_targets(self):
        """Display drones receiving RTCM3 data."""
        targets = ", ".join(self.rtcm3_plugin.forward_targets) if self.rtcm3_plugin.forward_targets else "None"
        string = f"Forwarding to   : {targets}"
        return Text(string, style="bold")

    def _text_critical_messages(self):
        """Display status of critical RTCM3 message types needed for RTK."""
        critical_types = self.rtcm3_plugin.CRITICAL_MESSAGE_TYPES
        received_critical = sum(1 for msg_type in critical_types 
                               if msg_type in self.rtcm3_plugin.message_type_counters)
        total_critical = len(critical_types)
        
        # Color code based on reception
        if received_critical == 0:
            color = "red"
            status = "⚠️  CRITICAL - No message types"
        elif received_critical < total_critical // 2:
            color = "yellow"
            status = f"⚠️  WARNING - {received_critical}/{total_critical} message types"
        else:
            color = "green"
            status = f"✓ OK - {received_critical}/{total_critical} message types"
        
        string = f"Critical Msgs   : {status}"
        return Text(string, style=f"bold {color}")

    def _text_errors(self):
        """Display error counters."""
        errors = self.rtcm3_plugin.crc_errors + self.rtcm3_plugin.parse_errors
        color = "red" if errors > 0 else "green"
        string = f"Errors          : CRC={self.rtcm3_plugin.crc_errors} Parse={self.rtcm3_plugin.parse_errors}"
        return Text(string, style=f"bold {color}")

    async def update_display(self):
        while True:
            try:
                if self.rtcm3_plugin.is_connected:
                    # Show detailed information when connected
                    text_output = Text.assemble(
                        self._text_connection_status(), "\n",
                        self._text_frame_count(), "\n",
                        self._text_critical_messages(), "\n",
                        self._text_forward_targets(), "\n",
                        self._text_errors(), "\n",
                    )
                else:
                    # Show only connection status when disconnected
                    text_output = self._text_connection_status()
                
                self.update(text_output)
            except Exception as e:
                self.logger.debug(f"Exception updating RTCM3 status pane: {repr(e)}", exc_info=True)
            await asyncio.sleep(1 / self.update_frequency)


class TextualLogHandler(logging.Handler):
    """Logging Handler for textual log objects."""
    def __init__(self, log_textual: Log, *args, **kwargs):
        """Create TextualLogHandler.

        Args:
            log_textual: The textual Log object to which we write.
            *args: Passthrough to logging Handler class.
            **kwargs: Passthrough to logging Handler class.
        """
        super().__init__(*args, **kwargs)
        self.log_textual: Log = log_textual  #: The textual Log object to which we write.

    def emit(self, record: logging.LogRecord):
        """Write the log record to the textual log pane.

        Args:
            record: The log record to write out.
        """
        self.log_textual.write_line(self.format(record))
