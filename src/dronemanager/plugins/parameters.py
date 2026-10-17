"""Save, apply and change flight controller parameters.

The parameters plugin can back up the complete parameter set of a connected drone to a file, write such a file back to
the same or another drone, and read or change single parameters. It is loaded with ``load parameters`` and its CLI
commands use the prefix ``param``. Every command supports ``-h`` for a description of its arguments.

Parameter files
---------------

Parameter files use the tab-separated QGroundControl ``.params`` format, so they can also be loaded or created with
QGroundControl::

    # Vehicle-Id	Component-Id	Name	Value	Type
    1	1	MPC_XY_VEL_MAX	12.0	9
    1	1	SYS_AUTOSTART	4001	6

The type column follows ``MAV_PARAM_TYPE``: Types 1 to 8 are integers, 9 and 10 floats. Lines starting with ``#`` are
comments. Mission Planner style ``NAME,VALUE`` lines are also accepted when reading, in which case the type is taken
from the drone.

By default, the files are stored in the "drone_parameters" folder of the DroneManager directory in the user
documents, see :py:data:`PARAM_DIR`. A different directory can be set with the ``directory`` entry in the plugin
settings of the configuration file.

Drone specific parameters
-------------------------

Some parameters belong to one physical drone and shouldn't be copied to others. When applying or comparing a file,
two groups of them are skipped by default:

- Calibration (:py:data:`CALIBRATION_PARAMS`): Sensor, battery and RC calibrations, sensor device IDs and statistics.
  Applied with ``--include_calibration``.
- Tuning (:py:data:`TUNING_PARAMS`): Attitude, rate and position controller gains, gyro filters and hover thrust.
  Applied with ``--include_tuning``.

This allows using one file for a whole fleet, for example with the settings for indoor tracking instead of GPS, without
overwriting the calibration and tuning of each drone. To restore a backup to the very drone it was taken from, use both
flags. Further patterns can be excluded with the ``extra_excludes`` plugin setting, these are never applied.

Verification after reboot
-------------------------

Some parameters only take effect, or are only checked, after a reboot. With ``param-apply --verify``, the drone is
rebooted after the parameters have been written and the parameters are read again afterwards and compared to the file.
The read-back uses raw MAVLink ``PARAM_VALUE`` messages instead of the MAVSDK parameter cache, so the values compared
are those the flight controller actually reports after the reboot. How the reboot is handled depends on the connection:

- UDP/TCP, for example over WiFi: The connection is kept open. DroneManager waits for the drone heartbeat to disappear
  and to come back. If the drone does not come back within the timeout, a full reconnect is attempted.
- Serial, for example USB: The serial port disappears while the flight controller reboots, so DroneManager disconnects
  the drone and reconnects to it under the same name and address once the port is available again.

Example configuration in the ``plugin_settings`` section of the configuration file::

    "parameters": {
      "directory": "C:/Users/me/drone_params",
      "extra_excludes": ["RC_MAP_*", "COM_RC_IN_MODE"]
    }
"""
import asyncio
import datetime
import fnmatch
import logging
import math
import pathlib
import re
import struct
from collections.abc import Iterable
from dataclasses import dataclass, field

import numpy as np

import dronemanager
import dronemanager.core
from dronemanager.drone import DroneMAVSDK
from dronemanager.plugin import Plugin, DOC_DIR
from dronemanager.utils import parse_address


PARAM_DIR: pathlib.Path = DOC_DIR.joinpath("drone_parameters")
"""Default directory for parameter files, in the DroneManager directory in the user documents.

:meta hide-value:"""

PARAM_FILE_SUFFIX = ".params"
"""File extension of parameter files."""

MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9
INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}
FLOAT_PARAM_TYPES = {9, 10}
# struct formats for decoding integer parameters that are sent bytewise in the float field of PARAM_VALUE (PX4)
_BYTEWISE_FORMATS = {1: "<B", 2: "<b", 3: "<H", 4: "<h", 5: "<I", 6: "<i"}
PARAM_VALUE_MSG_ID = 22

CALIBRATION_PARAMS: dict[str, list[str]] = {
    "PX4": [
        "CAL_*",              # Sensor calibrations and sensor device IDs
        "SENS_BOARD_*_OFF",   # Level horizon calibration
        "SENS_DPRES_OFF",     # Airspeed sensor offset
        "TC_*",               # Thermal calibration
        "BAT*_V_DIV",         # Battery voltage and current calibration
        "BAT*_A_PER_V",
        "RC*_MIN",            # RC calibration
        "RC*_MAX",
        "RC*_TRIM",
        "RC*_REV",
        "RC*_DZ",
        "LND_FLIGHT_T_*",     # Flight time statistics
        "COM_FLIGHT_UUID",    # Flight counter
        "SYS_AUTOCONFIG",     # Triggers a parameter reset on the next boot
    ],
    "ArduPilot": [
        "INS_*OFFS*",         # IMU calibration
        "INS_*SCAL*",
        "INS_*_ID",           # IMU device IDs
        "INS_ACC*ID",
        "INS_GYR*ID",
        "COMPASS_OFS*",       # Compass calibration
        "COMPASS_DIA*",
        "COMPASS_ODI*",
        "COMPASS_MOT*",
        "COMPASS_SCALE*",
        "COMPASS_DEV_ID*",    # Compass device IDs
        "COMPASS_PRIO*",
        "BARO*_GND_PRESS",    # Barometer calibration
        "BARO*_DEVID",
        "ARSPD*_OFFSET",      # Airspeed sensor offset
        "AHRS_TRIM_*",        # Level calibration
        "BATT*_VOLT_MULT",    # Battery voltage and current calibration
        "BATT*_AMP_PERVLT",
        "BATT*_AMP_OFFSET",
        "RC*_MIN",            # RC calibration
        "RC*_MAX",
        "RC*_TRIM",
        "RC*_REVERSED",
        "RC*_DZ",
        "STAT_*",             # Statistics
        "FORMAT_VERSION",
        "SYSID_SW_*",
    ],
}
"""Calibration parameters per autopilot, as ``fnmatch`` patterns. Skipped unless ``--include_calibration`` is given.

If the autopilot is unknown, the patterns of all autopilots are used."""

TUNING_PARAMS: dict[str, list[str]] = {
    "PX4": [
        "MC_ROLLRATE_[PIDK]",  # Rate controller
        "MC_ROLLRATE_FF",
        "MC_PITCHRATE_[PIDK]",
        "MC_PITCHRATE_FF",
        "MC_YAWRATE_[PIDK]",
        "MC_YAWRATE_FF",
        "MC_RR_INT_LIM",
        "MC_PR_INT_LIM",
        "MC_YR_INT_LIM",
        "MC_ROLL_P",           # Attitude controller
        "MC_PITCH_P",
        "MC_YAW_P",
        "MC_YAW_WEIGHT",
        "MPC_XY_P",            # Position and velocity controller
        "MPC_Z_P",
        "MPC_XY_VEL_?_ACC",
        "MPC_Z_VEL_?_ACC",
        "MPC_THR_HOVER",       # Hover thrust
        "THR_MDL_FAC",         # Motor thrust curve
        "IMU_GYRO_CUTOFF",     # Gyro and accelerometer filters
        "IMU_DGYRO_CUTOFF",
        "IMU_ACCEL_CUTOFF",
        "IMU_GYRO_NF*",
        "IMU_GYRO_DNF_*",
    ],
    "ArduPilot": [
        "ATC_RAT_*",           # Rate controller
        "ATC_ANG_*_P",         # Attitude controller
        "ATC_ACCEL_*_MAX",
        "PSC_*",               # Position and velocity controller
        "MOT_THST_HOVER",      # Hover thrust and motor thrust curve
        "MOT_THST_EXPO",
        "MOT_SPIN_*",
        "INS_GYRO_FILTER",     # Gyro and accelerometer filters
        "INS_ACCEL_FILTER",
        "INS_HNTCH_*",
        "INS_HNTC2_*",
    ],
}
"""Tuning parameters per autopilot, as ``fnmatch`` patterns. Skipped unless ``--include_tuning`` is given.

If the autopilot is unknown, the patterns of all autopilots are used."""


@dataclass
class ParamValue:
    """A single parameter value."""

    name: str
    """The parameter name."""
    value: int | float | str
    """The parameter value."""
    kind: type | None = None
    """The python type of the parameter, ``int``, ``float`` or ``str``. ``None`` if unknown, i.e. read from a file
    without type information."""


@dataclass
class ParamDiff:
    """The difference between a set of target parameters and the parameters on a drone."""

    to_change: list[tuple[ParamValue, ParamValue]] = field(default_factory=list)
    """Pairs of (target, current) values for all parameters that differ."""
    unchanged: list[str] = field(default_factory=list)
    """Names of parameters that already have the target value."""
    excluded: list[str] = field(default_factory=list)
    """Names of parameters that were skipped due to the exclude patterns."""
    missing_on_drone: list[str] = field(default_factory=list)
    """Names of parameters in the target set that the drone doesn't have."""
    invalid: list[tuple[str, str]] = field(default_factory=list)
    """Pairs of (name, reason) for target values that can't be converted to the type on the drone."""


def parse_param_value(text: str, kind: type | None) -> int | float | str:
    """Parse a string into a parameter value of the given kind.

    Integers written as floats, i.e. "3.0", are accepted for integer parameters. If ``kind`` is ``None``, the value is
    parsed as an integer if possible and as a float otherwise.

    Args:
        text: The value as a string.
        kind: The type of the parameter, ``int``, ``float``, ``str`` or ``None`` if unknown.

    Returns:
        The parsed value.

    Raises:
        ValueError: If the text can't be parsed as the given kind without losing information.
    """
    text = text.strip()
    if kind is str:
        return text
    if kind is float:
        return float(text)
    try:
        return int(text)
    except ValueError:
        value = float(text)
        if kind is None:
            return value
        if not value.is_integer():
            raise ValueError(f"{text} is not an integer")
        return int(value)


def convert_value(value: int | float | str, kind: type | None) -> int | float | str:
    """Convert a value to the given kind.

    Args:
        value: The value to convert.
        kind: The target type, or ``None`` to keep the value as it is.

    Returns:
        The converted value.

    Raises:
        ValueError: If the conversion would lose information, for example 1.5 to an integer.
    """
    if kind is None:
        return value
    if isinstance(value, str) and kind is not str:
        return parse_param_value(value, kind)
    if kind is int:
        if isinstance(value, float) and not value.is_integer():
            raise ValueError(f"{value} is not an integer")
        return int(value)
    return kind(value)


def values_equal(a: int | float | str, b: int | float | str) -> bool:
    """Compare two parameter values.

    Floats are compared at 32-bit precision, as that is what the flight controller stores.

    Args:
        a: The first value.
        b: The second value.

    Returns:
        Whether the values are equal.
    """
    if isinstance(a, str) or isinstance(b, str):
        return str(a) == str(b)
    if isinstance(a, float) or isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return True
        try:
            return struct.pack("<f", a) == struct.pack("<f", b)
        except OverflowError:
            return math.isclose(a, b, rel_tol=1e-6)
    return a == b


def format_value(value: int | float | str, kind: type | None) -> str:
    """Format a parameter value for a parameter file or the log.

    Floats are formatted with the shortest representation that is exact at 32-bit precision, i.e. 0.1 instead of
    0.10000000149011612.

    Args:
        value: The value to format.
        kind: The type of the parameter.

    Returns:
        The formatted value.
    """
    if kind is float or isinstance(value, float):
        return np.format_float_positional(np.float32(value), unique=True, trim="0")
    return str(value)


def is_excluded(name: str, patterns: Iterable[str]) -> bool:
    """Check whether a parameter name matches any of the ``fnmatch`` patterns. Case sensitive.

    Args:
        name: The parameter name.
        patterns: The patterns to check.

    Returns:
        Whether the name matches any pattern.
    """
    return any(fnmatch.fnmatchcase(name, pattern) for pattern in patterns)


def patterns_for(groups: dict[str, list[str]], autopilot: str | None) -> list[str]:
    """Get the patterns of a parameter group for an autopilot.

    Args:
        groups: The parameter group, i.e. :py:data:`CALIBRATION_PARAMS`.
        autopilot: The autopilot of the drone, i.e. "PX4". If the autopilot is unknown, all patterns are returned.

    Returns:
        A list of ``fnmatch`` patterns.
    """
    if autopilot in groups:
        return list(groups[autopilot])
    return [pattern for patterns in groups.values() for pattern in patterns]


def read_params_file(path: pathlib.Path | str) -> dict[str, ParamValue]:
    """Read a QGroundControl ``.params`` file, or a Mission Planner style ``NAME,VALUE`` file.

    Args:
        path: The file to read.

    Returns:
        The parameters in the file, with their names as keys.

    Raises:
        ValueError: If a line can't be parsed. The message contains the line number.
    """
    path = pathlib.Path(path)
    params: dict[str, ParamValue] = {}
    for line_no, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            if "," in line and "\t" not in line:
                fields = [entry.strip() for entry in line.split(",")]
                if len(fields) != 2:
                    raise ValueError("expected NAME,VALUE")
                name, value_str = fields
                kind = None
            else:
                fields = line.split()
                if len(fields) != 5:
                    raise ValueError("expected 5 columns: Vehicle-Id, Component-Id, Name, Value, Type")
                _, _, name, value_str, type_str = fields
                param_type = int(type_str)
                if param_type in INT_PARAM_TYPES:
                    kind = int
                elif param_type in FLOAT_PARAM_TYPES:
                    kind = float
                else:
                    raise ValueError(f"unsupported parameter type {param_type}")
            params[name] = ParamValue(name, parse_param_value(value_str, kind), kind)
        except ValueError as e:
            raise ValueError(f"{path.name}, line {line_no}: {e}") from e
    return params


def write_params_file(path: pathlib.Path | str, params: Iterable[ParamValue], header: Iterable[str] = (),
                      system_id: int = 1, component_id: int = 1) -> int:
    """Write parameters to a QGroundControl ``.params`` file.

    Only integer and float parameters are written, others are skipped.

    Args:
        path: The file to write.
        params: The parameters to write.
        header: Lines for the comment header, without the leading "#".
        system_id: The MAVLink system ID written to the Vehicle-Id column.
        component_id: The MAVLink component ID written to the Component-Id column.

    Returns:
        The number of parameters written.
    """
    lines = [f"# {line}".rstrip() for line in header]
    lines.append("#")
    lines.append("# Vehicle-Id\tComponent-Id\tName\tValue\tType")
    count = 0
    for param in sorted(params, key=lambda p: p.name):
        if param.kind is int:
            param_type = MAV_PARAM_TYPE_INT32
        elif param.kind is float:
            param_type = MAV_PARAM_TYPE_REAL32
        else:
            continue
        value = format_value(param.value, param.kind)
        lines.append(f"{system_id}\t{component_id}\t{param.name}\t{value}\t{param_type}")
        count += 1
    pathlib.Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return count


def diff_params(target: dict[str, ParamValue], current: dict[str, ParamValue],
                excludes: Iterable[str] = ()) -> ParamDiff:
    """Compare target parameters to those currently on a drone.

    The type of each parameter on the drone takes precedence over the type in the target set.

    Args:
        target: The parameters that should be set.
        current: The parameters currently on the drone.
        excludes: ``fnmatch`` patterns for parameters that should be skipped.

    Returns:
        The differences between the two sets of parameters.
    """
    excludes = list(excludes)
    diff = ParamDiff()
    for name in sorted(target):
        if is_excluded(name, excludes):
            diff.excluded.append(name)
            continue
        if name not in current:
            diff.missing_on_drone.append(name)
            continue
        cur = current[name]
        try:
            value = convert_value(target[name].value, cur.kind)
        except ValueError as e:
            diff.invalid.append((name, str(e)))
            continue
        if values_equal(value, cur.value):
            diff.unchanged.append(name)
        else:
            diff.to_change.append((ParamValue(name, value, cur.kind), cur))
    return diff


def decode_param_value_msg(value: float, param_type: int, bytewise: bool) -> int | float:
    """Decode the float value field of a MAVLink PARAM_VALUE message.

    PX4 encodes integer parameters bytewise into the float field, ArduPilot casts them to float.

    Args:
        value: The value field of the message.
        param_type: The ``MAV_PARAM_TYPE`` of the parameter.
        bytewise: Whether integers are encoded bytewise.

    Returns:
        The decoded value.
    """
    if param_type in FLOAT_PARAM_TYPES:
        return float(value)
    if bytewise and param_type in _BYTEWISE_FORMATS:
        fmt = _BYTEWISE_FORMATS[param_type]
        raw = struct.pack("<f", value)
        return struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0]
    return int(round(value))


def _serial_port_present(port: str) -> bool | None:
    """Check whether a serial port currently exists.

    Args:
        port: The name of the port, i.e. "COM5" or "/dev/ttyACM0".

    Returns:
        Whether the port exists, or None if that can't be determined.
    """
    try:
        from serial.tools import list_ports
        return any(info.device == port for info in list_ports.comports()) or pathlib.Path(port).exists()
    except Exception:
        return None


def _short_list(names: list[str], limit: int = 20) -> str:
    """Join names for a log message, shortening long lists.

    Args:
        names: The names to join.
        limit: The maximum number of names shown.

    Returns:
        The joined names.
    """
    if len(names) <= limit:
        return ", ".join(names)
    return ", ".join(names[:limit]) + f", ... ({len(names) - limit} more)"


class ParametersPlugin(Plugin):
    """Save, apply and change flight controller parameters.

    CLI commands:

    * "save" - :py:meth:`save`: Save all parameters of a drone to a file.
    * "apply" - :py:meth:`apply`: Apply a parameter file to a drone, optionally verifying it after a reboot.
    * "diff" - :py:meth:`diff`: Show what applying a file would change.
    * "set" - :py:meth:`set_parameter`: Set a single parameter.
    * "get" - :py:meth:`get_parameter`: Read a single parameter.
    * "list" - :py:meth:`list_files`: List the saved parameter files.
    """

    PREFIX = "param"
    """The prefix for the CLI commands, "param" by default."""

    LINK_DROP_TIMEOUT = 15.0
    """How long to wait for the connection to drop after the reboot command, in seconds."""
    SETTLE_TIME = 3.0
    """How long to wait after the drone is back before reading parameters, in seconds."""
    RECONNECT_DELAY = 5.0
    """How long to wait after disconnecting before the first reconnection attempt, in seconds. Used if the serial port
    can't be monitored."""
    PORT_STABLE_TIME = 5.0
    """A serial port has to exist this long before reconnecting, in seconds. The bootloader of many flight
    controllers briefly uses the same port before the firmware starts."""
    CONNECT_ATTEMPT_TIMEOUT = 20.0
    """Timeout for a single reconnection attempt, in seconds."""
    FETCH_TIMEOUT = 60.0
    """Maximum time to read all parameters with raw MAVLink, in seconds."""
    FETCH_QUIET_TIME = 2.0
    """Missing parameters are requested again if no parameter arrived for this long, in seconds."""
    ADJUST_DELAY = 1.0
    """How long to wait after writing before checking whether the flight controller adjusted any parameters, in
    seconds."""
    APPLY_PASSES = 3
    """How often param-apply checks for and writes again parameters that the flight controller changed, i.e. due to
    limits that depend on other parameters."""

    def __init__(self, dm: "dronemanager.core.DroneManager", logger: logging.Logger, name: str,
                 directory: str | None = None, extra_excludes: list[str] | None = None):
        """Create the ParametersPlugin.

        Args:
            dm: The associated DroneManager instance.
            logger: The logger used for output and errors.
            name: The name of the plugin.
            directory: Directory for parameter files. Defaults to :py:data:`PARAM_DIR`.
            extra_excludes: Additional ``fnmatch`` patterns for parameters that should never be applied from a file.
        """
        super().__init__(dm, logger, name)
        self.directory: pathlib.Path = pathlib.Path(directory) if directory else PARAM_DIR
        """The directory where parameter files are stored."""
        self.directory.mkdir(parents=True, exist_ok=True)
        self.extra_excludes: list[str] = list(extra_excludes) if extra_excludes else []
        """Patterns for parameters that are never applied from a file."""
        self.cli_commands = {
            "save": self.save,
            "apply": self.apply,
            "diff": self.diff,
            "set": self.set_parameter,
            "get": self.get_parameter,
            "list": self.list_files,
        }
        self._busy: set[str] = set()

    async def status(self):
        """Log the parameter directory and the exclude patterns."""
        self.logger.info(f"Parameter files are stored in {self.directory}. Extra excludes: "
                         f"{', '.join(self.extra_excludes) if self.extra_excludes else 'none'}")

    # Helper functions ################################################################################################

    def _get_drone(self, drone: str) -> DroneMAVSDK | None:
        """Get a connected drone by name, logging a warning if there is none.

        Args:
            drone: The name of the drone.

        Returns:
            The drone object, or None if there is no such drone.
        """
        drone_obj = self.dm.drones.get(drone, None)
        if drone_obj is None:
            self.logger.warning(f"No drone named {drone}!")
        return drone_obj

    def _check_safe(self, drone: str, drone_obj: DroneMAVSDK, action: str) -> bool:
        """Check that the drone is disarmed and on the ground, logging a warning if not.

        Args:
            drone: The name of the drone.
            drone_obj: The drone object.
            action: Description of the action for the warning.

        Returns:
            Whether it is safe to change parameters.
        """
        if drone_obj.is_armed or drone_obj.in_air:
            self.logger.warning(f"Refusing to {action} on {drone} while it is armed or in the air!")
            return False
        return True

    def _start_operation(self, drone: str) -> bool:
        """Mark a drone as busy, so that parameter operations don't overlap.

        Args:
            drone: The name of the drone.

        Returns:
            False if another operation is already running for this drone.
        """
        if drone in self._busy:
            self.logger.warning(f"Another parameter operation is already running for {drone}!")
            return False
        self._busy.add(drone)
        return True

    def _resolve_file(self, filename: str) -> pathlib.Path:
        """Turn a file name into a path in the parameter directory.

        Args:
            filename: A file name or a path. ".params" is added if there is no extension.

        Returns:
            The path of the file.
        """
        path = pathlib.Path(filename)
        if not path.is_absolute() and path.parent == pathlib.Path("."):
            path = self.directory.joinpath(path)
        if not path.suffix:
            path = path.with_suffix(PARAM_FILE_SUFFIX)
        return path

    def _load_file(self, filename: str) -> tuple[pathlib.Path, dict[str, ParamValue] | None]:
        """Read a parameter file, logging a warning if that isn't possible.

        Args:
            filename: A file name or a path.

        Returns:
            The path of the file and its parameters, or None instead of the parameters if the file couldn't be read.
        """
        path = self._resolve_file(filename)
        if not path.is_file():
            self.logger.warning(f"No parameter file {path}! Use param-list to show available files.")
            return path, None
        try:
            params = read_params_file(path)
        except (ValueError, OSError) as e:
            self.logger.warning(f"Couldn't read parameter file: {e}")
            return path, None
        if not params:
            self.logger.warning(f"Parameter file {path} contains no parameters!")
            return path, None
        return path, params

    def _exclude_groups(self, drone_obj: DroneMAVSDK, include_calibration: bool,
                        include_tuning: bool) -> dict[str, list[str]]:
        """Get the patterns of parameters that should be skipped, by group.

        Args:
            drone_obj: The drone, used to determine the autopilot.
            include_calibration: Whether calibration parameters should be included, i.e. not skipped.
            include_tuning: Whether tuning parameters should be included, i.e. not skipped.

        Returns:
            A dictionary with the group names "calibration", "tuning" and "extra_excludes" as keys and the patterns of
            each skipped group as values.
        """
        groups = {"extra_excludes": list(self.extra_excludes)}
        if not include_calibration:
            groups["calibration"] = patterns_for(CALIBRATION_PARAMS, drone_obj.autopilot)
        if not include_tuning:
            groups["tuning"] = patterns_for(TUNING_PARAMS, drone_obj.autopilot)
        return groups

    async def _read_drone_params(self, drone_obj: DroneMAVSDK) -> dict[str, ParamValue]:
        """Read all parameters of a drone with MAVSDK.

        Args:
            drone_obj: The drone.

        Returns:
            The parameters, with their names as keys.
        """
        all_params = await drone_obj.system.param.get_all_params()
        params = {}
        for param in all_params.int_params:
            params[param.name] = ParamValue(param.name, param.value, int)
        for param in all_params.float_params:
            params[param.name] = ParamValue(param.name, param.value, float)
        for param in all_params.custom_params:
            params[param.name] = ParamValue(param.name, param.value, str)
        return params

    async def _lookup_param(self, drone_obj: DroneMAVSDK, name: str) -> ParamValue | None:
        """Read a single parameter from the drone.

        MAVSDK needs to know the type of the parameter. It is taken from the parameters cached on the drone object,
        which are refreshed if the name isn't among them. Without a cache, each type is tried in turn.

        Args:
            drone_obj: The drone.
            name: The name of the parameter. If it doesn't exist, the upper case version is tried as well.

        Returns:
            The parameter, or None if the drone doesn't have it.
        """
        candidates = [name] if name == name.upper() else [name, name.upper()]
        getters = {int: drone_obj.system.param.get_param_int,
                   float: drone_obj.system.param.get_param_float,
                   str: drone_obj.system.param.get_param_custom}
        drone_params = getattr(drone_obj, "drone_params", None)
        cached = drone_params.raw if drone_params is not None and drone_params.raw else {}
        if cached and not any(candidate in cached for candidate in candidates):
            # The parameter might have appeared since the cache was filled, i.e. after enabling a driver
            self._update_cache(drone_obj, (await self._read_drone_params(drone_obj)).values())
        if cached:
            candidates = [candidate for candidate in candidates if candidate in cached]
        for candidate in candidates:
            kinds = [cached[candidate][1]] if candidate in cached else [int, float, str]
            for kind in kinds:
                getter = getters[kind]
                try:
                    value = await getter(candidate)
                    return ParamValue(candidate, value, kind)
                except Exception as e:
                    self.logger.debug(f"Reading {candidate} as {kind.__name__} failed: {repr(e)}")
        return None

    async def _read_current_params(self, drone_obj: DroneMAVSDK) -> dict[str, ParamValue]:
        """Read all parameters as the flight controller currently reports them, bypassing the MAVSDK cache if possible.

        Args:
            drone_obj: The drone.

        Returns:
            The parameters, with their names as keys.
        """
        params = await self._fetch_params_mavlink(drone_obj)
        if params is None:
            params = await self._read_drone_params(drone_obj)
        return params

    async def _write_params(self, drone_obj: DroneMAVSDK,
                            changes: list[tuple[ParamValue, ParamValue]]) -> tuple[list[str], list[str]]:
        """Write parameters, logging each change.

        Args:
            drone_obj: The drone.
            changes: Pairs of (new, old) values.

        Returns:
            The names of all parameters that were attempted and the names of those that failed.
        """
        attempted = []
        failed = []
        written = []
        for new, old in changes:
            attempted.append(new.name)
            if await self._write_param(drone_obj, new):
                written.append(new)
                self.logger.info(f"\t{new.name}: {format_value(old.value, old.kind)} -> "
                                 f"{format_value(new.value, new.kind)}")
            else:
                failed.append(new.name)
        self._update_cache(drone_obj, written)
        return attempted, failed

    async def _rewrite_adjusted(self, drone_obj: DroneMAVSDK, target: dict[str, ParamValue], excludes: list[str],
                                attempted: list[str], failed: list[str]) -> list[str]:
        """Write parameters again that the flight controller changed after they were written.

        Flight controllers limit some parameters based on others, i.e. PX4 limits MPC_VEL_MANUAL to MPC_XY_VEL_MAX.
        Depending on the order in which they are written, a parameter can be changed back right after writing it.
        Writing it again once the parameters it depends on are set fixes this. Parameters that fail to write are
        added to ``failed``.

        Args:
            drone_obj: The drone.
            target: The parameters from the file.
            excludes: Patterns of parameters that are skipped.
            attempted: The names of the parameters that were written.
            failed: The names of the parameters that couldn't be written.

        Returns:
            The names of parameters that still don't have the target value after all passes.
        """
        checked = {name: target[name] for name in attempted if name not in failed}
        if not checked:
            return []
        for attempt in range(self.APPLY_PASSES):
            await asyncio.sleep(self.ADJUST_DELAY)
            current = await self._read_current_params(drone_obj)
            self._update_cache(drone_obj, current.values())
            diff = diff_params(checked, current, excludes)
            if not diff.to_change:
                return []
            for new, actual in diff.to_change:
                self.logger.info(f"The flight controller changed {new.name} to "
                                 f"{format_value(actual.value, actual.kind)} after it was written.")
            if attempt == self.APPLY_PASSES - 1:
                return [new.name for new, actual in diff.to_change]
            self.logger.info(f"Writing {len(diff.to_change)} parameters again...")
            _, retry_failed = await self._write_params(drone_obj, diff.to_change)
            failed.extend(name for name in retry_failed if name not in failed)
        return []

    async def _write_param(self, drone_obj: DroneMAVSDK, param: ParamValue) -> bool:
        """Write a single parameter.

        Args:
            drone_obj: The drone.
            param: The parameter with its new value.

        Returns:
            Whether the parameter was written.
        """
        try:
            if param.kind is int:
                await drone_obj.system.param.set_param_int(param.name, int(param.value))
            elif param.kind is float:
                await drone_obj.system.param.set_param_float(param.name, float(param.value))
            else:
                await drone_obj.system.param.set_param_custom(param.name, str(param.value))
            return True
        except Exception as e:
            self.logger.warning(f"Couldn't set {param.name} to {format_value(param.value, param.kind)}: {e}")
            self.logger.debug(repr(e), exc_info=True)
            return False

    @staticmethod
    def _update_cache(drone_obj: DroneMAVSDK, params: Iterable[ParamValue]):
        """Keep the parameters cached on the drone object (used by the ``params`` command) up to date.

        Args:
            drone_obj: The drone.
            params: The parameters with their current values.
        """
        drone_params = getattr(drone_obj, "drone_params", None)
        if drone_params is not None and drone_params.raw is not None:
            for param in params:
                drone_params.raw[param.name] = (param.value, param.kind)

    def _log_diff(self, drone: str, path: pathlib.Path, diff: ParamDiff, groups: dict[str, list[str]],
                  list_changes: bool):
        """Log a summary of the differences between a file and a drone.

        Args:
            drone: The name of the drone.
            path: The parameter file.
            diff: The differences.
            groups: The skipped parameter groups, see :py:meth:`_exclude_groups`.
            list_changes: Whether to log each parameter that would change.
        """
        self.logger.info(f"Comparing {path.name} to {drone}: {len(diff.to_change)} to change, "
                         f"{len(diff.unchanged)} unchanged, {len(diff.excluded)} skipped, "
                         f"{len(diff.missing_on_drone)} not on the drone, {len(diff.invalid)} invalid.")
        if list_changes:
            for new, old in diff.to_change:
                self.logger.info(f"\t{new.name}: {format_value(old.value, old.kind)} -> "
                                 f"{format_value(new.value, new.kind)}")
        for group, flag in (("calibration", "--include_calibration"), ("tuning", "--include_tuning"),
                            ("extra_excludes", None)):
            names = [name for name in diff.excluded if is_excluded(name, groups.get(group, []))]
            if not names:
                continue
            if flag is None:
                self.logger.info(f"Skipped {len(names)} parameters matching the extra_excludes plugin setting.")
            else:
                self.logger.info(f"Skipped {len(names)} {group} parameters. Use {flag} to include them.")
            self.logger.debug(f"Skipped {group} parameters: {', '.join(names)}")
        if diff.missing_on_drone:
            self.logger.warning(f"Parameters in the file that {drone} doesn't have: "
                                f"{_short_list(diff.missing_on_drone)}")
        for name, reason in diff.invalid:
            self.logger.warning(f"Invalid value for {name} in the file: {reason}")

    # CLI commands ####################################################################################################

    async def save(self, drone: str, filename: str | None = None, force: bool = False) -> pathlib.Path | bool:
        """Save all parameters of a drone to a .params file.

        The file is written in the QGroundControl format to the parameters directory, see param-list. All parameters
        are saved, including calibration and tuning.

        Args:
            drone: Name of the drone.
            filename: File name, ".params" is added if there is no extension. Defaults to <drone>_<date>_<time>.params.
            force: Overwrite the file if it already exists.

        Returns:
            The path of the file, or False if the parameters couldn't be saved.
        """
        drone_obj = self._get_drone(drone)
        if drone_obj is None:
            return False
        if filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = re.sub(r"[^\w\-]", "_", f"{drone}_{timestamp}")
        path = self._resolve_file(filename)
        if path.exists() and not force:
            self.logger.warning(f"{path} already exists! Use --force to overwrite it.")
            return False
        if not self._start_operation(drone):
            return False
        try:
            self.logger.info(f"Reading parameters from {drone}...")
            params = await self._read_drone_params(drone_obj)
            header = [f"Parameters of drone {drone}",
                      f"Autopilot: {drone_obj.autopilot}",
                      f"Saved: {datetime.datetime.now().isoformat(timespec='seconds')}",
                      f"Saved with DroneManager {dronemanager.__version__}"]
            path.parent.mkdir(parents=True, exist_ok=True)
            count = write_params_file(path, params.values(), header,
                                      system_id=drone_obj.drone_system_id or 1,
                                      component_id=drone_obj.drone_component_id or 1)
            skipped = len(params) - count
            self.logger.info(f"Saved {count} parameters of {drone} to {path}")
            if skipped:
                self.logger.warning(f"Skipped {skipped} parameters with custom types, which the file format "
                                    f"doesn't support.")
            return path
        except Exception as e:
            self.logger.error(f"Couldn't save the parameters of {drone}: {repr(e)}")
            self.logger.debug(repr(e), exc_info=True)
            return False
        finally:
            self._busy.discard(drone)

    async def diff(self, drone: str, filename: str, include_calibration: bool = False,
                   include_tuning: bool = False) -> ParamDiff | bool:
        """Show which parameters param-apply would change, without changing anything.

        Calibration and tuning parameters are skipped by default, as they are specific to each physical drone, just
        like with param-apply. Use --include_calibration and --include_tuning to compare them as well.

        Args:
            drone: Name of the drone.
            filename: Parameter file, either a name in the parameters directory or a path.
            include_calibration: Also compare calibration parameters (sensor, battery and RC calibration, device IDs,
              statistics), which are skipped by default.
            include_tuning: Also compare tuning parameters (controller gains, filters, hover thrust), which are
              skipped by default.

        Returns:
            The differences, or False if the comparison failed.
        """
        drone_obj = self._get_drone(drone)
        if drone_obj is None:
            return False
        path, target = self._load_file(filename)
        if target is None:
            return False
        if not self._start_operation(drone):
            return False
        try:
            groups = self._exclude_groups(drone_obj, include_calibration, include_tuning)
            current = await self._read_drone_params(drone_obj)
            diff = diff_params(target, current, [pattern for patterns in groups.values() for pattern in patterns])
            self._log_diff(drone, path, diff, groups, list_changes=True)
            return diff
        except Exception as e:
            self.logger.error(f"Couldn't compare the parameters of {drone}: {repr(e)}")
            self.logger.debug(repr(e), exc_info=True)
            return False
        finally:
            self._busy.discard(drone)

    async def apply(self, drone: str, filename: str, verify: bool = False, include_calibration: bool = False,
                    include_tuning: bool = False, reboot_timeout: float = 90.0) -> bool:
        """Apply a parameter file to a drone.

        Only parameters that differ from the drone are written. Calibration and tuning parameters are skipped by
        default, as they are specific to each physical drone. This way, one file can be applied to several drones.
        Use --include_calibration and --include_tuning to apply them as well, i.e. to restore a backup to the drone it
        was taken from. With --verify, the drone is rebooted afterwards and the parameters are read again and compared
        to the file. The drone must be disarmed.

        Args:
            drone: Name of the drone.
            filename: Parameter file, either a name in the parameters directory or a path.
            verify: Reboot the drone after applying and check that the parameters persisted.
            include_calibration: Also apply calibration parameters (sensor, battery and RC calibration, device IDs,
              statistics), which are skipped by default.
            include_tuning: Also apply tuning parameters (controller gains, filters, hover thrust), which are skipped
              by default.
            reboot_timeout: How long to wait for the drone to come back after the reboot, in seconds.

        Returns:
            True if all parameters were applied (and verified, if requested), False otherwise.
        """
        drone_obj = self._get_drone(drone)
        if drone_obj is None:
            return False
        if not self._check_safe(drone, drone_obj, "apply parameters"):
            return False
        path, target = self._load_file(filename)
        if target is None:
            return False
        if not self._start_operation(drone):
            return False
        try:
            groups = self._exclude_groups(drone_obj, include_calibration, include_tuning)
            excludes = [pattern for patterns in groups.values() for pattern in patterns]
            current = await self._read_drone_params(drone_obj)
            diff = diff_params(target, current, excludes)
            self._log_diff(drone, path, diff, groups, list_changes=False)
            attempted, failed = await self._write_params(drone_obj, diff.to_change)
            adjusted = await self._rewrite_adjusted(drone_obj, target, excludes, attempted, failed)
            if failed:
                self.logger.warning(f"Applied {len(attempted) - len(failed)} of {len(attempted)} changes to {drone}. "
                                    f"Failed: {_short_list(failed)}")
            elif adjusted:
                self.logger.warning(f"Applied {len(attempted)} changes from {path.name} to {drone}, but the flight "
                                    f"controller keeps changing {_short_list(adjusted)}. The file probably contains "
                                    f"values that conflict with each other.")
            else:
                self.logger.info(f"Applied {len(attempted)} changes from {path.name} to {drone}.")
            success = not failed and not adjusted and not diff.invalid
            if verify:
                verified = await self._verify_after_reboot(drone, target, excludes, set(diff.missing_on_drone),
                                                           reboot_timeout)
                success = success and verified
            return success
        except Exception as e:
            self.logger.error(f"Couldn't apply the parameters to {drone}: {repr(e)}")
            self.logger.debug(repr(e), exc_info=True)
            return False
        finally:
            self._busy.discard(drone)

    async def set_parameter(self, drone: str, name: str, value: str) -> bool:
        """Set a single parameter on a drone.

        The type of the parameter is determined from the drone. The value is read back afterwards to confirm the
        change, and other parameters that the flight controller changed along with it are reported. The drone must be
        disarmed.

        Args:
            drone: Name of the drone.
            name: Name of the parameter, i.e. MPC_XY_VEL_MAX.
            value: New value of the parameter.

        Returns:
            True if the parameter was set and read back with the new value, False otherwise.
        """
        drone_obj = self._get_drone(drone)
        if drone_obj is None:
            return False
        if not self._check_safe(drone, drone_obj, "set parameters"):
            return False
        if not self._start_operation(drone):
            return False
        try:
            old = await self._lookup_param(drone_obj, name)
            if old is None:
                self.logger.warning(f"{drone} has no parameter {name}!")
                return False
            try:
                new_value = parse_param_value(value, old.kind)
            except ValueError:
                self.logger.warning(f"{value} is not a valid value for {old.name}, which is of type "
                                    f"{old.kind.__name__}!")
                return False
            new = ParamValue(old.name, new_value, old.kind)
            before = await self._fetch_params_mavlink(drone_obj)
            if not await self._write_param(drone_obj, new):
                return False
            readback = await self._lookup_param(drone_obj, old.name)
            if readback is None or not values_equal(readback.value, new_value):
                actual = "nothing" if readback is None else format_value(readback.value, readback.kind)
                self.logger.warning(f"Set {old.name} on {drone} to {format_value(new_value, new.kind)}, but read "
                                    f"back {actual}!")
                return False
            self._update_cache(drone_obj, [readback])
            self.logger.info(f"{drone}: {old.name} {format_value(old.value, old.kind)} -> "
                             f"{format_value(readback.value, readback.kind)}")
            if before is not None:
                await self._report_side_effects(drone, drone_obj, old.name, before)
            return True
        except Exception as e:
            self.logger.error(f"Couldn't set {name} on {drone}: {repr(e)}")
            self.logger.debug(repr(e), exc_info=True)
            return False
        finally:
            self._busy.discard(drone)

    async def _report_side_effects(self, drone: str, drone_obj: DroneMAVSDK, name: str,
                                   before: dict[str, ParamValue]):
        """Log other parameters that changed while setting one, usually adjusted by the flight controller.

        Args:
            drone: The name of the drone.
            drone_obj: The drone.
            name: The name of the parameter that was set.
            before: All parameters before setting it.
        """
        await asyncio.sleep(self.ADJUST_DELAY)
        after = await self._fetch_params_mavlink(drone_obj) or {}
        self._update_cache(drone_obj, after.values())
        changed = [f"{other} {format_value(before[other].value, before[other].kind)} -> "
                   f"{format_value(param.value, param.kind)}"
                   for other, param in sorted(after.items())
                   if other != name and other in before and not values_equal(before[other].value, param.value)]
        if changed:
            self.logger.warning(f"Other parameters on {drone} changed as well, usually because the flight controller "
                                f"limits them based on {name}: {', '.join(changed)}")

    async def get_parameter(self, drone: str, name: str) -> int | float | str | None:
        """Read a single parameter from a drone.

        Args:
            drone: Name of the drone.
            name: Name of the parameter, i.e. MPC_XY_VEL_MAX.

        Returns:
            The value of the parameter, or None if the drone doesn't have it.
        """
        drone_obj = self._get_drone(drone)
        if drone_obj is None:
            return None
        param = await self._lookup_param(drone_obj, name)
        if param is None:
            self.logger.warning(f"{drone} has no parameter {name}!")
            return None
        self.logger.info(f"{drone}: {param.name} = {format_value(param.value, param.kind)} ({param.kind.__name__})")
        return param.value

    async def list_files(self) -> list[pathlib.Path]:
        """List the parameter files in the parameters directory.

        Returns:
            A list with the paths of the parameter files.
        """
        files = sorted(path for path in self.directory.iterdir()
                       if path.is_file() and path.suffix in (PARAM_FILE_SUFFIX, ".param"))
        if not files:
            self.logger.info(f"No parameter files in {self.directory}")
            return files
        lines = [f"Parameter files in {self.directory}:"]
        for path in files:
            modified = datetime.datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
            try:
                count = f"{len(read_params_file(path))} parameters"
            except (ValueError, OSError):
                count = "unreadable"
            lines.append(f"\t{path.name} ({count}, {modified})")
        self.logger.info("\n".join(lines))
        return files

    # Reboot verification #############################################################################################

    async def _verify_after_reboot(self, drone: str, target: dict[str, ParamValue], excludes: list[str],
                                   missing_before: set[str], reboot_timeout: float) -> bool:
        """Reboot the drone, read its parameters again and compare them to the target values.

        Args:
            drone: The name of the drone.
            target: The parameters from the file.
            excludes: Patterns of parameters that are skipped.
            missing_before: Names of parameters that the drone didn't have before the reboot.
            reboot_timeout: How long to wait for the drone to come back, in seconds.

        Returns:
            Whether all parameters have the target value after the reboot.
        """
        drone_obj = self.dm.drones[drone]
        address = drone_obj.drone_addr
        scheme, _, _ = parse_address(address)
        self.logger.info(f"Rebooting {drone} to verify the parameters...")
        try:
            await asyncio.wait_for(drone_obj.system.action.reboot(), 5)
        except Exception as e:
            # The acknowledgement is often lost because the flight controller reboots right away
            self.logger.debug(f"Reboot command didn't complete cleanly: {repr(e)}")

        if scheme == "serial":
            drone_obj = await self._reconnect(drone, drone_obj, address, reboot_timeout)
        else:
            rebooted = await self._wait_for_link_cycle(drone, drone_obj, reboot_timeout)
            if rebooted is None:
                self.logger.warning(f"{drone} didn't come back after the reboot, trying to reconnect...")
                drone_obj = await self._reconnect(drone, drone_obj, address, reboot_timeout)
            elif not rebooted:
                self.logger.error(f"Verification FAILED: Couldn't confirm that {drone} rebooted. The connection "
                                  f"never dropped.")
                return False
        if drone_obj is None:
            self.logger.error(f"Verification FAILED: Couldn't reconnect to {drone} after the reboot!")
            return False

        self.logger.info(f"Reading parameters from {drone} after the reboot...")
        after = await self._fetch_params_mavlink(drone_obj)
        if after is None:
            self.logger.warning("Couldn't read parameters with raw MAVLink, using MAVSDK instead.")
            after = await self._read_drone_params(drone_obj)
        self._update_cache(drone_obj, after.values())
        return self._report_verification(drone, target, after, excludes, missing_before)

    def _report_verification(self, drone: str, target: dict[str, ParamValue], after: dict[str, ParamValue],
                             excludes: list[str], missing_before: set[str]) -> bool:
        """Compare the parameters after the reboot to the target values and log the result.

        Args:
            drone: The name of the drone.
            target: The parameters from the file.
            after: The parameters after the reboot.
            excludes: Patterns of parameters that are skipped.
            missing_before: Names of parameters that the drone didn't have before the reboot.

        Returns:
            Whether the verification passed.
        """
        mismatches = []
        lost = []
        appeared = []
        checked = 0
        for name in sorted(target):
            if is_excluded(name, excludes):
                continue
            if name not in after:
                if name not in missing_before:
                    lost.append(name)
                continue
            if name in missing_before:
                appeared.append(name)  # Couldn't be applied, as the drone didn't have it at the time
                continue
            try:
                expected = convert_value(target[name].value, after[name].kind)
            except ValueError:
                continue  # Already reported as invalid
            checked += 1
            if not values_equal(expected, after[name].value):
                mismatches.append(f"{name}: expected {format_value(expected, after[name].kind)}, "
                                  f"got {format_value(after[name].value, after[name].kind)}")
        if appeared:
            self.logger.warning(f"{len(appeared)} parameters from the file only exist after the reboot (usually "
                                f"enabled by other parameters): {_short_list(appeared)}. Run param-apply again to "
                                f"set them.")
        if mismatches or lost:
            self.logger.error(f"Verification FAILED for {drone}: {len(mismatches)} of {checked} parameters differ "
                              f"from the file after the reboot, {len(lost)} disappeared.")
            for line in mismatches[:50]:
                self.logger.error(f"\t{line}")
            if len(mismatches) > 50:
                self.logger.error(f"\t... and {len(mismatches) - 50} more, see the log file.")
                for line in mismatches[50:]:
                    self.logger.debug(f"\t{line}")
            if lost:
                self.logger.error(f"Parameters that disappeared after the reboot: {_short_list(lost)}")
            return False
        self.logger.info(f"Verification PASSED for {drone}: All {checked} parameters match the file after the "
                         f"reboot.")
        return True

    async def _wait_for_link_cycle(self, drone: str, drone_obj: DroneMAVSDK, timeout: float) -> bool | None:
        """Wait for the connection to drop and come back.

        Args:
            drone: The name of the drone.
            drone_obj: The drone.
            timeout: How long to wait in total, in seconds.

        Returns:
            True if the connection dropped and came back, False if it never dropped and None if it didn't come back.
        """
        loop = asyncio.get_running_loop()
        start = loop.time()
        dropped = False
        while loop.time() - start < min(self.LINK_DROP_TIMEOUT, timeout):
            if not drone_obj.is_connected:
                dropped = True
                break
            await asyncio.sleep(0.1)
        if not dropped:
            return False
        self.logger.info(f"Lost connection to {drone}, waiting for it to come back...")
        while loop.time() - start < timeout:
            if drone_obj.is_connected:
                self.logger.info(f"{drone} is back after {loop.time() - start:.1f}s.")
                await asyncio.sleep(self.SETTLE_TIME)
                return True
            await asyncio.sleep(0.2)
        return None

    async def _reconnect(self, drone: str, drone_obj: DroneMAVSDK, address: str,
                         timeout: float) -> DroneMAVSDK | None:
        """Disconnect the drone and connect to it again under the same name and address.

        Args:
            drone: The name of the drone.
            drone_obj: The drone object before the reboot.
            address: The connection address.
            timeout: How long to try reconnecting, in seconds.

        Returns:
            The new drone object, or None if the reconnection failed.
        """
        telemetry_frequency = getattr(getattr(drone_obj, "config", None), "position_rate", None)
        await self.dm.disconnect(drone, force=True)
        loop = asyncio.get_running_loop()
        start = loop.time()
        scheme, port, _ = parse_address(address)
        if scheme == "serial" and _serial_port_present(port) is not None:
            await self._wait_for_serial_port(port, start + timeout)
        else:
            await asyncio.sleep(self.RECONNECT_DELAY)
        while (remaining := timeout - (loop.time() - start)) > 0:
            self.logger.info(f"Reconnecting to {drone} @ {address}...")
            connected = await self.dm.connect_to_drone(drone, drone_address=address,
                                                       timeout=min(self.CONNECT_ATTEMPT_TIMEOUT, remaining),
                                                       telemetry_frequency=telemetry_frequency,
                                                       log_telemetry=None)
            if connected and drone in self.dm.drones:
                await asyncio.sleep(self.SETTLE_TIME)
                return self.dm.drones[drone]
            await asyncio.sleep(2)
        return None

    async def _wait_for_serial_port(self, port: str, deadline: float):
        """Wait for the serial port to disappear during the reboot and then to be available for a while.

        Args:
            port: The name of the port.
            deadline: The event loop time after which to stop waiting.
        """
        loop = asyncio.get_running_loop()
        gone_by = loop.time() + 10
        while _serial_port_present(port) and loop.time() < gone_by:
            await asyncio.sleep(0.1)
        self.logger.info(f"Waiting for {port} to come back...")
        present_since = None
        while loop.time() < deadline:
            if not _serial_port_present(port):
                present_since = None
            elif present_since is None:
                present_since = loop.time()
            elif loop.time() - present_since >= self.PORT_STABLE_TIME:
                self.logger.info(f"{port} is available again.")
                break
            await asyncio.sleep(0.2)

    async def _fetch_params_mavlink(self, drone_obj: DroneMAVSDK) -> dict[str, ParamValue] | None:
        """Read all parameters with PARAM_REQUEST_LIST, independent of the MAVSDK parameter cache.

        Args:
            drone_obj: The drone.

        Returns:
            The parameters, or None if there is no MAVLink connection to use or no parameters were received.
        """
        mav = getattr(drone_obj, "mav_conn", None)
        if mav is None or mav.con_drone_in is None:
            return None
        loop = asyncio.get_running_loop()
        bytewise = drone_obj.autopilot == "PX4"
        target_system = mav.drone_system
        target_component = drone_obj.drone_component_id or 1
        received: dict[str, ParamValue] = {}
        indices: set[int] = set()
        param_count = None
        last_received = loop.time()

        async def on_param_value(msg: object):
            """Collect a PARAM_VALUE message from the flight controller.

            Args:
                msg: The pymavlink PARAM_VALUE message.
            """
            nonlocal param_count, last_received
            if msg.get_srcSystem() == target_system and msg.get_srcComponent() == target_component:
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode("ascii", errors="ignore")
                param_id = param_id.rstrip("\x00")
                kind = float if msg.param_type in FLOAT_PARAM_TYPES else int
                value = decode_param_value_msg(msg.param_value, msg.param_type, bytewise)
                received[param_id] = ParamValue(param_id, value, kind)
                if msg.param_index < msg.param_count:
                    indices.add(msg.param_index)
                param_count = msg.param_count
                last_received = loop.time()

        mav.add_drone_message_callback(PARAM_VALUE_MSG_ID, on_param_value)
        try:
            mav.send_as_gcs(mav.con_drone_in.mav.param_request_list_encode(target_system, target_component))
            start = loop.time()
            while loop.time() - start < self.FETCH_TIMEOUT:
                await asyncio.sleep(0.2)
                if param_count is not None and len(indices) >= param_count:
                    break
                if loop.time() - last_received > self.FETCH_QUIET_TIME:
                    if param_count is None:
                        mav.send_as_gcs(mav.con_drone_in.mav.param_request_list_encode(target_system,
                                                                                       target_component))
                    else:
                        missing = sorted(set(range(param_count)) - indices)
                        self.logger.debug(f"Requesting {len(missing)} missing parameters again")
                        for index in missing:
                            mav.send_as_gcs(mav.con_drone_in.mav.param_request_read_encode(
                                target_system, target_component, b"", index))
                    last_received = loop.time()
        finally:
            mav.remove_drone_message_callback(PARAM_VALUE_MSG_ID, on_param_value)
        if not received:
            return None
        if param_count is not None and len(indices) < param_count:
            self.logger.warning(f"Only received {len(indices)} of {param_count} parameters!")
        return received
