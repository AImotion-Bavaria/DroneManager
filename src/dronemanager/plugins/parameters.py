""" Save, apply and change flight controller parameters.

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

By default, the files are stored in ``src/dronemanager/resources/parameters``. This directory is part of the repository,
but the files in it are ignored by git, so saved configurations stay local. A different directory can be set with the
``directory`` entry in the plugin settings of the configuration file.

Calibration parameters
----------------------

When applying a file, parameters that are specific to a single airframe or flight controller are skipped by default.
These are sensor calibrations, sensor device IDs and statistics counters, see :py:data:`DEFAULT_EXCLUDES`. Copying
these from one drone to another would overwrite the calibration of the target drone. They can be included with
``--include_calibration``, for example to restore a backup to the very drone it was taken from. Further patterns can be
excluded with the ``extra_excludes`` plugin setting.

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
      "extra_excludes": ["RC*_TRIM", "BAT1_V_DIV"]
    }
"""
import asyncio
import datetime
import fnmatch
import math
import pathlib
import re
import struct
from collections.abc import Iterable
from dataclasses import dataclass, field

import numpy as np

import dronemanager
from dronemanager.plugin import Plugin
from dronemanager.utils import SRC_DIR, parse_address


PARAM_DIR = SRC_DIR.joinpath("resources", "parameters")
"""Default directory for parameter files.

:meta hide-value:
"""

PARAM_FILE_SUFFIX = ".params"

MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9
INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}
FLOAT_PARAM_TYPES = {9, 10}
# struct formats for decoding integer parameters that are sent bytewise in the float field of PARAM_VALUE (PX4)
_BYTEWISE_FORMATS = {1: "<B", 2: "<b", 3: "<H", 4: "<h", 5: "<I", 6: "<i"}
PARAM_VALUE_MSG_ID = 22

DEFAULT_EXCLUDES: dict[str, list[str]] = {
    "PX4": [
        "CAL_*",            # Sensor calibrations and sensor device IDs
        "SENS_BOARD_*_OFF",  # Level horizon calibration
        "TC_*",             # Thermal calibration
        "LND_FLIGHT_T_*",   # Flight time statistics
        "COM_FLIGHT_UUID",  # Flight counter
        "SYS_AUTOCONFIG",   # Triggers a parameter reset on the next boot
    ],
    "ArduPilot": [
        "INS_*OFFS*",       # IMU calibration
        "INS_*SCAL*",
        "INS_*_ID",         # IMU device IDs
        "INS_ACC*ID",
        "INS_GYR*ID",
        "COMPASS_OFS*",     # Compass calibration
        "COMPASS_DIA*",
        "COMPASS_ODI*",
        "COMPASS_MOT*",
        "COMPASS_SCALE*",
        "COMPASS_DEV_ID*",  # Compass device IDs
        "COMPASS_PRIO*",
        "BARO*_GND_PRESS",  # Barometer calibration
        "BARO*_DEVID",
        "AHRS_TRIM_*",      # Level calibration
        "STAT_*",           # Statistics
        "FORMAT_VERSION",
        "SYSID_SW_*",
    ],
}
"""Parameters skipped when applying a file, per autopilot, as ``fnmatch`` patterns.

If the autopilot is unknown, the patterns of all autopilots are used."""


@dataclass
class ParamValue:
    """ A single parameter value."""
    #: The parameter name.
    name: str
    #: The parameter value.
    value: int | float | str
    #: The python type of the parameter, ``int``, ``float`` or ``str``. ``None`` if unknown, i.e. read from a file
    #: without type information.
    kind: type | None = None


@dataclass
class ParamDiff:
    """ The difference between a set of target parameters and the parameters on a drone."""
    #: Pairs of (target, current) values for all parameters that differ.
    to_change: list[tuple[ParamValue, ParamValue]] = field(default_factory=list)
    #: Names of parameters that already have the target value.
    unchanged: list[str] = field(default_factory=list)
    #: Names of parameters that were skipped due to the exclude patterns.
    excluded: list[str] = field(default_factory=list)
    #: Names of parameters in the target set that the drone doesn't have.
    missing_on_drone: list[str] = field(default_factory=list)
    #: Pairs of (name, reason) for target values that can't be converted to the type on the drone.
    invalid: list[tuple[str, str]] = field(default_factory=list)


def parse_param_value(text: str, kind: type | None) -> int | float | str:
    """ Parse a string into a parameter value of the given kind.

    Integers written as floats, i.e. "3.0", are accepted for integer parameters. If ``kind`` is ``None``, the value is
    parsed as an integer if possible and as a float otherwise.

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
    """ Convert a value to the given kind.

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
    """ Compare two parameter values.

    Floats are compared at 32-bit precision, as that is what the flight controller stores.
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
    """ Format a parameter value for a parameter file or the log.

    Floats are formatted with the shortest representation that is exact at 32-bit precision, i.e. 0.1 instead of
    0.10000000149011612.
    """
    if kind is float or isinstance(value, float):
        return np.format_float_positional(np.float32(value), unique=True, trim="0")
    return str(value)


def is_excluded(name: str, patterns: Iterable[str]) -> bool:
    """ Whether the parameter name matches any of the ``fnmatch`` patterns. Case sensitive."""
    return any(fnmatch.fnmatchcase(name, pattern) for pattern in patterns)


def default_excludes(autopilot: str | None) -> list[str]:
    """ The default exclude patterns for an autopilot, or all patterns if the autopilot is unknown."""
    if autopilot in DEFAULT_EXCLUDES:
        return list(DEFAULT_EXCLUDES[autopilot])
    return [pattern for patterns in DEFAULT_EXCLUDES.values() for pattern in patterns]


def read_params_file(path: pathlib.Path | str) -> dict[str, ParamValue]:
    """ Read a QGroundControl ``.params`` file, or a Mission Planner style ``NAME,VALUE`` file.

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
    """ Write parameters to a QGroundControl ``.params`` file.

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
        lines.append(f"{system_id}\t{component_id}\t{param.name}\t{format_value(param.value, param.kind)}\t{param_type}")
        count += 1
    pathlib.Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return count


def diff_params(target: dict[str, ParamValue], current: dict[str, ParamValue],
                excludes: Iterable[str] = ()) -> ParamDiff:
    """ Compare target parameters to those currently on a drone.

    The type of each parameter on the drone takes precedence over the type in the target set.
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
    """ Decode the float value field of a MAVLink PARAM_VALUE message.

    PX4 encodes integer parameters bytewise into the float field, ArduPilot casts them to float.
    """
    if param_type in FLOAT_PARAM_TYPES:
        return float(value)
    if bytewise and param_type in _BYTEWISE_FORMATS:
        fmt = _BYTEWISE_FORMATS[param_type]
        raw = struct.pack("<f", value)
        return struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0]
    return int(round(value))


def _serial_port_present(port: str) -> bool | None:
    """ Whether a serial port currently exists, or None if that can't be determined."""
    try:
        from serial.tools import list_ports
        return any(info.device == port for info in list_ports.comports()) or pathlib.Path(port).exists()
    except Exception:
        return None


def _short_list(names: list[str], limit: int = 20) -> str:
    if len(names) <= limit:
        return ", ".join(names)
    return ", ".join(names[:limit]) + f", ... ({len(names) - limit} more)"


class ParametersPlugin(Plugin):
    """ Save, apply and change flight controller parameters.

    Args:
        directory: Directory for parameter files. Defaults to :py:data:`PARAM_DIR`.
        extra_excludes: Additional ``fnmatch`` patterns for parameters that should never be applied from a file.
    """

    PREFIX = "param"

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

    def __init__(self, dm, logger, name, directory: str | None = None, extra_excludes: list[str] | None = None):
        super().__init__(dm, logger, name)
        self.directory = pathlib.Path(directory) if directory else PARAM_DIR
        self.directory.mkdir(parents=True, exist_ok=True)
        self.extra_excludes = list(extra_excludes) if extra_excludes else []
        self.cli_commands = {
            "save": self.save,
            "apply": self.apply,
            "diff": self.diff,
            "set": self.set_parameter,
            "get": self.get_parameter,
            "list": self.list_files,
        }
        self._busy: set[str] = set()

    # Helper functions ################################################################################################

    def _get_drone(self, drone: str):
        drone_obj = self.dm.drones.get(drone, None)
        if drone_obj is None:
            self.logger.warning(f"No drone named {drone}!")
        return drone_obj

    def _check_safe(self, drone: str, drone_obj, action: str) -> bool:
        if drone_obj.is_armed or drone_obj.in_air:
            self.logger.warning(f"Refusing to {action} on {drone} while it is armed or in the air!")
            return False
        return True

    def _start_operation(self, drone: str) -> bool:
        if drone in self._busy:
            self.logger.warning(f"Another parameter operation is already running for {drone}!")
            return False
        self._busy.add(drone)
        return True

    def _resolve_file(self, filename: str) -> pathlib.Path:
        path = pathlib.Path(filename)
        if not path.is_absolute() and path.parent == pathlib.Path("."):
            path = self.directory.joinpath(path)
        if not path.suffix:
            path = path.with_suffix(PARAM_FILE_SUFFIX)
        return path

    def _load_file(self, filename: str) -> tuple[pathlib.Path, dict[str, ParamValue] | None]:
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

    def _excludes(self, drone_obj, include_calibration: bool) -> list[str]:
        excludes = list(self.extra_excludes)
        if not include_calibration:
            excludes.extend(default_excludes(drone_obj.autopilot))
        return excludes

    async def _read_drone_params(self, drone_obj) -> dict[str, ParamValue]:
        all_params = await drone_obj.system.param.get_all_params()
        params = {}
        for param in all_params.int_params:
            params[param.name] = ParamValue(param.name, param.value, int)
        for param in all_params.float_params:
            params[param.name] = ParamValue(param.name, param.value, float)
        for param in all_params.custom_params:
            params[param.name] = ParamValue(param.name, param.value, str)
        return params

    async def _lookup_param(self, drone_obj, name: str) -> ParamValue | None:
        """ Read a single parameter from the drone.

        MAVSDK needs to know the type of the parameter. It is taken from the parameters cached on the drone object,
        which are refreshed if the name isn't among them. Without a cache, each type is tried in turn.
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

    async def _read_current_params(self, drone_obj) -> dict[str, ParamValue]:
        """ Read all parameters as the flight controller currently reports them, bypassing the MAVSDK cache."""
        params = await self._fetch_params_mavlink(drone_obj)
        if params is None:
            params = await self._read_drone_params(drone_obj)
        return params

    async def _write_params(self, drone_obj, changes: list[tuple[ParamValue, ParamValue]]) \
            -> tuple[list[str], list[str]]:
        """ Write (new, old) pairs of parameters, logging each change.

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

    async def _rewrite_adjusted(self, drone_obj, target: dict[str, ParamValue], excludes: list[str],
                                attempted: list[str], failed: list[str]) -> list[str]:
        """ Write parameters again that the flight controller changed after they were written.

        Flight controllers limit some parameters based on others, i.e. PX4 limits MPC_VEL_MANUAL to MPC_XY_VEL_MAX.
        Depending on the order in which they are written, a parameter can be changed back right after writing it.
        Writing it again once the parameters it depends on are set fixes this. Parameters that fail to write are
        added to ``failed``.

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

    async def _write_param(self, drone_obj, param: ParamValue) -> bool:
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
    def _update_cache(drone_obj, params: Iterable[ParamValue]):
        """ Keep the parameters cached on the drone object (used by the ``params`` command) up to date."""
        drone_params = getattr(drone_obj, "drone_params", None)
        if drone_params is not None and drone_params.raw is not None:
            for param in params:
                drone_params.raw[param.name] = (param.value, param.kind)

    def _log_diff(self, drone: str, path: pathlib.Path, diff: ParamDiff, list_changes: bool):
        self.logger.info(f"Comparing {path.name} to {drone}: {len(diff.to_change)} to change, "
                         f"{len(diff.unchanged)} unchanged, {len(diff.excluded)} excluded, "
                         f"{len(diff.missing_on_drone)} not on the drone, {len(diff.invalid)} invalid.")
        if list_changes:
            for new, old in diff.to_change:
                self.logger.info(f"\t{new.name}: {format_value(old.value, old.kind)} -> "
                                 f"{format_value(new.value, new.kind)}")
        if diff.excluded:
            self.logger.info(f"Skipped {len(diff.excluded)} calibration/hardware specific parameters. Use "
                             f"--include_calibration to apply them as well.")
            self.logger.debug(f"Excluded parameters: {', '.join(diff.excluded)}")
        if diff.missing_on_drone:
            self.logger.warning(f"Parameters in the file that {drone} doesn't have: "
                                f"{_short_list(diff.missing_on_drone)}")
        for name, reason in diff.invalid:
            self.logger.warning(f"Invalid value for {name} in the file: {reason}")

    # CLI commands ####################################################################################################

    async def save(self, drone: str, filename: str | None = None, force: bool = False):
        """ Save all parameters of a drone to a .params file.

        The file is written in the QGroundControl format to the parameters directory, see param-list.

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

    async def diff(self, drone: str, filename: str, include_calibration: bool = False):
        """ Show which parameters param-apply would change, without changing anything.

        Args:
            drone: Name of the drone.
            filename: Parameter file, either a name in the parameters directory or a path.
            include_calibration: Also compare calibration and hardware specific parameters, which are skipped by
              default.

        Returns:
            The :py:class:`ParamDiff`, or False if the comparison failed.
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
            current = await self._read_drone_params(drone_obj)
            diff = diff_params(target, current, self._excludes(drone_obj, include_calibration))
            self._log_diff(drone, path, diff, list_changes=True)
            return diff
        except Exception as e:
            self.logger.error(f"Couldn't compare the parameters of {drone}: {repr(e)}")
            self.logger.debug(repr(e), exc_info=True)
            return False
        finally:
            self._busy.discard(drone)

    async def apply(self, drone: str, filename: str, verify: bool = False, include_calibration: bool = False,
                    reboot_timeout: float = 90.0):
        """ Apply a parameter file to a drone.

        Only parameters that differ from the drone are written. Calibration and hardware specific parameters are
        skipped unless --include_calibration is given. With --verify, the drone is rebooted afterwards and the
        parameters are read again and compared to the file. The drone must be disarmed.

        Args:
            drone: Name of the drone.
            filename: Parameter file, either a name in the parameters directory or a path.
            verify: Reboot the drone after applying and check that the parameters persisted.
            include_calibration: Also apply calibration and hardware specific parameters, i.e. when restoring a
              backup to the drone it was taken from.
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
            excludes = self._excludes(drone_obj, include_calibration)
            current = await self._read_drone_params(drone_obj)
            diff = diff_params(target, current, excludes)
            self._log_diff(drone, path, diff, list_changes=False)
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

    async def set_parameter(self, drone: str, name: str, value: str):
        """ Set a single parameter on a drone.

        The type of the parameter is determined from the drone. The value is read back afterwards to confirm the
        change. The drone must be disarmed.

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

    async def _report_side_effects(self, drone: str, drone_obj, name: str, before: dict[str, ParamValue]):
        """ Log other parameters that changed while setting one, usually adjusted by the flight controller."""
        await asyncio.sleep(self.ADJUST_DELAY)
        after = await self._fetch_params_mavlink(drone_obj)
        if after is None:
            return
        self._update_cache(drone_obj, after.values())
        changed = [f"{other} {format_value(before[other].value, before[other].kind)} -> "
                   f"{format_value(param.value, param.kind)}"
                   for other, param in sorted(after.items())
                   if other != name and other in before and not values_equal(before[other].value, param.value)]
        if changed:
            self.logger.warning(f"Other parameters on {drone} changed as well, usually because the flight controller "
                                f"limits them based on {name}: {', '.join(changed)}")

    async def get_parameter(self, drone: str, name: str):
        """ Read a single parameter from a drone.

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

    async def list_files(self):
        """ List the parameter files in the parameters directory.

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

    async def _wait_for_link_cycle(self, drone: str, drone_obj, timeout: float) -> bool | None:
        """ Wait for the connection to drop and come back.

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

    async def _reconnect(self, drone: str, drone_obj, address: str, timeout: float):
        """ Disconnect the drone and connect to it again under the same name and address.

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
        """ Wait for the serial port to disappear during the reboot and then to be available for a while."""
        loop = asyncio.get_running_loop()
        gone_by = loop.time() + 10
        while _serial_port_present(port) and loop.time() < gone_by:
            await asyncio.sleep(0.1)
        self.logger.info(f"Waiting for {port} to come back...")
        present_since = None
        while loop.time() < deadline:
            if _serial_port_present(port):
                if present_since is None:
                    present_since = loop.time()
                elif loop.time() - present_since >= self.PORT_STABLE_TIME:
                    self.logger.info(f"{port} is available again.")
                    return
            else:
                present_since = None
            await asyncio.sleep(0.2)

    async def _fetch_params_mavlink(self, drone_obj) -> dict[str, ParamValue] | None:
        """ Read all parameters with PARAM_REQUEST_LIST, independent of the MAVSDK parameter cache.

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

        async def on_param_value(msg):
            nonlocal param_count, last_received
            if msg.get_srcSystem() != target_system or msg.get_srcComponent() != target_component:
                return
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            param_id = param_id.rstrip("\x00")
            kind = float if msg.param_type in FLOAT_PARAM_TYPES else int
            received[param_id] = ParamValue(param_id, decode_param_value_msg(msg.param_value, msg.param_type,
                                                                             bytewise), kind)
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
