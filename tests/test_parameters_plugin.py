""" Tests for the parameters plugin, using a simulated flight controller instead of a drone."""
import argparse
import asyncio
import logging
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from dronemanager.app import add_cli_command_parser, parse_docstring_args
from dronemanager.plugins import parameters
from dronemanager.plugins.parameters import (ParametersPlugin, ParamValue, read_params_file, write_params_file,
                                             diff_params, values_equal, format_value, default_excludes,
                                             decode_param_value_msg, parse_param_value)


LOGGER = logging.getLogger("test_parameters")


# Simulated flight controller #########################################################################################

def _f32(value):
    return float(np.float32(value))


class FakeFC:
    """ The flight controller: RAM values, persistent storage and reboots. Survives reconnects."""

    def __init__(self, params, volatile=()):
        self.params = {name: ParamValue(name, _f32(v) if k is float else v, k) for name, (v, k) in params.items()}
        self.persistent = dict(self.params)
        self.volatile = set(volatile)  # Parameters that revert on reboot, i.e. rejected by the FC on boot
        self.after_reboot_only = {}  # Parameters that only exist after a reboot
        self.writes = []
        self.reboots = 0

    def set(self, name, value, kind):
        param = ParamValue(name, _f32(value) if kind is float else value, kind)
        self.params[name] = param
        self.writes.append(name)
        if name not in self.volatile:
            self.persistent[name] = param
        self._apply_limits()

    def _apply_limits(self):
        # Like PX4: "Manual speed has been constrained by max speed"
        max_speed = self.params.get("MPC_XY_VEL_MAX")
        manual_speed = self.params.get("MPC_VEL_MANUAL")
        if max_speed and manual_speed and manual_speed.value > max_speed.value:
            limited = ParamValue("MPC_VEL_MANUAL", max_speed.value, float)
            self.params["MPC_VEL_MANUAL"] = limited
            self.persistent["MPC_VEL_MANUAL"] = limited

    def reboot(self):
        self.reboots += 1
        self.params = dict(self.persistent)
        self.params.update(self.after_reboot_only)


class ParamError(Exception):
    pass


class FakeParamAPI:
    """ Stand-in for mavsdk.param.Param."""

    def __init__(self, fc: FakeFC):
        self.fc = fc

    async def get_all_params(self):
        def entries(kind):
            return [SimpleNamespace(name=p.name, value=p.value) for p in self.fc.params.values() if p.kind is kind]
        return SimpleNamespace(int_params=entries(int), float_params=entries(float), custom_params=[])

    async def _get(self, name, kind):
        param = self.fc.params.get(name)
        if param is None:
            raise ParamError("NotFound")
        if param.kind is not kind:
            raise ParamError("WrongType")
        return param.value

    async def _set(self, name, value, kind):
        param = self.fc.params.get(name)
        if param is None:
            raise ParamError("NotFound")
        if param.kind is not kind:
            raise ParamError("WrongType")
        self.fc.set(name, value, kind)

    async def get_param_int(self, name):
        return await self._get(name, int)

    async def get_param_float(self, name):
        return await self._get(name, float)

    async def get_param_custom(self, name):
        return await self._get(name, str)

    async def set_param_int(self, name, value):
        await self._set(name, value, int)

    async def set_param_float(self, name, value):
        await self._set(name, value, float)

    async def set_param_custom(self, name, value):
        await self._set(name, value, str)


class FakeMavConnection:
    """ Stand-in for MAVPassthrough, answering PARAM_REQUEST_LIST/READ like a PX4 (bytewise integer encoding)."""

    def __init__(self, fc: FakeFC, drop_indices=()):
        self.fc = fc
        self.drone_system = 1
        self.callbacks = {}
        self.drop_indices = set(drop_indices)  # Lost on the first PARAM_REQUEST_LIST, to test re-requests
        encoder = SimpleNamespace(param_request_list_encode=lambda sys, comp: ("list",),
                                  param_request_read_encode=lambda sys, comp, pid, idx: ("read", idx))
        self.con_drone_in = SimpleNamespace(mav=encoder)
        self.requests = []

    def add_drone_message_callback(self, message_id, func):
        self.callbacks.setdefault(message_id, set()).add(func)

    def remove_drone_message_callback(self, message_id, func):
        self.callbacks[message_id].remove(func)

    def _msg(self, index, param, count):
        if param.kind is int:
            value = struct.unpack("<f", struct.pack("<i", param.value))[0]
            param_type = 6
        else:
            value = param.value
            param_type = 9
        return SimpleNamespace(get_srcSystem=lambda: 1, get_srcComponent=lambda: 1, param_id=param.name,
                               param_value=value, param_type=param_type, param_index=index, param_count=count)

    def send_as_gcs(self, msg):
        self.requests.append(msg)
        params = sorted(self.fc.params.values(), key=lambda p: p.name)
        if msg[0] == "list":
            indices = [i for i in range(len(params)) if i not in self.drop_indices]
            self.drop_indices = set()
        else:
            indices = [msg[1]]
        for index in indices:
            message = self._msg(index, params[index], len(params))
            for callback in list(self.callbacks.get(22, ())):
                asyncio.get_running_loop().create_task(callback(message))


class FakeDrone:

    def __init__(self, fc: FakeFC, address="udp://192.168.1.31:14561", autopilot="PX4", raw_mavlink=True):
        self.fc = fc
        self.drone_addr = address
        self.autopilot = autopilot
        self.is_armed = False
        self.in_air = False
        self.is_connected = True
        self.drone_system_id = 1
        self.drone_component_id = 1
        self.drone_params = SimpleNamespace(raw={})
        self.config = SimpleNamespace(position_rate=5.0)
        self.mav_conn = FakeMavConnection(fc) if raw_mavlink else None
        self.system = SimpleNamespace(param=FakeParamAPI(fc), action=SimpleNamespace(reboot=self._reboot))
        self.link_drops_on_reboot = True

    async def _reboot(self):
        if self.drone_addr.startswith("serial"):
            self.fc.reboot()
            self.is_connected = False
            return
        if self.link_drops_on_reboot:
            self.is_connected = False

            async def come_back():
                await asyncio.sleep(0.3)
                self.fc.reboot()
                self.is_connected = True
            asyncio.get_running_loop().create_task(come_back())


class FakeDM:

    def __init__(self, fc: FakeFC):
        self.fc = fc
        self.drones = {}
        self.disconnects = []
        self.connects = []

    async def disconnect(self, names, force=False):
        self.disconnects.append((names, force))
        self.drones.pop(names, None)

    async def connect_to_drone(self, name, drone_address=None, timeout=30, telemetry_frequency=None,
                               log_telemetry=None, **kwargs):
        self.connects.append((name, drone_address, telemetry_frequency))
        self.drones[name] = FakeDrone(self.fc, drone_address)
        return True


PX4_PARAMS = {
    "MPC_XY_VEL_MAX": (12.0, float),
    "MPC_VEL_MANUAL": (5.0, float),
    "MIS_TAKEOFF_ALT": (2.5, float),
    "COM_RC_IN_MODE": (1, int),
    "SYS_AUTOSTART": (4001, int),
    "CAL_ACC0_ID": (1310988, int),
    "CAL_ACC0_XOFF": (0.0123, float),
    "NEG_INT": (-1, int),
}


def make_setup(tmp_path, address="udp://192.168.1.31:14561", raw_mavlink=True, volatile=()):
    fc = FakeFC(PX4_PARAMS, volatile=volatile)
    dm = FakeDM(fc)
    dm.drones["tom"] = FakeDrone(fc, address, raw_mavlink=raw_mavlink)
    plugin = ParametersPlugin(dm, LOGGER, "parameters", directory=str(tmp_path))
    plugin.LINK_DROP_TIMEOUT = 1.0
    plugin.SETTLE_TIME = 0.0
    plugin.RECONNECT_DELAY = 0.0
    plugin.PORT_STABLE_TIME = 0.3
    plugin.ADJUST_DELAY = 0.05
    plugin.FETCH_TIMEOUT = 3.0
    plugin.FETCH_QUIET_TIME = 0.2
    return fc, dm, plugin


def write_target(tmp_path, name="target", **changes):
    params = {name_: ParamValue(name_, v, k) for name_, (v, k) in PX4_PARAMS.items()}
    for param_name, value in changes.items():
        params[param_name] = ParamValue(param_name, value, params[param_name].kind)
    write_params_file(tmp_path / f"{name}.params", params.values())
    return f"{name}.params"


# File format and helper functions ###################################################################################

def test_params_file_round_trip(tmp_path):
    params = [ParamValue("A_INT", 42, int), ParamValue("B_FLOAT", _f32(0.1), float), ParamValue("C_NEG", -7, int),
              ParamValue("D_BIG", _f32(123456.789), float), ParamValue("E_CUSTOM", "abc", str)]
    count = write_params_file(tmp_path / "a.params", params, ["Header line"], system_id=3, component_id=1)
    assert count == 4  # Custom parameter skipped
    text = (tmp_path / "a.params").read_text()
    assert text.startswith("# Header line\n")
    assert "3\t1\tB_FLOAT\t0.1\t9" in text
    loaded = read_params_file(tmp_path / "a.params")
    assert set(loaded) == {"A_INT", "B_FLOAT", "C_NEG", "D_BIG"}
    for param in params[:4]:
        assert loaded[param.name].kind is param.kind
        assert values_equal(loaded[param.name].value, param.value)


def test_read_qgc_and_mission_planner_files(tmp_path):
    (tmp_path / "qgc.params").write_text(
        "# Onboard parameters for Vehicle 1\n#\n# Stack: PX4 Pro\n#\n"
        "# Vehicle-Id Component-Id Name Value Type\n"
        "1\t1\tMPC_XY_VEL_MAX\t12.000000000000000000\t9\n"
        "1\t1\tSYS_AUTOSTART\t4001\t6\n"
        "1\t1\tCOM_RC_IN_MODE\t1.0\t6\n\n")
    params = read_params_file(tmp_path / "qgc.params")
    assert params["MPC_XY_VEL_MAX"] == ParamValue("MPC_XY_VEL_MAX", 12.0, float)
    assert params["SYS_AUTOSTART"] == ParamValue("SYS_AUTOSTART", 4001, int)
    assert params["COM_RC_IN_MODE"] == ParamValue("COM_RC_IN_MODE", 1, int)

    (tmp_path / "mp.param").write_text("WPNAV_SPEED,500\nATC_RAT_RLL_P,0.135\n")
    params = read_params_file(tmp_path / "mp.param")
    assert params["WPNAV_SPEED"] == ParamValue("WPNAV_SPEED", 500, None)
    assert params["ATC_RAT_RLL_P"] == ParamValue("ATC_RAT_RLL_P", 0.135, None)


def test_read_invalid_file_reports_line(tmp_path):
    (tmp_path / "bad.params").write_text("# comment\n1\t1\tA\t1\t6\n1\t1\tB\t1.5\t6\n")
    with pytest.raises(ValueError, match="line 3"):
        read_params_file(tmp_path / "bad.params")


def test_values_and_formatting():
    assert values_equal(0.1, _f32(0.1))
    assert not values_equal(0.1, 0.1001)
    assert values_equal(3, 3)
    assert not values_equal(3, 4)
    assert values_equal(float("nan"), float("nan"))
    assert format_value(_f32(0.1), float) == "0.1"
    assert format_value(1.0, float) == "1.0"
    assert format_value(5, int) == "5"
    assert parse_param_value("3.0", int) == 3
    with pytest.raises(ValueError):
        parse_param_value("3.5", int)


def test_diff_and_excludes():
    current = {"A": ParamValue("A", 1, int), "B": ParamValue("B", 2.0, float), "CAL_X": ParamValue("CAL_X", 5, int)}
    target = {"A": ParamValue("A", 1, int), "B": ParamValue("B", 3, None), "CAL_X": ParamValue("CAL_X", 6, int),
              "MISSING": ParamValue("MISSING", 1, int)}
    diff = diff_params(target, current, default_excludes("PX4"))
    assert diff.unchanged == ["A"]
    assert [(new.name, new.value, new.kind) for new, old in diff.to_change] == [("B", 3.0, float)]
    assert diff.excluded == ["CAL_X"]
    assert diff.missing_on_drone == ["MISSING"]
    assert "COMPASS_OFS_X" not in default_excludes("PX4")
    assert "CAL_*" in default_excludes(None) and "STAT_*" in default_excludes(None)


def test_decode_bytewise_and_cast():
    as_float = struct.unpack("<f", struct.pack("<i", 4001))[0]
    assert decode_param_value_msg(as_float, 6, bytewise=True) == 4001
    assert decode_param_value_msg(4001.0, 6, bytewise=False) == 4001
    assert decode_param_value_msg(0.5, 9, bytewise=True) == 0.5


# CLI parser generation ###############################################################################################

def test_parse_docstring_args():
    doc = ("Summary line.\n\nLonger text.\n\nArgs:\n    drone: Name of the drone.\n    filename (str): A file\n"
           "      spanning two lines.\n\nReturns:\n    Something.")
    assert parse_docstring_args(doc) == {"drone": "Name of the drone.", "filename": "A file spanning two lines."}


def test_cli_parser_for_plugin_commands(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command")
    for command_name, command in plugin.cli_commands.items():
        add_cli_command_parser(subparsers, f"{plugin.PREFIX}-{command_name}", command)

    args = vars(parser.parse_args(["param-apply", "tom", "base", "--verify"]))
    assert args == {"command": "param-apply", "drone": "tom", "filename": "base", "verify": True,
                    "include_calibration": False, "reboot_timeout": 90.0}
    args = vars(parser.parse_args(["param-save", "tom"]))
    assert args == {"command": "param-save", "drone": "tom", "filename": None, "force": False}

    apply_help = " ".join(subparsers.choices["param-apply"].format_help().split())
    assert "Reboot the drone after applying" in apply_help
    assert "Only parameters that differ from the drone are written." in apply_help
    assert "Default: 90.0" in apply_help
    assert "Name of the parameter" in subparsers.choices["param-set"].format_help()


# Plugin commands #####################################################################################################

def test_save_writes_all_params(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    path = asyncio.run(plugin.save("tom", "backup"))
    assert path == tmp_path / "backup.params"
    saved = read_params_file(path)
    assert set(saved) == set(PX4_PARAMS)
    assert saved["NEG_INT"].value == -1
    # No overwriting without force
    assert asyncio.run(plugin.save("tom", "backup")) is False
    assert asyncio.run(plugin.save("tom", "backup", force=True)) == path
    assert asyncio.run(plugin.save("nobody")) is False


def test_apply_writes_only_changes_and_skips_calibration(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0, CAL_ACC0_XOFF=0.5)
    assert asyncio.run(plugin.apply("tom", filename)) is True
    assert fc.writes == ["MPC_XY_VEL_MAX"]
    assert fc.params["MPC_XY_VEL_MAX"].value == 8.0
    assert fc.params["CAL_ACC0_XOFF"].value == _f32(0.0123)
    assert dm.drones["tom"].drone_params.raw["MPC_XY_VEL_MAX"] == (8.0, float)

    assert asyncio.run(plugin.apply("tom", filename, include_calibration=True)) is True
    assert fc.params["CAL_ACC0_XOFF"].value == 0.5


def test_apply_refused_when_armed(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0)
    dm.drones["tom"].is_armed = True
    assert asyncio.run(plugin.apply("tom", filename)) is False
    assert asyncio.run(plugin.set_parameter("tom", "MPC_XY_VEL_MAX", "5")) is False
    assert fc.writes == []


def test_diff_does_not_write(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0)
    diff = asyncio.run(plugin.diff("tom", filename))
    assert [new.name for new, old in diff.to_change] == ["MPC_XY_VEL_MAX"]
    assert fc.writes == []


def test_set_and_get(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    assert asyncio.run(plugin.set_parameter("tom", "mis_takeoff_alt", "3.5")) is True
    assert fc.params["MIS_TAKEOFF_ALT"].value == 3.5
    assert asyncio.run(plugin.get_parameter("tom", "MIS_TAKEOFF_ALT")) == 3.5
    assert asyncio.run(plugin.set_parameter("tom", "COM_RC_IN_MODE", "2")) is True
    assert fc.params["COM_RC_IN_MODE"] == ParamValue("COM_RC_IN_MODE", 2, int)
    # Wrong type and unknown names are rejected without writing
    assert asyncio.run(plugin.set_parameter("tom", "COM_RC_IN_MODE", "1.5")) is False
    assert asyncio.run(plugin.set_parameter("tom", "NOT_A_PARAM", "1")) is False
    assert asyncio.run(plugin.get_parameter("tom", "NOT_A_PARAM")) is None
    assert fc.writes == ["MIS_TAKEOFF_ALT", "COM_RC_IN_MODE"]


def test_list_files(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    write_target(tmp_path, "one")
    write_target(tmp_path, "two")
    (tmp_path / "notes.txt").write_text("not a parameter file")
    assert [path.name for path in asyncio.run(plugin.list_files())] == ["one.params", "two.params"]


# Reboot verification #################################################################################################

def test_verify_udp_keeps_connection(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    original_drone = dm.drones["tom"]
    original_drone.mav_conn.drop_indices = {2}  # One parameter is lost and has to be requested again
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0, SYS_AUTOSTART=4002)
    assert asyncio.run(plugin.apply("tom", filename, verify=True, reboot_timeout=5)) is True
    assert fc.reboots == 1
    assert dm.disconnects == [] and dm.connects == []
    assert dm.drones["tom"] is original_drone
    assert ("read", 2) in original_drone.mav_conn.requests


def test_verify_udp_detects_reverted_params(tmp_path):
    fc, dm, plugin = make_setup(tmp_path, volatile={"MPC_XY_VEL_MAX"})
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0)
    assert asyncio.run(plugin.apply("tom", filename, verify=True, reboot_timeout=5)) is False
    assert fc.reboots == 1
    assert fc.params["MPC_XY_VEL_MAX"].value == 12.0


def test_verify_udp_fails_without_link_drop(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    dm.drones["tom"].link_drops_on_reboot = False
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0)
    assert asyncio.run(plugin.apply("tom", filename, verify=True, reboot_timeout=5)) is False


def test_verify_serial_reconnects(tmp_path, monkeypatch):
    # The port is still there briefly, disappears during the reboot, is used by the bootloader for a moment and
    # comes back for good
    port_states = iter([True, True, False, False, True, False, False])
    checked = []

    def port_present(port):
        checked.append(port)
        return next(port_states, True)
    monkeypatch.setattr(parameters, "_serial_port_present", port_present)
    fc, dm, plugin = make_setup(tmp_path, address="serial://COM5:57600")
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0)
    assert asyncio.run(plugin.apply("tom", filename, verify=True, reboot_timeout=10)) is True
    assert fc.reboots == 1
    assert dm.disconnects == [("tom", True)]
    assert dm.connects == [("tom", "serial://COM5:57600", 5.0)]
    assert len(checked) >= 9 and set(checked) == {"COM5"}


def test_verify_serial_detects_reverted_params_with_mavsdk_fallback(tmp_path, monkeypatch):
    monkeypatch.setattr(parameters, "_serial_port_present", lambda port: None)
    fc, dm, plugin = make_setup(tmp_path, address="serial://COM5:57600", raw_mavlink=False,
                                volatile={"SYS_AUTOSTART"})
    filename = write_target(tmp_path, SYS_AUTOSTART=4002)
    assert asyncio.run(plugin.apply("tom", filename, verify=True, reboot_timeout=5)) is False


def test_verify_reports_params_that_appear_after_reboot(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    fc.after_reboot_only["NEW_DRIVER_PARAM"] = ParamValue("NEW_DRIVER_PARAM", 0, int)
    params = {name: ParamValue(name, v, k) for name, (v, k) in PX4_PARAMS.items()}
    params["NEW_DRIVER_PARAM"] = ParamValue("NEW_DRIVER_PARAM", 1, int)
    write_params_file(tmp_path / "with_new.params", params.values())
    # The new parameter couldn't be applied before the reboot, which is a warning, not a failure
    assert asyncio.run(plugin.apply("tom", "with_new", verify=True, reboot_timeout=5)) is True


def test_lookup_uses_cached_type(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    drone = dm.drones["tom"]
    drone.drone_params.raw = {name: (v, k) for name, (v, k) in PX4_PARAMS.items()}
    calls = []
    for kind in ("int", "float", "custom"):
        original = getattr(drone.system.param, f"get_param_{kind}")

        async def wrapped(name, _original=original, _kind=kind):
            calls.append(_kind)
            return await _original(name)
        setattr(drone.system.param, f"get_param_{kind}", wrapped)
    assert asyncio.run(plugin.get_parameter("tom", "MIS_TAKEOFF_ALT")) == 2.5
    assert calls == ["float"]
    # Parameters missing from the cache, i.e. ones that appeared later, are found after refreshing the cache
    fc.params["NEW_PARAM"] = ParamValue("NEW_PARAM", 0.5, float)
    calls.clear()
    assert asyncio.run(plugin.get_parameter("tom", "NEW_PARAM")) == 0.5
    assert calls == ["float"]
    # Unknown names are rejected without trying to read them
    calls.clear()
    assert asyncio.run(plugin.get_parameter("tom", "NOT_A_PARAM")) is None
    assert calls == []


# Parameters limited by other parameters ##############################################################################

def test_apply_rewrites_params_limited_by_others(tmp_path):
    """ The scenario from the usage guide: lowering MPC_XY_VEL_MAX also lowers MPC_VEL_MANUAL. Applying the baseline
    writes MPC_VEL_MANUAL first, which the FC limits again until MPC_XY_VEL_MAX is raised."""
    fc, dm, plugin = make_setup(tmp_path)
    fc.set("MPC_VEL_MANUAL", 10.0, float)
    assert asyncio.run(plugin.save("tom", "baseline")) == tmp_path / "baseline.params"
    assert asyncio.run(plugin.set_parameter("tom", "MPC_XY_VEL_MAX", "8")) is True
    assert fc.params["MPC_VEL_MANUAL"].value == 8.0
    fc.writes.clear()
    assert asyncio.run(plugin.apply("tom", "baseline", verify=True, reboot_timeout=5)) is True
    assert fc.params["MPC_XY_VEL_MAX"].value == 12.0
    assert fc.params["MPC_VEL_MANUAL"].value == 10.0
    assert fc.writes == ["MPC_VEL_MANUAL", "MPC_XY_VEL_MAX", "MPC_VEL_MANUAL"]


def test_apply_reports_conflicting_values(tmp_path):
    fc, dm, plugin = make_setup(tmp_path)
    filename = write_target(tmp_path, MPC_XY_VEL_MAX=8.0, MPC_VEL_MANUAL=10.0)
    assert asyncio.run(plugin.apply("tom", filename)) is False
    assert fc.writes.count("MPC_VEL_MANUAL") == plugin.APPLY_PASSES
    assert fc.params["MPC_VEL_MANUAL"].value == 8.0


def test_set_reports_side_effects(tmp_path, caplog):
    fc, dm, plugin = make_setup(tmp_path)
    with caplog.at_level(logging.INFO):
        assert asyncio.run(plugin.set_parameter("tom", "MPC_XY_VEL_MAX", "4")) is True
    assert "MPC_VEL_MANUAL 5.0 -> 4.0" in caplog.text
    assert dm.drones["tom"].drone_params.raw["MPC_VEL_MANUAL"] == (4.0, float)
