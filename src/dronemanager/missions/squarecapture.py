"""DroneManager mission: fly a square, stop at the corners, sweep the heading,
capture an image/LiDAR pair from the drone rig at every stop.

Flies the campaign's offline reconstruction dataset — by default a 10 m square
at 10 m altitude, 4 corner stations, 8 headings per station in 45 deg steps,
32 posed image/LiDAR pairs. Fully autonomous: arm, takeoff, sweep, return, land.

    mission-load squarecapture --name sq
    sq-add Romulus
    sq-plan                       # print the schedule without flying
    sq-check                      # preflight only: link, sensors, GPS, battery
    sq-dryrun flight01            # the whole capture loop, no flight commands
    sq-run flight01               # the real thing
    sq-abort                      # stop and land where we are

DIVISION OF LABOUR. This file only sequences flight and capture. The geometry
lives in `squarecapture_plan.py` (pure stdlib, unit-tested in
`tests/test_squarecapture_plan.py`); the capture itself — sensor grab, per-beam
averaging, depth projection, FC-pose stamping, tlog — belongs entirely to the
daemon on the Jetson, reached through `drone_capture_client.Link`. Nothing here
touches MAVLink directly: the FC link is DroneManager's, and the daemon's
separate receive-only tap is the daemon's.

POSE. The daemon stamps each capture with FC pose and labels it
`trust: "prior_only"`, which is the truth: splat-grade work needs attitude to
0.0092 deg and no flight controller is close. These captures are a registration
SEED, not an answer. The payload also hangs 25 cm below the FC — see
`squarecapture_plan.PAYLOAD_OFFSET_FC_M`, which the consumer must apply.
(Measured on the airframe 2026-09-04: the lidar origin is 28 cm below the FC and
the main GPS antenna 15 cm ahead of and 8 cm above it. `Scripts/flight_poses.py`
carries the full chain.)

⚠ SCANS: CONSIDER `--scans 1` IN FLIGHT. The default 10 averages ten sweeps over
0.9 s to cut range noise, which is right on a tripod and was validated there at
4.6 mm of per-beam spread. Measured on `flight03`, in the air, the same
statistic is 114 mm median (49-251) — 25x the bench — because the aircraft
drifts 0.125 m median across that window (correlation 0.60) and yaws more than
1 deg inside it on 11 of 32 captures. The sweeps being averaged are not taken
from one place. A single sweep carries roughly 15 mm of range noise, so in
flight one sweep is the better trade until deskewing exists. The default is left
at 10 because changing what a sortie captures is an operator decision, not a
refactor.

SAFETY. The RC transmitter is the real abort: a mode switch takes the drone out
of offboard immediately and nothing here can override that. If this mission's
task dies for any reason the drone keeps hovering on MAVSDK's last streamed
setpoint — it does not fall and it does not fly away — so `sq-abort` cancels the
sweep and then commands a landing, rather than relying on the cancellation
itself to be safe.
"""

import asyncio
import enum
import json
import math
import os
import sys
import time

from dronemanager.plugins.mission import Mission, MissionStage, FlightArea      # noqa: E402
from dronemanager.navigation.core import Waypoint, WayPointType                 # noqa: E402
from dronemanager.navigation.rectlocalfence import RectLocalFence               # noqa: E402

# This mission has to import two siblings from whichever directory it happens to
# live in, and there are two such directories with DIFFERENT import semantics.
#
#  * SHIPPED, in `dronemanager/missions/`: loaded as a real package module, so a
#    relative import works and sys.path must be left alone.
#  * USER, in `Documents/DroneManager/missions/`: loaded by file path under a
#    parent package that does not exist, so the relative import fails and the
#    directory is not importable at all until it is put on sys.path.
#
# Try the package import first. It is the case that must NOT touch sys.path:
# appending `dronemanager/missions/`'s parent would make `core`, `drone`,
# `utils` and friends importable as top-level modules for the whole app.
try:                                                                            # noqa: E402
    from . import squarecapture_plan as plan
    from .drone_capture_client import Link
except ImportError:                                                             # noqa: E402
    # APPEND rather than insert: this permanently alters sys.path for the whole
    # DroneManager process, and a directory at the FRONT can shadow a real
    # package for every other plugin in the app.
    _HERE = os.path.dirname(os.path.abspath(__file__))
    for _p in (_HERE, os.path.dirname(_HERE)):
        if _p not in sys.path:
            sys.path.append(_p)
    import squarecapture_plan as plan
    from drone_capture_client import Link


DEFAULT_HOST = "192.168.1.55"
DEFAULT_USER = "dronetrekkers"
DEFAULT_PORT = 5757

# Where the mission keeps its OWN files on the ground station: the marks the
# operator records with the drone, and one JSON per sortie with everything the
# daemon never sees (the target, the plan, the commanded poses). This is the
# directory DroneManager itself creates, so it survives re-copying the mission
# files and reinstalling the package. ⚠ A OneDrive-redirected Documents folder
# is the one way these can "disappear" — every save logs its absolute path.
USER_DIR = os.path.join(os.path.expanduser("~"), "Documents", "DroneManager")
MARKS_FILE = "sq_marks.json"
SESSIONS_SUBDIR = "sq_sessions"
# A mark stores BOTH the GNSS position and the EKF-local one. The offset is
# always computed from GNSS (frame-independent across a reboot); the local
# delta is a cross-check, and above this disagreement the operator is told the
# EKF origin moved between marking and now.
MARK_MAX_NED_DISAGREE_M = 1.0

# The sortie the pattern study chose (eval/Flight/pattern-study.md): an aimed
# orbit at radius == altitude — the fixed 45 deg mount centres the target only
# then — 8 stations, a 15 deg fan (17.5 is the wrap-safe ceiling on an
# 8-ring), one scan per capture. Baked into ONE command so the field does not
# depend on ten flags being typed right; flight02 was lost to a default nobody
# typed.
ORBIT_PRESET = dict(pattern="orbit", anchor="centre", aim="yes",
                    radius=7.0, altitude=7.0, stations=8, fan=15.0,
                    headings=8, side=10.0, ring2_radius=12.0,
                    ring2_altitude=12.0, ring2_stations=6)

CALL_TIMEOUT = 15.0        # daemon's own shot_timeout is 3 s; this is the wire
FLY_TIMEOUT = 120.0        # a 10 m leg at a few m/s, with generous slack
YAW_TIMEOUT = 45.0         # 45 deg at 30 deg/s is 1.5 s
TAKEOFF_TIMEOUT = 120.0
LAND_TIMEOUT = 180.0
SYNC_EVERY_S = 30.0        # the Jetson has no RTC; re-anchor its clock in flight


class CaptureAbort(RuntimeError):
    """The sortie is not worth continuing: every capture is failing the same
    way. Raised inside the sweep, caught by `_sortie`, which LANDS."""


class SquareStage(MissionStage):
    Idle = enum.auto()
    Preflight = enum.auto()
    Takeoff = enum.auto()
    Transit = enum.auto()
    Sweep = enum.auto()
    Return = enum.auto()
    Landing = enum.auto()
    Done = enum.auto()
    Aborted = enum.auto()


class SquareFlightArea(FlightArea):
    """The fence box, republished in the FlightArea shape other components read.
    x is north, y is east, z is down — so z_min is the CEILING."""

    def __init__(self, bounds):
        super().__init__()
        self.bounds = tuple(float(b) for b in bounds)
        (self._x_min, self._x_max, self._y_min, self._y_max,
         self._z_min, self._z_max) = self.bounds

    x_min = property(lambda self: self._x_min)
    x_max = property(lambda self: self._x_max)
    y_min = property(lambda self: self._y_min)
    y_max = property(lambda self: self._y_max)
    z_min = property(lambda self: self._z_min)
    z_max = property(lambda self: self._z_max)


# --------------------------------------------------------------------------- #
#  The capture daemon, from the event loop
# --------------------------------------------------------------------------- #

class CaptureLink:
    """`drone_capture_client.Link` driven from asyncio.

    `Link` is deliberately blocking and stdlib-only so it runs on the Windows
    ground station with a bare `python`. That makes it a hazard here: a blocking
    `recv` on the mission's event loop would also stall the setpoint stream that
    keeps the drone in offboard, and PX4 drops out of offboard after ~0.5 s
    without setpoints. Every call therefore goes through the default executor,
    with a timeout the daemon's own 3 s `shot_timeout` fits inside.

    One SSH pipe is held open for the whole flight — 32 fresh `ssh -W` handshakes
    would be 32 extra seconds of hover and 32 more things to fail — and it
    reconnects once, transparently, if the pipe breaks mid-sortie.
    """

    def __init__(self, logger, host=DEFAULT_HOST, user=DEFAULT_USER,
                 port=DEFAULT_PORT, direct=False, token=None):
        self.logger = logger
        self._kw = dict(host=host, user=user, port=port, direct=direct,
                        token=token)
        self._link = None
        self._last_sync = 0.0
        self.reconnects = 0

    # -- plumbing -------------------------------------------------------- #

    def _open(self):
        link = Link(**self._kw)
        link.__enter__()
        return link

    def _blocking_call(self, req):
        if self._link is None:
            self._link = self._open()
        try:
            return self._link.call(req)
        except Exception:                                          # noqa: BLE001
            # The pipe is gone. Tear it down and try exactly once more: a
            # transient Wi-Fi drop should not end a flight, but a retry loop
            # over a dead link would hold the drone hovering indefinitely.
            self._close_blocking()
            self._link = self._open()
            self.reconnects += 1
            return self._link.call(req)

    def _close_blocking(self):
        link, self._link = self._link, None
        if link is not None:
            try:
                link.__exit__(None, None, None)
            except Exception:                                      # noqa: BLE001
                pass

    async def call(self, req, timeout=CALL_TIMEOUT):
        loop = asyncio.get_running_loop()
        try:
            return await asyncio.wait_for(
                loop.run_in_executor(None, self._blocking_call, req), timeout)
        except asyncio.TimeoutError:
            # The executor thread is still parked in a blocking read. Closing
            # the pipe from here is what releases it: terminating ssh makes the
            # read return empty and the thread unwinds on its own.
            self.logger.warning("capture daemon timed out on %r", req.get("cmd"))
            await loop.run_in_executor(None, self._close_blocking)
            return {"ok": False, "reason": "timeout", "detail": req.get("cmd")}
        except Exception as exc:                                   # noqa: BLE001
            self.logger.warning("capture daemon call failed: %r", exc)
            return {"ok": False, "reason": "link_error", "detail": repr(exc)}

    async def close(self):
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._close_blocking)

    # -- protocol -------------------------------------------------------- #

    async def sync_if_due(self, force=False):
        """Re-anchor the Jetson's clock to the ground station's.

        The Jetson has no RTC battery, so its own UTC is wrong by days; the
        session records the GCS anchor and every resync. Without this the
        capture timestamps cannot be lined up against the tlog."""
        now = time.time()
        if force or now - self._last_sync > SYNC_EVERY_S:
            self._last_sync = now
            return await self.call({"cmd": "sync", "gcs_utc_ns": time.time_ns()})
        return None

    async def start_session(self, name, scans, exposure_ms, gain_db, gate,
                            require_pose, require_fix):
        return await self.call({
            "cmd": "start", "name": name, "scans": scans,
            "exposure_ms": exposure_ms, "gain_db": gain_db,
            "average": True, "gate": gate,
            "require_pose": require_pose, "require_fix": require_fix,
            "gcs_utc_ns": time.time_ns(),
            "gcs_utc_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }, timeout=30.0)

    async def meter(self, gain_db, target_sat=0.5):
        """Ask the daemon to meter the exposure on the real picture."""
        return await self.call({"cmd": "meter", "gain_db": gain_db,
                                "target_sat": target_sat}, timeout=90.0)

    async def shot(self, tag):
        return await self.call({"cmd": "shot", "tag": tag})

    async def rec_start(self):
        """Start the continuous recording (every sweep, frame and IMU sample)
        into the open session. The daemon refuses without a session."""
        return await self.call({"cmd": "rec_start"}, timeout=30.0)

    async def rec_stop(self):
        """Stop it and get the summary (sweeps, frames, imu rows, drops)."""
        return await self.call({"cmd": "rec_stop"}, timeout=30.0)

    async def stop_session(self):
        return await self.call({"cmd": "stop"}, timeout=30.0)

    async def status(self):
        return await self.call({"cmd": "status"})


# --------------------------------------------------------------------------- #
#  The mission
# --------------------------------------------------------------------------- #

class SquareCaptureMission(Mission):
    """Square flight pattern with a yaw sweep and a rig capture at every stop."""

    def __init__(self, dm, logger, name="square"):
        super().__init__(dm, logger, name)
        self.cli_commands.update({
            "plan": self.show_plan,
            "check": self.preflight,
            "run": self.run,
            "dryrun": self.dryrun,
            "abort": self.abort,
            "link": self.set_link,
            "mark": self.mark,
            "marks": self.list_marks,
            "unmark": self.unmark,
            "orbit": self.orbit,
            "orbit-plan": self.orbit_plan,
        })
        self.current_stage = SquareStage.Idle

        self.side_m = 10.0
        self.altitude_m = 10.0
        # True once a TARGET-CENTRED pattern is planned; it widens the geofence
        # from the square formula to the stations' own bounding box.
        self._centred = False
        self.settle_s = 2.0
        self.retry_delay_s = 1.0
        # ⚠ The guard flight04 did not have: it flew all 8 stations, retried
        # all 64 captures and landed after 11 minutes with 64 x `no_fix` and
        # nothing on disk — the THIRD sortie lost to "every capture fails the
        # same way and nothing acts on it" (flight01 gain 0, flight02 98.5%
        # saturated). Three consecutive refusals sharing one reason, each
        # already retried once, end the sortie and land.
        self.max_consecutive_misses = 3
        # Seconds the aircraft sits STILL, recording, before arming. The gyro
        # bias random walk is unmeasurable without a stationary segment
        # (flight03 had 2 s of 213, both after landing). Tests set it to 0.
        self.prearm_still_s = 30.0
        self._recording = None      # what the daemon said when asked to record
        self.yaw_rate = 30.0
        self.position_tolerance = 0.3
        self.yaw_tolerance = 2.0

        self._host, self._user, self._port = DEFAULT_HOST, DEFAULT_USER, DEFAULT_PORT
        self._direct, self._token = False, None

        self._flight_task = None
        self._link = None
        self._results = []
        self._origin = None

    # -- Mission interface ----------------------------------------------- #

    async def add_drones(self, names: list[str]):
        """Add drones to the mission."""
        for name in names:
            drone = self.dm.drones.get(name)
            if drone is None:
                self.logger.warning("No drone named %s is connected.", name)
                continue
            if self.drones:
                self.logger.warning(
                    "%s already has %s; this mission flies exactly one drone.",
                    self.name, list(self.drones))
                return
            self.drones[name] = drone
            self.logger.info("Added %s to mission %s.", name, self.name)

    async def remove_drones(self, names: list[str]):
        """Remove drones from the mission."""
        if self._running():
            self.logger.warning("Mission is flying; %s-abort first.", self.name)
            return
        for name in names:
            if self.drones.pop(name, None) is not None:
                self.logger.info("Removed %s from mission %s.", name, self.name)

    async def mission_ready(self, drone: str):
        """Check that a drone is still connected and usable."""
        d = self.drones.get(drone)
        return bool(d is not None and d.is_connected)

    async def reset(self):
        """Return the mission to Idle. Does not move the drone."""
        if self._running():
            self.logger.warning("Mission is flying; %s-abort first.", self.name)
            return
        await self._close_link()
        self._results, self._origin = [], None
        self.current_stage = SquareStage.Idle
        self.additional_info = {}
        self.logger.info("Mission %s reset.", self.name)

    async def status(self):
        """Report mission progress."""
        done = sum(1 for r in self._results if r.get("ok"))
        self.logger.info(
            "%s: stage=%s drone=%s captures=%d/%d ok origin=%s",
            self.name, self.current_stage.name if self.current_stage else None,
            list(self.drones), done, len(self._results),
            None if self._origin is None else
            "N%.2f E%.2f D%.2f" % tuple(self._origin))
        if self._link is not None:
            self.logger.info("  daemon %s@%s:%d reconnects=%d",
                             self._user, self._host, self._port,
                             self._link.reconnects)
        for r in self._results:
            if not r.get("ok"):
                self.logger.info("  MISSED %s: %s %s", r.get("tag"),
                                 r.get("reason"), r.get("detail") or "")

    # -- CLI ------------------------------------------------------------- #

    async def set_link(self, host: str, user: str = DEFAULT_USER,
                       port: int = DEFAULT_PORT, direct: str = "no"):
        """Point the mission at a different capture daemon.

        `--direct yes` uses a plain TCP connection instead of `ssh -W`, which is
        how the SITL rehearsal reaches `mock_capture_daemon.py`. It is not a
        field option: the real daemon binds loopback precisely so that nothing
        is exposed on the shared Wi-Fi.
        """
        if self._running():
            self.logger.warning("Cannot change the link mid-flight.")
            return
        await self._close_link()
        self._host, self._user, self._port = host, user, int(port)
        self._direct = str(direct).lower() in ("yes", "true", "1")
        self.logger.info("Capture daemon set to %s%s:%d%s",
                         "" if self._direct else user + "@", host, self._port,
                         " (direct TCP)" if self._direct else "")

    @staticmethod
    def _yes(v):
        """CLI booleans arrive as strings: DroneManager builds the command line
        from the type hints and only str/float/int/list[str] are allowed
        (test_cli_command_parameters_are_all_annotated). Same idiom as
        set_link's `direct`."""
        return str(v).strip().lower() in ("yes", "true", "1", "on")

    def _build_pattern(self, pattern, anchor, side, altitude, radius, stations,
                       ring2_radius, ring2_altitude, ring2_stations, aim, fan,
                       headings, target_n, target_e):
        """-> (stations_ned | None, headings_fn | None). ValueError to refuse.

        A one-to-one mirror of `unity_square_check.expected_poses`, so the
        pattern that verified in Unity is the pattern that gets flown. Two
        things here are easy to get wrong and both are silent:

        * ANCHOR. `corner` is the original take-off-anchored square — the
          drone stands at the south-west corner and the square grows north and
          east. That is what flight01-03 flew, so it stays the default and its
          plan is byte-identical to before. Every other pattern is built
          AROUND the target and needs `anchor=centre`.
        * THE TARGET OFFSET. `capture_plan` hands `headings_fn` a station
          relative to the TAKE-OFF POINT, but the bearing must be to the
          TARGET. Without subtracting the offset every station aims at the
          spot the drone lifted off from, which on a 10 m offset is a 90 deg
          error at the near stations and looks like a heading-convention bug.
          You cannot take off under the truck, so the offset is the normal
          case, not the exotic one.
        """
        if anchor == "corner":
            if pattern != "square":
                raise ValueError(
                    "anchor=corner only describes the original take-off-"
                    "anchored square; pattern=%s must be flown with "
                    "anchor=centre and a target offset" % (pattern,))
            if target_n or target_e:
                raise ValueError("anchor=corner has no target to offset from")
            return None, None
        if anchor != "centre":
            raise ValueError("anchor must be 'corner' or 'centre', got %r"
                             % (anchor,))

        if math.hypot(float(target_n), float(target_e)) < 1.0:
            # An orbit never flies OVER its target and the return leg lands at
            # the take-off point regardless, so this is a warning, not a
            # refusal — it can only mean a rehearsal over empty grass.
            self.logger.warning(
                "The target is (essentially) the take-off point: the ring is "
                "centred on the landing spot. Right for a rehearsal over "
                "empty grass, wrong for anything you cannot take off under.")
        rel = [(n + target_n, e + target_e, a) for n, e, a in
               plan.pattern_stations(
                   pattern, side_m=side, altitude_m=altitude, radius_m=radius,
                   n_stations=stations, ring2_radius_m=ring2_radius,
                   ring2_altitude_m=ring2_altitude,
                   ring2_stations=ring2_stations)]

        if not self._yes(aim):
            if headings != len(plan.HEADINGS_8):
                raise ValueError("headings=%d without aim=yes: the fixed sweep "
                                 "is %d headings"
                                 % (headings, len(plan.HEADINGS_8)))
            return rel, None

        about_target = [(n - target_n, e - target_e) for n, e, _a in rel]
        lim = plan.max_safe_fan_deg(about_target)
        if fan > lim:
            raise ValueError(
                "fan=%.1f deg exceeds the wrap-safe limit %.1f for this "
                "pattern — a heading at +/-180 hangs is_at_heading forever. "
                "Narrow the fan or re-phase the ring." % (fan, lim))

        def headings_fn(st):
            return plan.headings_toward((st[0] - target_n, st[1] - target_e),
                                        headings, fan)
        return rel, headings_fn

    async def show_plan(self, side: float = 10.0, altitude: float = 10.0,
                        pattern: str = "square", anchor: str = "corner",
                        radius: float = 7.0, stations: int = 8,
                        aim: str = "no", fan: float = 15.0,
                        headings: int = 8, ring2_radius: float = 12.0,
                        ring2_altitude: float = 12.0,
                        ring2_stations: int = 6, target_n: float = 0.0,
                        target_e: float = 0.0, target_mark: str = ""):
        """Print the station/heading schedule without flying it."""
        origin = self._read_origin() or (0.0, 0.0, 0.0)
        try:
            target_n, target_e, tinfo = self._resolve_target(
                target_mark, target_n, target_e)
            stations_ned, headings_fn = self._build_pattern(
                pattern, anchor, side, altitude, radius, stations,
                ring2_radius, ring2_altitude, ring2_stations, aim, fan,
                headings, target_n, target_e)
            sts = plan.capture_plan(side_m=side, altitude_m=altitude,
                                    origin_ned=origin,
                                    initial_yaw=self._read_yaw() or 0.0,
                                    stations_ned=stations_ned,
                                    headings_fn=headings_fn)
        except ValueError as exc:
            self.logger.error("Cannot plan: %s", exc)
            return
        for line in plan.plan_summary(sts):
            self.logger.info(line)
        if stations_ned is not None and self._yes(aim):
            self.logger.info("aim fan %.1f deg (limit %.1f)", fan,
                             plan.max_safe_fan_deg(
                                 [(n - target_n, e - target_e)
                                  for n, e, _a in stations_ned]))
        bounds = plan.fence_bounds(origin, side, altitude,
                                   stations=sts if stations_ned else None)
        self.logger.info("fence %s", ("N[%.1f,%.1f] E[%.1f,%.1f] D[%.1f,%.1f]"
                                      % bounds))
        if stations_ned is not None:
            # The card: where the OBJECT is, in words checkable on a field.
            target = (origin[0] + target_n, origin[1] + target_e, origin[2])
            for line in plan.placement_card(
                    origin, target, sts, bounds,
                    target_global=tinfo.get("global"),
                    marks=tinfo.get("marks", ()),
                    altitude_delta_m=tinfo.get("amsl_delta")):
                self.logger.info(line)

    async def preflight(self):
        """Check the drone, the capture daemon and the sensors without flying."""
        ok = await self._preflight(require_flight=True)
        self.logger.info("Preflight %s.", "PASSED" if ok else "FAILED")
        return ok

    async def run(self, session: str, side: float = 10.0, altitude: float = 10.0,
                  scans: int = 10, settle: float = 2.0,
                  exposure: float | None = None,
                  gain: float = 1.0, gate: str = "warn",
                  pattern: str = "square", anchor: str = "corner",
                  radius: float = 7.0, stations: int = 8, aim: str = "no",
                  fan: float = 15.0, headings: int = 8,
                  ring2_radius: float = 12.0, ring2_altitude: float = 12.0,
                  ring2_stations: int = 6, target_n: float = 0.0,
                  target_e: float = 0.0, target_mark: str = "",
                  record: str = "yes", prearm: float = -1.0):
        """Fly the square and capture the dataset. Fully autonomous.

        gain is the ZED X's ANALOG gain in dB (1.0..7.0 on this AR0234;
        1.0 = least noise, which daylight can afford). The D455-era "unity
        gain 64" rule is gone with the sensor: on the ZED white is 255 at
        every gain, so the clip test always sees an overexposure.

        exposure (ms) defaults to None, which METERS ON SITE just before
        takeoff. Two sorties have already been lost to a hardcoded exposure:
        `flight01` to a gain below unity, and `flight02` to this default
        sitting at 2.0 ms — an indoor value — which clipped 98.5% of every
        frame. The daemon had the number (it recorded 94.5% saturated on
        capture #0) and nothing acted on it. Metering removes the human step
        that failed twice. Pass an explicit `exposure=` only to override it.

        record=yes records the WHOLE flight continuously (every sweep with
        its per-column timestamps, every frame, the Ouster IMU) from before
        arming to after landing — the real-rig counterpart of the Unity C2
        fixture, for LIO/odometry. prearm is the stationary hold before
        arming while recording (default 30 s; the gyro bias walk needs it).
        """
        await self._launch(
            session, fly=True, side=side, altitude=altitude, scans=scans,
            settle=settle, exposure=exposure, gain=gain, gate=gate,
            pattern=pattern, anchor=anchor, radius=radius, stations=stations,
            ring2_radius=ring2_radius, ring2_altitude=ring2_altitude,
            ring2_stations=ring2_stations, aim=aim, fan=fan,
            headings=headings, target_n=target_n, target_e=target_e,
            target_mark=target_mark, record=record, prearm=prearm)

    async def dryrun(self, session: str, side: float = 10.0,
                     altitude: float = 10.0, scans: int = 10,
                     settle: float = 0.5, exposure: float | None = None,
                     gain: float = 1.0, gate: str = "off",
                     pattern: str = "square", anchor: str = "corner",
                     radius: float = 7.0, stations: int = 8, aim: str = "no",
                     fan: float = 15.0, headings: int = 8,
                     ring2_radius: float = 12.0, ring2_altitude: float = 12.0,
                     ring2_stations: int = 6, target_n: float = 0.0,
                     target_e: float = 0.0, target_mark: str = "",
                     record: str = "yes", prearm: float = 0.0):
        """Run the whole capture loop with every flight command skipped.
        The recording still runs (record=yes) so the bench can verify the
        continuous files; the pre-arm hold defaults to 0 here."""
        await self._launch(
            session, fly=False, side=side, altitude=altitude, scans=scans,
            settle=settle, exposure=exposure, gain=gain, gate=gate,
            pattern=pattern, anchor=anchor, radius=radius, stations=stations,
            ring2_radius=ring2_radius, ring2_altitude=ring2_altitude,
            ring2_stations=ring2_stations, aim=aim, fan=fan,
            headings=headings, target_n=target_n, target_e=target_e,
            target_mark=target_mark, record=record, prearm=prearm)

    async def orbit(self, session: str, target_mark: str = "",
                    target_n: float = 0.0, target_e: float = 0.0,
                    scans: int = 1, settle: float = 2.0,
                    exposure: float | None = None, gain: float = 1.0,
                    gate: str = "warn", record: str = "yes",
                    prearm: float = -1.0):
        """Fly the chosen sortie: aimed orbit r=7 alt=7, 8 stations, ONE scan.

        Say where the object is with `--target_mark NAME` (recorded with
        sq-mark by carrying the drone to it) or `--target_n/--target_e`
        metres from the take-off point. scans defaults to 1 here, unlike
        sq-run: the 0.9 s ten-sweep window smears 11 cm and 2.3 deg in hover.
        The whole flight is recorded continuously (record=yes) with a 30 s
        stationary hold before arming (prearm); see `run`.
        """
        await self._launch(
            session, fly=True, scans=scans, settle=settle, exposure=exposure,
            gain=gain, gate=gate, target_n=target_n, target_e=target_e,
            target_mark=target_mark, record=record, prearm=prearm,
            **ORBIT_PRESET)

    async def orbit_plan(self, target_mark: str = "", target_n: float = 0.0,
                         target_e: float = 0.0):
        """Print the orbit schedule and the placement card, fly nothing."""
        await self.show_plan(target_n=target_n, target_e=target_e,
                             target_mark=target_mark, **ORBIT_PRESET)

    # ------------------------------------------------------------------ #
    #  Marks: where the object is, measured with the drone's own receiver
    # ------------------------------------------------------------------ #

    async def mark(self, name: str):
        """Record the drone's CURRENT position as NAME (carry it to the truck).

        Metre-level is enough: a metre of target error is ~8 deg off frame
        centre at 7 m and the 6 m truck stays in frame. One mark on the ground
        at the truck's mid-side is within that; front and rear bumper averaged
        (`--target_mark front,rear`) is better.
        """
        if not name or "/" in name or "\\" in name or ".." in name:
            self.logger.error("Bad mark name %r.", name)
            return
        drone = self.drones.get(next(iter(self.drones), None))
        if drone is None or not getattr(drone, "is_connected", False):
            self.logger.error("Add a connected drone first: %s-add <name>",
                              self.name)
            return
        fix = getattr(drone, "fix_type", None)
        fix_v = int(getattr(fix, "value", 0) or 0)
        if fix_v < 3:
            self.logger.error("No 3D fix (fix_type %s) — a mark now would be "
                              "a guess.", fix)
            return
        cur = self._read_global()
        if cur is None:
            self.logger.error("The drone reports no global position.")
            return
        if fix_v < 6:
            self.logger.warning("Fix type %d is not RTK fixed: the mark is "
                                "metre-level, which is within tolerance.", fix_v)
        ned = self._read_origin()
        marks = self._load_marks()
        marks[name] = {"lat": cur[0], "lon": cur[1], "amsl": cur[2],
                       "ned": list(ned) if ned is not None else None,
                       "fix": fix_v,
                       "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                       "drone": next(iter(self.drones))}
        path = self._save_marks(marks)
        self.logger.info("mark %s: lat %.7f lon %.7f amsl %.1f (fix %d) -> %s",
                         name, cur[0], cur[1], cur[2], fix_v, path)

    async def list_marks(self):
        """List the marks with distance and true bearing from the drone NOW."""
        marks = self._load_marks()
        if not marks:
            self.logger.info("No marks (%s).", self._marks_path())
            return
        cur = self._read_global()
        if cur is None:
            self.logger.warning("The drone reports no global position; "
                                "showing raw coordinates only.")
        for name, m in sorted(marks.items()):
            if cur is not None:
                n, e = plan.ned_from_llh(m["lat"], m["lon"], cur[0], cur[1])
                self.logger.info("  %-12s %6.1f m at bearing %03.0f from here "
                                 "(N%+.1f E%+.1f)  fix %s  %s", name,
                                 math.hypot(n, e), plan.bearing_deg(n, e),
                                 n, e, m.get("fix"), m.get("utc"))
            else:
                self.logger.info("  %-12s lat %.7f lon %.7f  fix %s  %s", name,
                                 m["lat"], m["lon"], m.get("fix"), m.get("utc"))
        self.logger.info("(%s)", self._marks_path())

    async def unmark(self, name: str):
        """Forget a mark."""
        marks = self._load_marks()
        if name not in marks:
            self.logger.warning("No mark %r.", name)
            return
        del marks[name]
        self._save_marks(marks)
        self.logger.info("Forgot mark %s.", name)

    async def abort(self):
        """Stop the sweep and land where we are."""
        task, self._flight_task = self._flight_task, None
        if task is not None and not task.done():
            self.logger.warning("ABORT requested — cancelling the sweep.")
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):            # noqa: BLE001
                pass
        self.current_stage = SquareStage.Aborted
        if self._link is not None:
            if self._recording is not None:
                r = await self._link.rec_stop()
                self._recording["summary"] = r.get("summary")
            await self._link.stop_session()
            await self._close_link()
        if self.drones:
            self.current_stage = SquareStage.Landing
            self.logger.warning("Landing at the current position.")
            await self.dm.land(list(self.drones), schedule=False)
            await self._hand_back(next(iter(self.drones)))
        self.current_stage = SquareStage.Aborted

    # -- orchestration ---------------------------------------------------- #

    def _running(self):
        return self._flight_task is not None and not self._flight_task.done()

    async def _launch(self, session, *, fly, side=10.0, altitude=10.0,
                      scans=10, settle=2.0, exposure=None, gain=64.0,
                      gate="warn", pattern="square", anchor="corner",
                      radius=7.0, stations=8, ring2_radius=12.0,
                      ring2_altitude=12.0, ring2_stations=6, aim="no",
                      fan=15.0, headings=8, target_n=0.0, target_e=0.0,
                      target_mark="", record="yes", prearm=-1.0):
        if self._running():
            self.logger.warning("Mission %s is already flying.", self.name)
            return
        if not session or "/" in session or "\\" in session:
            self.logger.warning("Bad session name %r.", session)
            return
        if len(self.drones) != 1:
            self.logger.warning("Add exactly one drone first: %s-add <name>",
                                self.name)
            return
        # Build and trial-plan the pattern HERE, before a task exists and
        # before the daemon is asked to open a session. An unflyable fan or a
        # heading on the wrap is a typo, and a typo should cost a log line
        # rather than an aborted sortie with a half-written session on disk.
        try:
            target_n, target_e, tinfo = self._resolve_target(
                target_mark, target_n, target_e)
            stations_ned, headings_fn = self._build_pattern(
                pattern, anchor, side, altitude, radius, stations,
                ring2_radius, ring2_altitude, ring2_stations, aim, fan,
                headings, target_n, target_e)
            plan.capture_plan(side_m=side, altitude_m=altitude,
                              origin_ned=(0.0, 0.0, 0.0), initial_yaw=0.0,
                              stations_ned=stations_ned,
                              headings_fn=headings_fn)
        except ValueError as exc:
            self.logger.error("Cannot plan %s: %s", pattern, exc)
            return
        # Everything the daemon never sees, kept for the sortie's own record.
        meta = dict(pattern=pattern, anchor=anchor, side=side,
                    altitude=altitude, radius=radius, stations=stations,
                    ring2_radius=ring2_radius, ring2_altitude=ring2_altitude,
                    ring2_stations=ring2_stations, aim=aim, fan=fan,
                    headings=headings, scans=scans, settle=settle,
                    exposure=exposure, gain=gain, gate=gate,
                    record=self._yes(record),
                    prearm=(float(prearm) if float(prearm) >= 0
                            else self.prearm_still_s),
                    target=dict(tinfo, offset_ned=[target_n, target_e]))
        self.side_m, self.altitude_m, self.settle_s = side, altitude, settle
        self._results = []
        self._recording = None
        self._flight_task = asyncio.create_task(
            self._sortie(session, side, altitude, scans, settle, exposure,
                         gain, gate, stations_ned, headings_fn, fly, meta))
        self.running_tasks.add(self._flight_task)

    async def _sortie(self, session, side, altitude, scans, settle, exposure,
                      gain, gate, stations_ned, headings_fn, fly, meta=None):
        name = next(iter(self.drones))
        t0 = time.time()
        origin, stations, started = None, None, {}
        outcome = "not_started"
        try:
            self.current_stage = SquareStage.Preflight
            if not await self._preflight(require_flight=fly):
                self.logger.error("Preflight failed — not flying.")
                self.current_stage = SquareStage.Idle
                return

            origin = self._read_origin() if fly else (0.0, 0.0, 0.0)
            if origin is None:
                self.logger.error("No local position — cannot anchor the "
                                  "pattern.")
                self.current_stage = SquareStage.Idle
                return
            self._origin = origin
            stations = plan.capture_plan(
                side_m=side, altitude_m=altitude, origin_ned=origin,
                initial_yaw=(self._read_yaw() or 0.0) if fly else 0.0,
                stations_ned=stations_ned, headings_fn=headings_fn)
            self._centred = stations_ned is not None
            for line in plan.plan_summary(stations):
                self.logger.info(line)
            if self._centred and meta:
                off = meta["target"]["offset_ned"]
                t = (origin[0] + off[0], origin[1] + off[1], origin[2])
                for line in plan.placement_card(origin, t, stations,
                                                with_map=False)[:2]:
                    self.logger.info(line)

            exposure_ms = exposure
            if exposure_ms is None:
                self.logger.info("Metering exposure on site (analog gain %.1f dB)...", gain)
                m = await self._link.meter(gain)
                if not m.get("ok"):
                    self.logger.error("Metering failed: %s — pass exposure= to "
                                      "override.", m.get("reason"))
                    self.current_stage = SquareStage.Idle
                    return
                exposure_ms = m["exposure_ms"]
                self.logger.info("Metered %.2f ms at %.1f dB: %.2f%% clipped",
                                 exposure_ms, m.get("gain_db", gain),
                                 m.get("saturated_pct", -1))
                if m.get("hint"):
                    self.logger.warning("Metering hint: %s", m["hint"])
                # A metered result pinned against a rail is not an exposure, it
                # is a report that the scene is outside what this gain can hold.
                # Flying on it burns the whole sortie, as flight01 and flight02
                # both did.
                if m.get("at_floor") and m.get("saturated_pct", 0) > 5.0:
                    self.logger.error(
                        "Still %.1f%% clipped at the shortest exposure the "
                        "sensor has. Lower the gain and re-run; not flying.",
                        m.get("saturated_pct"))
                    self.current_stage = SquareStage.Idle
                    return

            started = await self._link.start_session(
                session, scans, exposure_ms, gain, gate,
                require_pose=True, require_fix=bool(fly))
            if not started.get("ok"):
                self.logger.error("Capture daemon refused the session: %s %s",
                                  started.get("reason"), started.get("detail"))
                self.current_stage = SquareStage.Idle
                return
            self.logger.info("Capture session %s -> %s", session,
                             started.get("dir"))

            if meta and meta.get("record"):
                r = await self._link.rec_start()
                if not r.get("ok"):
                    # The recording is the point of this sortie; a refused
                    # recorder is a refused sortie, not a silent downgrade.
                    self.logger.error("Capture daemon refused to record: %s %s "
                                      "— not flying. Pass record=no to fly "
                                      "shots only.", r.get("reason"),
                                      r.get("detail") or "")
                    self.current_stage = SquareStage.Idle
                    return
                self._recording = {"started": r.get("recording"),
                                   "prearm_s": meta.get("prearm"),
                                   "summary": None}
                self.logger.info("Recording the whole flight -> %s",
                                 started.get("dir"))
                hold = float(meta.get("prearm") or 0.0)
                if hold > 0:
                    self.logger.info("Holding STILL for %.0f s before arming "
                                     "(pre-arm IMU segment) — do not touch "
                                     "the aircraft.", hold)
                    await asyncio.sleep(hold)

            if fly:
                await self._takeoff(name, origin, side, altitude, stations)
            try:
                await self._sweep(name, stations, fly)
            except CaptureAbort as exc:
                # Land. The old path set Aborted and left the aircraft
                # hovering on its last setpoint for the pilot to notice.
                self.logger.error("SORTIE ABORTED: %s — returning to land.", exc)
                outcome = "aborted"
                self.current_stage = SquareStage.Aborted
                if fly:
                    await self._return_and_land(name, origin, stations[0])
                self.current_stage = SquareStage.Aborted
                return
            if fly:
                await self._return_and_land(name, origin, stations[0])

            self.current_stage = SquareStage.Done
            outcome = "done"
        except asyncio.CancelledError:
            outcome = "cancelled"
            self.logger.warning("Sortie cancelled — the drone is holding its "
                                "last setpoint; %s-abort lands it.", self.name)
            raise
        except Exception as exc:                                   # noqa: BLE001
            outcome = "failed"
            self.logger.error("Sortie failed: %r", exc)
            self.logger.debug("traceback", exc_info=True)
            self.current_stage = SquareStage.Aborted
        finally:
            if self._link is not None:
                if self._recording is not None:
                    r = await self._link.rec_stop()
                    self._recording["summary"] = r.get("summary")
                    if r.get("ok"):
                        sm = r["summary"] or {}
                        self.logger.info("Recording: %s sweeps, %s frames, %s "
                                         "imu rows, %.2f GB, dropped %s/%s",
                                         sm.get("sweeps"), sm.get("frames"),
                                         sm.get("imu_rows"),
                                         (sm.get("bytes") or 0) / 1e9,
                                         sm.get("dropped_sweeps"),
                                         sm.get("dropped_frames"))
                await self._link.stop_session()
                await self._close_link()
            ok = sum(1 for r in self._results if r.get("ok"))
            self.logger.info("Sortie %s: %d/%d captures in %.1f s",
                             session, ok, len(self._results), time.time() - t0)
            for r in self._results:
                if not r.get("ok"):
                    self.logger.warning("  MISSED %s: %s %s", r["tag"],
                                        r.get("reason"), r.get("detail") or "")
            self.additional_info = {"session": session, "captured": ok,
                                    "planned": len(self._results)}
            # The sortie's own record, written ground-side because the
            # daemon's `start` whitelists its keys and would drop all of this.
            # A disk error here must never mask the landing path above.
            try:
                self._write_mission_json(session, meta, origin, stations,
                                         fly, started, outcome)
            except Exception as exc:                            # noqa: BLE001
                self.logger.error("mission.json not written: %r", exc)

    async def _takeoff(self, name, origin, side, altitude, stations=None):
        self.current_stage = SquareStage.Takeoff
        # A target-centred pattern is NOT bounded by the square formula — an
        # orbit around a target 10 m out puts half its ring outside that box,
        # and those waypoints would be rejected in the air.
        bounds = plan.fence_bounds(origin, side, altitude,
                                   stations=stations if self._centred else None)
        self.flight_area = SquareFlightArea(bounds)
        # dm.set_fence takes a constructed instance. Drone.set_fence does NOT —
        # it passes the logger as the first positional, which lands in
        # `north_lower` and builds a nonsense box. Use the manager one.
        self.dm.set_fence(name, RectLocalFence(*bounds, safety_level=3))
        self.logger.info("Fence N[%.1f,%.1f] E[%.1f,%.1f] D[%.1f,%.1f]", *bounds)

        if not self._ok(await asyncio.wait_for(
                self.dm.arm([name], schedule=False), TAKEOFF_TIMEOUT)):
            raise RuntimeError("arm refused")
        await asyncio.sleep(0.5)
        if not self._ok(await asyncio.wait_for(
                self.dm.takeoff([name], altitude=altitude, schedule=False),
                TAKEOFF_TIMEOUT)):
            raise RuntimeError("takeoff refused")
        self.logger.info("Airborne at %.1f m.", altitude)

    async def _sweep(self, name, stations, fly):
        """
        NOTE the argument shapes below, which are not interchangeable.
        `fly_to` and `yaw_to` go through `_multiple_drone_multiple_params_action`,
        which only unwraps "raw" per-drone arguments — `local=[n, e, d]` rather
        than `local=[[n, e, d]]` — when `names` is a bare STRING. Pass `[name]`
        there and the coordinates are read as one value per drone. `arm`,
        `takeoff`, `land` and `disarm` take the other path and are happy with a
        list.
        """
        drone = self.drones[name]
        streak, streak_reason = 0, None
        for station in stations:
            self.current_stage = SquareStage.Transit
            if fly:
                self.logger.info("-> station %d  N%+.2f E%+.2f D%+.2f  yaw %+.1f",
                                 station.corner, *station.ned, station.arrival_yaw)
                if not self._ok(await asyncio.wait_for(
                        self.dm.fly_to(name, local=list(station.ned),
                                       yaw=station.arrival_yaw,
                                       tol=self.position_tolerance,
                                       schedule=False), FLY_TIMEOUT)):
                    raise RuntimeError("fly_to station %d failed" % station.corner)

            self.current_stage = SquareStage.Sweep
            for cap in station.captures:
                if fly:
                    await self._point(drone, name, station.ned, cap.heading)
                # Settle is not just for the airframe to stop swinging: the
                # daemon averages the last `scans` sweeps, so the buffer must
                # have refilled since the yaw or the capture averages in sweeps
                # taken mid-rotation. At the dome's 10 Hz, the 2 s default
                # leaves ~20 fresh sweeps behind the 10 that get used. Shorten
                # it and the LiDAR smears.
                await asyncio.sleep(self.settle_s)
                await self._link.sync_if_due()
                ok, reason = await self._capture(cap, drone if fly else None)
                if ok or reason != streak_reason:
                    streak, streak_reason = (0 if ok else 1), (None if ok else reason)
                else:
                    streak += 1
                if streak >= self.max_consecutive_misses:
                    raise CaptureAbort(
                        "%d consecutive captures refused with %r (each "
                        "retried once); nothing this sortie records would "
                        "be usable" % (streak, reason))

    async def _point(self, drone, name, ned, heading):
        """Aim the rig, then pin the drone there for the exposure.

        `yaw_to` deactivates the path follower and streams its own setpoints,
        so `local` must be passed explicitly or it re-samples a noisy
        `position_ned` as the hold point and the station drifts across the
        sweep. It is also skipped when there is nothing to turn: `yaw_to`
        divides by a step count derived from the yaw delta, so a delta of
        exactly zero is a ZeroDivisionError rather than a no-op.
        """
        current = drone.attitude[2]
        if abs(plan.heading_delta(current, heading)) >= 0.5:
            if not self._ok(await asyncio.wait_for(
                    self.dm.yaw_to(name, yaw=float(heading),
                                   yaw_rate=self.yaw_rate, local=list(ned),
                                   tol=self.yaw_tolerance, schedule=False),
                    YAW_TIMEOUT)):
                raise RuntimeError("yaw_to %+.1f failed" % heading)
        # Re-assert the exact station pose. MAVSDK re-streams the last offboard
        # setpoint in the background, so this one command holds both position
        # and heading for as long as the capture takes.
        await drone.set_setpoint(Waypoint(WayPointType.POS_NED, pos=list(ned),
                                          yaw=float(heading)))

    async def _capture(self, cap, drone):
        reply = await self._link.shot(cap.tag)
        rec = reply.get("record") or {}
        entry = dict(tag=cap.tag, corner=cap.corner, heading=cap.heading,
                     ok=bool(reply.get("ok")), id=reply.get("id"),
                     reason=reply.get("reason"), detail=reply.get("detail"))
        if drone is not None:
            # The commanded pose, recorded next to the daemon's own FC stamp:
            # a disagreement between the two is the cheapest possible check
            # that the drone was where the plan says it was.
            entry["commanded_ned"] = [float(v) for v in drone.position_ned]
            entry["commanded_yaw"] = float(drone.attitude[2])
        self._results.append(entry)

        if reply.get("ok"):
            self.logger.info("  %s #%s pts=%s sat=%.1f%% gyro=%s pose=%s",
                             cap.tag, reply.get("id"), rec.get("points"),
                             rec.get("saturated_pct") or 0.0, rec.get("gyro_peak"),
                             (rec.get("pose") or {}).get("have"))
            return True, None
        else:
            # One retry, then move on. Stranding the drone in a hover to chase a
            # single frame trades a whole sortie for 1/32nd of a dataset.
            self.logger.warning("  %s REFUSED: %s %s — retrying once", cap.tag,
                                reply.get("reason"), reply.get("detail") or "")
            await asyncio.sleep(self.retry_delay_s)
            reply = await self._link.shot(cap.tag)
            entry.update(ok=bool(reply.get("ok")), id=reply.get("id"),
                         reason=reply.get("reason"), detail=reply.get("detail"),
                         retried=True)
            if reply.get("ok"):
                # Say so. Without this the operator sees a REFUSED warning and
                # then silence, and cannot tell a recovered capture from a lost
                # one until the end-of-sortie tally.
                rec = reply.get("record") or {}
                self.logger.info("  %s #%s RECOVERED on retry  pts=%s pose=%s",
                                 cap.tag, reply.get("id"), rec.get("points"),
                                 (rec.get("pose") or {}).get("have"))
                return True, None
            self.logger.error("  %s LOST: %s", cap.tag, reply.get("reason"))
            return False, reply.get("reason")

    async def _return_and_land(self, name, origin, first):
        """Fly back over the TAKE-OFF POINT at station 0's altitude, then land.

        For the take-off-anchored square that IS station 0, which is what this
        did before. For a target-centred pattern station 0 sits on the ring,
        metres from the landing spot and possibly over the object — the
        take-off point is the only place the aircraft is known to be able to
        come down.
        """
        self.current_stage = SquareStage.Return
        back = (float(origin[0]), float(origin[1]), float(first.ned[2]))
        self.logger.info("Returning to the take-off point at %.1f m.",
                         -back[2] + float(origin[2]))
        await asyncio.wait_for(
            self.dm.fly_to(name, local=list(back), yaw=first.arrival_yaw,
                           tol=self.position_tolerance, schedule=False),
            FLY_TIMEOUT)
        self.current_stage = SquareStage.Landing
        await asyncio.wait_for(self.dm.land([name], schedule=False), LAND_TIMEOUT)
        await asyncio.sleep(1.0)
        await self.dm.disarm([name], schedule=False)
        await self._hand_back(name)
        self.logger.info("Landed and disarmed.")

    async def _hand_back(self, name):
        """Leave the aircraft in Position mode for the pilot.

        DroneManager's landing is an offboard descent: it does not disarm and it
        does not leave offboard, so without this the FC sits in offboard with
        nothing streaming setpoints and fails safe on its own terms rather than
        ours. The shipped UAM mission ends the same way.
        """
        try:
            await self.dm.change_flightmode(name, "position")
        except Exception as exc:                                   # noqa: BLE001
            self.logger.warning("Could not return %s to position mode: %r",
                                name, exc)

    # -- preflight -------------------------------------------------------- #

    async def _preflight(self, require_flight=True):
        ok = True
        if len(self.drones) != 1:
            self.logger.error("Add exactly one drone: %s-add <name>", self.name)
            return False
        name = next(iter(self.drones))
        drone = self.drones[name]

        if not drone.is_connected:
            self.logger.error("%s is not connected.", name)
            ok = False
        if require_flight:
            if not drone.parameters_loaded:
                self.logger.error("%s has not loaded FC parameters yet.", name)
                ok = False
            if self._read_origin() is None:
                self.logger.error("%s has no local NED position.", name)
                ok = False
            # mavsdk.telemetry.FixType follows the MAVLink numbering —
            # 0 no GPS, 1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed —
            # so >= 3 is "3D or better".
            fix = getattr(drone, "fix_type", None)
            fix_ok = fix is not None and getattr(fix, "value", 0) >= 3
            self.logger.info("GPS fix: %s", fix)
            if not fix_ok:
                self.logger.error(
                    "No 3D fix. If SYS_STATUS reports the GPS bit "
                    "present=False the FC has not detected the receiver at all "
                    "— reboot the FC with it connected; a detected receiver "
                    "with no satellites still reports present=True.")
                ok = False
            # ⚠ NO BATTERY GATE. Removed 2026-08-31 on the operator's
            # instruction, because on this airframe it could only ever produce
            # FALSE failures: BAT1_SOURCE=2 (ESCs) with DSHOT_TEL_CFG=0 means
            # `battery_status` is never published, so `remaining` is always
            # MAVSDK's -1.0 "unknown" sentinel — which the old check compared
            # against 35% and refused on, reporting a flat pack when the real
            # one was at 97%.
            #
            # The value is still logged, because it is the only battery signal
            # the mission has. THE CONSEQUENCE IS REAL: nothing in software will
            # stop this sortie on a low pack, and PX4 has no low-battery
            # failsafe either without telemetry. Watch the pack yourself.
            # Restore a gate here once DSHOT_TEL_CFG points at the ESC telemetry
            # UART and `remaining` reports something other than -1.0.
            batts = getattr(drone, "batteries", None) or {}
            for bid, b in batts.items():
                rem = getattr(b, "remaining", None)
                self.logger.info("Battery %s: %s%s", bid, rem,
                                 "  (no telemetry — not checked)"
                                 if rem is None or rem < 0 else "  (not checked)")

        if self._link is None:
            self._link = CaptureLink(self.logger, self._host, self._user,
                                     self._port, self._direct, self._token)
        st = await self._link.status()
        if not st.get("ok", True) or "lidar" not in st:
            self.logger.error("Capture daemon unreachable at %s@%s:%d: %s",
                              self._user, self._host, self._port,
                              st.get("reason"))
            return False
        lid, cam = st.get("lidar", {}), st.get("camera", {})
        pose, disk = st.get("pose", {}), st.get("disk", {})
        self.logger.info("Daemon: lidar %sf/%sms  cam %sf  pose_link=%s  "
                         "free=%sGB  session=%s",
                         lid.get("frames"), lid.get("age_ms"), cam.get("frames"),
                         pose.get("link_up"), disk.get("free_gb"),
                         (st.get("session") or {}).get("name"))
        if st.get("session"):
            self.logger.error("A capture session is already open on the daemon.")
            ok = False
        if not lid.get("frames"):
            self.logger.error("No LiDAR sweeps arriving.")
            ok = False
        if not cam.get("frames"):
            self.logger.error("No camera frames arriving.")
            ok = False
        if not pose.get("link_up"):
            self.logger.error("The daemon sees no MAVLink pose.")
            ok = False
        await self._link.sync_if_due(force=True)
        return ok

    # -- helpers ---------------------------------------------------------- #

    def _ok(self, results):
        """Manager actions return a LIST of per-drone results, and they RETURN
        exceptions rather than raising them (`gather(return_exceptions=True)`),
        or return None outright when the name lookup failed. All three of those
        are failures and all three are falsy-adjacent enough to slip through a
        naive truth test."""
        if not results or isinstance(results, Exception):
            return False
        first = results[0] if isinstance(results, (list, tuple)) else results
        if isinstance(first, Exception):
            return False
        return first is not False

    def _read_origin(self):
        drone = self.drones.get(next(iter(self.drones), None))
        if drone is None:
            return None
        try:
            pos = [float(v) for v in drone.position_ned]
        except (TypeError, ValueError):
            return None
        return None if any(v != v for v in pos) else tuple(pos)

    def _read_yaw(self):
        drone = self.drones.get(next(iter(self.drones), None))
        if drone is None:
            return None
        try:
            yaw = float(drone.attitude[2])
        except (TypeError, ValueError, IndexError):
            return None
        return None if yaw != yaw else yaw

    def _read_global(self):
        """(lat, lon, amsl) or None.

        ⚠ A drone without a fix reports ZEROS, not NaN: DroneManager's
        `_position_g` starts as np.zeros and only telemetry.position() fills
        it. Null Island is not a position.
        """
        drone = self.drones.get(next(iter(self.drones), None))
        if drone is None:
            return None
        try:
            g = [float(v) for v in getattr(drone, "position_global")[:3]]
        except (TypeError, ValueError, IndexError, AttributeError):
            return None
        if any(v != v for v in g) or (g[0] == 0.0 and g[1] == 0.0):
            return None
        return tuple(g)

    def _marks_path(self):
        return os.path.abspath(os.path.join(USER_DIR, MARKS_FILE))

    def _mission_json_path(self, session):
        d = os.path.join(USER_DIR, SESSIONS_SUBDIR)
        path = os.path.join(d, session + ".mission.json")
        if os.path.exists(path):               # a re-fly keeps the first record
            path = os.path.join(d, "%s.%s.mission.json"
                                % (session, time.strftime("%Y%m%dT%H%M%SZ",
                                                          time.gmtime())))
        return os.path.abspath(path)

    @staticmethod
    def _write_json(path, obj):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(obj, f, indent=1)
        os.replace(tmp, path)                  # atomic, Windows included
        return path

    def _load_marks(self):
        path = self._marks_path()
        if not os.path.isfile(path):
            return {}
        try:
            with open(path) as f:
                marks = json.load(f)
            return marks if isinstance(marks, dict) else {}
        except (OSError, ValueError) as exc:
            self.logger.error("Cannot read %s: %r", path, exc)
            return {}

    def _save_marks(self, marks):
        return self._write_json(self._marks_path(), marks)

    def _resolve_target(self, target_mark, target_n, target_e):
        """-> (target_n, target_e, info). ValueError to refuse.

        The offset is computed from the GNSS positions of the mark and of the
        drone NOW, so it survives a reboot between marking and flying (the
        EKF-local frame does not — `LOCAL_POSITION_NED` resets). Same
        receiver, minutes apart: the common-mode GNSS error cancels, which is
        why this is better than either position's absolute accuracy, and why
        "not RTK fixed" is a warning at mark time rather than a gate here.
        """
        names = [s.strip() for s in str(target_mark or "").split(",")
                 if s.strip()]
        if not names:
            return float(target_n), float(target_e), {"source": "offset",
                                                      "marks": []}
        if target_n or target_e:
            raise ValueError("--target_mark and --target_n/--target_e were "
                             "both given; use one")
        marks = self._load_marks()
        missing = [n for n in names if n not in marks]
        if missing:
            raise ValueError("unknown mark(s) %s — %s-marks lists them"
                             % (", ".join(missing), self.name))
        cur = self._read_global()
        if cur is None:
            raise ValueError("the drone reports no global position, so a "
                             "mark cannot be resolved into an offset")
        offs = [plan.ned_from_llh(marks[n]["lat"], marks[n]["lon"],
                                  cur[0], cur[1]) for n in names]
        tn = sum(o[0] for o in offs) / len(offs)
        te = sum(o[1] for o in offs) / len(offs)

        ned_now = self._read_origin()
        for n, (on, oe) in zip(names, offs):
            mned = marks[n].get("ned")
            if ned_now is None or mned is None:
                continue
            dn, de = mned[0] - ned_now[0], mned[1] - ned_now[1]
            gap = math.hypot(dn - on, de - oe)
            if gap > MARK_MAX_NED_DISAGREE_M:
                self.logger.warning(
                    "mark %s: GNSS says N%+.1f E%+.1f but the EKF-local delta "
                    "says N%+.1f E%+.1f (%.1f m apart) — the EKF origin moved "
                    "since marking (reboot or reset); using GNSS.",
                    n, on, oe, dn, de, gap)
        if len(offs) > 1:
            spread = max(math.hypot(o[0] - tn, o[1] - te) for o in offs)
            self.logger.info("target = mean of %d marks, %.1f m spread",
                             len(offs), spread)
        return tn, te, {
            "source": "mark", "marks": names,
            "global": [sum(marks[n]["lat"] for n in names) / len(names),
                       sum(marks[n]["lon"] for n in names) / len(names)],
            "from_global": list(cur),
            "amsl_delta": (sum(marks[n].get("amsl", cur[2]) for n in names)
                           / len(names) - cur[2]),
        }

    def _write_mission_json(self, session, meta, origin, stations, fly,
                            started, outcome):
        """One JSON per sortie with what the daemon never sees.

        `outcome` is done / cancelled / failed / not_started; `stage` is where
        the sortie WAS when that happened (an abort during take-off reads
        'Takeoff', which is the useful number — sq-abort sets Aborted only
        after this record is written).
        """
        rec = {
            "session": session, "fly": bool(fly),
            "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "drone": next(iter(self.drones), None),
            "outcome": outcome,
            "stage": self.current_stage.name,
            "params": meta or {},
            "origin_ned": list(origin) if origin is not None else None,
            "origin_global": (list(g) if (g := self._read_global()) else None),
            "fence": (list(self.flight_area.bounds)
                      if getattr(self, "flight_area", None) is not None
                      else None),
            "stations": [{"corner": s.corner, "ned": list(s.ned),
                          "arrival_yaw": s.arrival_yaw,
                          "headings": [c.heading for c in s.captures]}
                         for s in (stations or [])],
            "daemon": {"host": self._host, "session_dir": started.get("dir")},
            "recording": self._recording,
            "results": list(self._results),
        }
        if origin is not None and meta and meta.get("target"):
            off = meta["target"].get("offset_ned", [0.0, 0.0])
            rec["target_ned"] = [origin[0] + off[0], origin[1] + off[1],
                                 origin[2]]
        path = self._write_json(self._mission_json_path(session), rec)
        self.logger.info("Sortie record -> %s", path)
        return path

    async def _close_link(self):
        if self._link is not None:
            await self._link.close()
            self._link = None

    async def close(self):
        await self._close_link()
        await super().close()
