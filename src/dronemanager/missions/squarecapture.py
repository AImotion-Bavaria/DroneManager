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

SAFETY. The RC transmitter is the real abort: a mode switch takes the drone out
of offboard immediately and nothing here can override that. If this mission's
task dies for any reason the drone keeps hovering on MAVSDK's last streamed
setpoint — it does not fall and it does not fly away — so `sq-abort` cancels the
sweep and then commands a landing, rather than relying on the cancellation
itself to be safe.
"""

import asyncio
import enum
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

CALL_TIMEOUT = 15.0        # daemon's own shot_timeout is 3 s; this is the wire
FLY_TIMEOUT = 120.0        # a 10 m leg at a few m/s, with generous slack
YAW_TIMEOUT = 45.0         # 45 deg at 30 deg/s is 1.5 s
TAKEOFF_TIMEOUT = 120.0
LAND_TIMEOUT = 180.0
SYNC_EVERY_S = 30.0        # the Jetson has no RTC; re-anchor its clock in flight


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
        (self._x_min, self._x_max, self._y_min, self._y_max,
         self._z_min, self._z_max) = (float(b) for b in bounds)

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

    async def start_session(self, name, scans, exposure_ms, gain, gate,
                            require_pose, require_fix):
        return await self.call({
            "cmd": "start", "name": name, "scans": scans,
            "exposure_ms": exposure_ms, "gain": gain,
            "average": True, "gate": gate,
            "require_pose": require_pose, "require_fix": require_fix,
            "gcs_utc_ns": time.time_ns(),
            "gcs_utc_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }, timeout=30.0)

    async def meter(self, gain, target_sat=0.5):
        """Ask the daemon to meter the exposure on the real picture."""
        return await self.call({"cmd": "meter", "gain": gain,
                                "target_sat": target_sat}, timeout=90.0)

    async def shot(self, tag):
        return await self.call({"cmd": "shot", "tag": tag})

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
        })
        self.current_stage = SquareStage.Idle

        self.side_m = 10.0
        self.altitude_m = 10.0
        self.settle_s = 2.0
        self.retry_delay_s = 1.0
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

    async def show_plan(self, side: float = 10.0, altitude: float = 10.0):
        """Print the station/heading schedule without flying it."""
        origin = self._read_origin() or (0.0, 0.0, 0.0)
        stations = plan.capture_plan(side_m=side, altitude_m=altitude,
                                     origin_ned=origin,
                                     initial_yaw=self._read_yaw() or 0.0)
        for line in plan.plan_summary(stations):
            self.logger.info(line)
        self.logger.info("fence %s", ("N[%.1f,%.1f] E[%.1f,%.1f] D[%.1f,%.1f]"
                                      % plan.fence_bounds(origin, side, altitude)))

    async def preflight(self):
        """Check the drone, the capture daemon and the sensors without flying."""
        ok = await self._preflight(require_flight=True)
        self.logger.info("Preflight %s.", "PASSED" if ok else "FAILED")
        return ok

    async def run(self, session: str, side: float = 10.0, altitude: float = 10.0,
                  scans: int = 10, settle: float = 2.0,
                  exposure: float | None = None,
                  gain: float = 64.0, gate: str = "warn"):
        """Fly the square and capture the dataset. Fully autonomous.

        ⚠ gain defaults to 64 — UNITY, the lowest gain whose white point reaches
        255 — and must not be lowered for "headroom". `flight01` flew at gain 0,
        where a saturated pixel maxes out at 117, and returned 32 unusable flat-
        wash frames that every clip test scored at 0.00%. Below unity you do not
        gain headroom, you throw away two thirds of the output range.

        exposure defaults to None, which METERS ON SITE just before takeoff.
        Two sorties have already been lost to a hardcoded exposure: `flight01`
        to a gain below unity, and `flight02` to this default sitting at 2.0 ms
        — an indoor value — which clipped 98.5% of every frame at unity gain.
        The daemon had the number (it recorded 94.5% saturated on capture #0)
        and nothing acted on it. Metering removes the human step that failed
        twice. Pass an explicit `exposure=` only to override it.
        """
        await self._launch(session, side, altitude, scans, settle, exposure,
                           gain, gate, fly=True)

    async def dryrun(self, session: str, side: float = 10.0,
                     altitude: float = 10.0, scans: int = 10,
                     settle: float = 0.5, exposure: float | None = None,
                     gain: float = 64.0, gate: str = "off"):
        """Run the whole capture loop with every flight command skipped."""
        await self._launch(session, side, altitude, scans, settle, exposure,
                           gain, gate, fly=False)

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

    async def _launch(self, session, side, altitude, scans, settle, exposure,
                      gain, gate, fly):
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
        self.side_m, self.altitude_m, self.settle_s = side, altitude, settle
        self._results = []
        self._flight_task = asyncio.create_task(
            self._sortie(session, side, altitude, scans, settle, exposure,
                         gain, gate, fly))
        self._running_tasks.add(self._flight_task)

    async def _sortie(self, session, side, altitude, scans, settle, exposure,
                      gain, gate, fly):
        name = next(iter(self.drones))
        t0 = time.time()
        try:
            self.current_stage = SquareStage.Preflight
            if not await self._preflight(require_flight=fly):
                self.logger.error("Preflight failed — not flying.")
                self.current_stage = SquareStage.Idle
                return

            origin = self._read_origin() if fly else (0.0, 0.0, 0.0)
            if origin is None:
                self.logger.error("No local position — cannot anchor the square.")
                self.current_stage = SquareStage.Idle
                return
            self._origin = origin
            stations = plan.capture_plan(
                side_m=side, altitude_m=altitude, origin_ned=origin,
                initial_yaw=(self._read_yaw() or 0.0) if fly else 0.0)
            for line in plan.plan_summary(stations):
                self.logger.info(line)

            exposure_ms = exposure
            if exposure_ms is None:
                self.logger.info("Metering exposure on site (gain %.0f)...", gain)
                m = await self._link.meter(gain)
                if not m.get("ok"):
                    self.logger.error("Metering failed: %s — pass exposure= to "
                                      "override.", m.get("reason"))
                    self.current_stage = SquareStage.Idle
                    return
                exposure_ms = m["exposure_ms"]
                self.logger.info("Metered %.2f ms at gain %.0f: %.2f%% clipped "
                                 "against white point %s",
                                 exposure_ms, m.get("gain", gain),
                                 m.get("saturated_pct", -1), m.get("white_point"))
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

            if fly:
                await self._takeoff(name, origin, side, altitude)
            await self._sweep(name, stations, fly)
            if fly:
                await self._return_and_land(name, stations[0])

            self.current_stage = SquareStage.Done
        except asyncio.CancelledError:
            self.logger.warning("Sortie cancelled — the drone is holding its "
                                "last setpoint; %s-abort lands it.", self.name)
            raise
        except Exception as exc:                                   # noqa: BLE001
            self.logger.error("Sortie failed: %r", exc)
            self.logger.debug("traceback", exc_info=True)
            self.current_stage = SquareStage.Aborted
        finally:
            if self._link is not None:
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

    async def _takeoff(self, name, origin, side, altitude):
        self.current_stage = SquareStage.Takeoff
        bounds = plan.fence_bounds(origin, side, altitude)
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
        for station in stations:
            self.current_stage = SquareStage.Transit
            if fly:
                self.logger.info("-> corner %d  N%+.2f E%+.2f D%+.2f  yaw %+.1f",
                                 station.corner, *station.ned, station.arrival_yaw)
                if not self._ok(await asyncio.wait_for(
                        self.dm.fly_to(name, local=list(station.ned),
                                       yaw=station.arrival_yaw,
                                       tol=self.position_tolerance,
                                       schedule=False), FLY_TIMEOUT)):
                    raise RuntimeError("fly_to corner %d failed" % station.corner)

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
                await self._capture(cap, drone if fly else None)

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
            else:
                self.logger.error("  %s LOST: %s", cap.tag, reply.get("reason"))

    async def _return_and_land(self, name, first):
        self.current_stage = SquareStage.Return
        self.logger.info("Returning to corner 0.")
        await asyncio.wait_for(
            self.dm.fly_to(name, local=list(first.ned), yaw=first.arrival_yaw,
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

    async def _close_link(self):
        if self._link is not None:
            await self._link.close()
            self._link = None

    async def close(self):
        await self._close_link()
        await super().close()
