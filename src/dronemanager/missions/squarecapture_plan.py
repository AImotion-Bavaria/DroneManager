"""Flight-plan geometry for the square capture mission.

STDLIB ONLY, and it imports nothing from DroneManager or LiveGS on purpose:
everything here is arithmetic that decides where the drone stops and which way
it points, which is exactly the part that must be unit-tested on a workstation
rather than debugged at 10 m over a field. `squarecapture.py` is the thin async
shell that flies what this module computes.

The default plan is the campaign's dataset: a 10 m square at 10 m altitude,
a stop at each of the 4 corners, 8 headings per stop in 45 deg steps
=> 32 image/LiDAR pairs.

    >>> stations = capture_plan()
    >>> sum(len(s.captures) for s in stations)
    32
"""

import math
from collections import namedtuple

# --------------------------------------------------------------------------- #
#  Rig geometry
# --------------------------------------------------------------------------- #

#: Where the LiDAR/camera payload sits relative to the flight controller, in the
#: FC body frame (FRD: x forward, y right, z DOWN). The payload hangs 25 cm
#: below the FC, so z is POSITIVE.
#:
#: The capture daemon stamps FC pose and never launders it (`trust:
#: "prior_only"`). Whatever consumes those priors must push them through
#: `payload_position_ned` first — 25 cm is an order of magnitude larger than the
#: position accuracy the prior is there to seed, so skipping it is not a rounding
#: error, it is a systematic offset that biases every capture the same way and
#: therefore will NOT show up as registration noise.
PAYLOAD_OFFSET_FC_M = (0.0, 0.0, 0.25)

# --------------------------------------------------------------------------- #
#  Headings
# --------------------------------------------------------------------------- #

#: The 8 sweep headings, 45 deg apart, deliberately offset by 22.5 deg from the
#: cardinals.
#:
#: WHY THE OFFSET: DroneManager's `Drone.is_at_heading` (drone.py:399) compares
#: `abs(cur - target) < tolerance` with NO angular-difference wrap. It normalises
#: the target into (-180, 180], so a target of exactly 180 becomes -180, and a
#: drone sitting at +179.9 computes `abs(179.9 - -180) = 359.8` and never
#: converges: `fly_to` and `yaw_to` both spin on that check forever. Rotating the
#: whole set by 22.5 deg keeps every target 22.5 deg clear of the discontinuity
#: and costs the dataset nothing — the scene does not care which way is 0.
HEADINGS_8 = (-157.5, -112.5, -67.5, -22.5, 22.5, 67.5, 112.5, 157.5)

#: How close a heading may come to the +/-180 discontinuity before it is unsafe.
HEADING_WRAP_GUARD_DEG = 5.0


def normalize_heading(deg):
    """Fold a heading into (-180, 180], the range DroneManager commands in."""
    out = (float(deg) + 180.0) % 360.0 - 180.0
    # (-180 + 180) % 360 - 180 == -180; prefer the +180 end so the value stays
    # inside the half-open range the docstring promises.
    return 180.0 if out == -180.0 else out


def heading_delta(from_deg, to_deg):
    """Signed shortest rotation from one heading to another, in (-180, 180]."""
    return normalize_heading(to_deg - from_deg)


def is_wrap_safe(deg, guard=HEADING_WRAP_GUARD_DEG):
    """True if `is_at_heading` can actually converge on this target."""
    return abs(abs(normalize_heading(deg)) - 180.0) > guard


def order_headings(headings, from_yaw):
    """Rotate `headings` so the sweep starts at whichever one is closest to the
    drone's current heading, then proceeds monotonically around the circle.

    Every heading is visited exactly once either way, so the direction is free;
    the start is not. Beginning at the nearest heading saves up to 180 deg of
    rotation on arrival, and because the same 8 headings are used at every
    station, station N+1 starts on the heading station N ended on — the transit
    leg between corners needs no rotation at all.
    """
    ring = sorted(normalize_heading(h) for h in headings)
    if not ring:
        return []
    # Tie-break on the heading itself so a drone pointing exactly between two
    # candidates still produces one deterministic plan.
    start = min(range(len(ring)),
                key=lambda i: (abs(heading_delta(from_yaw, ring[i])), ring[i]))
    return ring[start:] + ring[:start]


# --------------------------------------------------------------------------- #
#  The plan
# --------------------------------------------------------------------------- #

Capture = namedtuple("Capture", "corner heading tag")
Station = namedtuple("Station", "corner ned arrival_yaw captures")


def centred_square(side_m, altitude_m):
    """The 4 corners of a square CENTRED on the target, as (north, east, alt).

    The mission's own `square_corners` anchors at the drone's take-off corner,
    because that is what the operator can stand at. A capture-pattern study
    anchors on the OBJECT instead — the truck is the thing worth being in the
    middle of. The two differ by a constant translation, which every shape
    comparison cancels, so the same plan describes both.
    """
    h = float(side_m) / 2.0
    a = float(altitude_m)
    return [(-h, -h, a), (h, -h, a), (h, h, a), (-h, h, a)]


def circle_stations(radius_m, n, altitude_m, phase_deg=None):
    """`n` stations on a circle about the target, as (north, east, alt).

    ⚠ Radius and altitude are not independent for this airframe. The payload is
    bolted at a fixed 45 deg nose-down and does not gimbal, so the look line
    meets the ground at a horizontal distance equal to the altitude: only
    `radius == altitude` puts the target in the middle of the frame. Any other
    pair points the camera past it or short of it, which is a coverage choice,
    not a free parameter.
    """
    if radius_m <= 0:
        raise ValueError("radius_m must be positive")
    if n <= 0:
        raise ValueError("n must be positive")
    if phase_deg is None:
        # HALF A STEP off the cardinals, for the same reason HEADINGS_8 is
        # offset 22.5 deg: an inward-aiming station due south of the target
        # commands a heading of exactly 180, and `is_at_heading` compares
        # without wrapping, so `yaw_to` spins on it forever. At n=8 this is
        # 22.5 deg and the bearings land exactly on HEADINGS_8. Measured: with
        # phase 0 the plan is REFUSED by the guard below.
        phase_deg = 180.0 / int(n)
    out = []
    for i in range(int(n)):
        t = math.radians(float(phase_deg) + 360.0 * i / int(n))
        out.append((radius_m * math.cos(t), radius_m * math.sin(t),
                    float(altitude_m)))
    return out


def pattern_stations(pattern, side_m=10.0, altitude_m=10.0, radius_m=7.0,
                     n_stations=8, ring2_radius_m=12.0, ring2_altitude_m=12.0,
                     ring2_stations=6):
    """Named capture patterns, as (north, east, alt) offsets from the target.

    Mirrors `SquareCaptureScanner.StationsNed()` in the Unity project. The two
    must agree or the checker is verifying a different plan than the one that
    was flown — which is the entire reason `unity_square_check` imports this
    module rather than reimplementing it.
    """
    if pattern == "square":
        return centred_square(side_m, altitude_m)
    if pattern == "square+centre":
        # A 5th station overhead turns the held-out corner from an
        # EXTRAPOLATION (outside the convex hull of the rest, the hardest case)
        # into an interpolation.
        return centred_square(side_m, altitude_m) + [(0.0, 0.0, altitude_m)]
    if pattern == "orbit":
        return circle_stations(radius_m, n_stations, altitude_m)
    if pattern == "tworings":
        # Half-step the outer ring so its stations sit BETWEEN the inner ring's
        # azimuths: two stations on one bearing add range diversity but no new
        # azimuth, and azimuth is the axis the office01 planning study found
        # short.
        # Both rings take their OWN half-step phase, which is the only phase
        # that keeps every inward bearing off the wrap. ⚠ Offsetting ring 2 by
        # half of ring 1's step instead — the obvious way to interleave — puts
        # it back ON the cardinals and the plan is refused. With EQUAL station
        # counts the two rings therefore share azimuths; to interleave, give
        # them different counts (the default 8 and 6 do).
        return (circle_stations(radius_m, n_stations, altitude_m) +
                circle_stations(ring2_radius_m, ring2_stations,
                                ring2_altitude_m))
    raise ValueError("unknown pattern %r (square, square+centre, orbit, "
                     "tworings)" % (pattern,))


def headings_toward(station_ned, n, fan_deg=45.0):
    """`n` headings fanned about the bearing from a station back to the target.

    The mount does not gimbal, so this sets YAW only; the 45 deg pitch is fixed
    and it is the station's radius/altitude that decides whether the target is
    in frame.

    ⚠ These are COMPUTED headings, so unlike `HEADINGS_8` they can land on the
    +/-180 discontinuity that `is_at_heading` cannot converge on. `capture_plan`
    still refuses such a set — that guard is the reason this returns headings
    rather than flying them.
    """
    n = max(1, int(n))
    # A station directly ABOVE the target has no inward bearing at all — the
    # horizontal vector to it is zero and atan2(-0,-0) quietly returns -180,
    # the one heading that hangs `is_at_heading`. Such a station sees a ring
    # around the target through the fixed 45 deg mount whichever way it points,
    # so the full sweep is both the safe answer and the useful one.
    if math.hypot(float(station_ned[0]), float(station_ned[1])) < 1e-6:
        return list(HEADINGS_8[:n]) if n <= len(HEADINGS_8) else list(HEADINGS_8)
    to_target = math.degrees(math.atan2(-station_ned[1], -station_ned[0]))
    if n == 1 or fan_deg <= 0:
        return [normalize_heading(to_target)] * n
    return [normalize_heading(to_target + (2.0 * i / (n - 1) - 1.0) * fan_deg)
            for i in range(n)]


def max_safe_fan_deg(stations_ned, guard=HEADING_WRAP_GUARD_DEG):
    """The widest aim fan that keeps every heading off the +/-180 discontinuity.

    A station's inward bearing is fixed by where it sits, so the fan is the only
    free part — and the constraint is not obvious from the pattern: a ring
    phased to put every BEARING safely off 180 can still have a fan that sweeps
    through it. On a half-step-phased 8-station ring the nearest bearing is
    22.5 deg from the wrap, so the fan may reach 17.5; on a square the diagonals
    sit 45 deg away and it may reach 40.

    Returns the limit in degrees (0 if even a dead-on aim is unsafe).
    """
    worst = 180.0
    for st in stations_ned:
        if math.hypot(float(st[0]), float(st[1])) < 1e-6:
            continue          # overhead: swept, not aimed (see headings_toward)
        b = normalize_heading(math.degrees(math.atan2(-st[1], -st[0])))
        worst = min(worst, abs(abs(b) - 180.0))
    return max(0.0, worst - float(guard))


def square_corners(side_m):
    """The 4 corners of a square as (north, east) offsets from the drone's
    position at mission start. The operator therefore places the drone at the
    SOUTH-WEST corner and the square grows north and east from there.

    Returned in a traversal order that walks the perimeter (no diagonal).
    """
    s = float(side_m)
    return [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)]


def format_tag(corner, heading):
    """The `tag` string handed to the capture daemon, e.g. `c0_h-157.5`.

    It carries the SIGNED NED yaw that was actually commanded, not a compass
    bearing, so the tag can be compared against the recorded FC attitude without
    a convention conversion in between — that conversion is precisely where the
    tripod sessions kept losing an hour.
    """
    return "c%d_h%+06.1f" % (int(corner), normalize_heading(heading))


def capture_plan(side_m=10.0, altitude_m=10.0, origin_ned=(0.0, 0.0, 0.0),
                 headings=HEADINGS_8, initial_yaw=0.0, stations_ned=None,
                 headings_fn=None, wrap_guard=True):
    """Build the full station/heading schedule.

    :param side_m: square edge length, metres.
    :param altitude_m: height above the mission-start position, metres (positive
        up; it is converted to NED down internally).
    :param origin_ned: the drone's local NED position when the mission starts,
        which is the square's south-west corner. The local NED origin is the EKF
        origin, NOT the takeoff point, so this must be read from telemetry at run
        time rather than assumed to be zero.
    :param initial_yaw: the drone's heading at mission start, used only to pick
        the first station's sweep start.
    :param stations_ned: an explicit station list, each `(north, east)` or
        `(north, east, altitude)` RELATIVE to `origin_ned`. Default: the
        take-off-anchored square. A per-station altitude is what lets a
        two-ring pattern fly its outer ring higher; when omitted, `altitude_m`
        applies to every station. See `pattern_stations`.
    :param headings_fn: `f(station_ned_relative) -> [heading]`, for patterns
        whose headings depend on where the station is (an inward-aiming orbit).
        Default: the same `headings` at every station.
    :param wrap_guard: refuse headings on the +/-180 discontinuity. ON by
        default, and it must STAY on for anything that will be flown --
        `is_at_heading` never wraps, so such a target hangs the sortie forever.
        Turn it OFF only to DESCRIBE a pattern nothing will fly: a dense
        simulated ring (the continuous Unity fixture aims 880 stations inward,
        so some of them necessarily point due south) has no yaw controller to
        hang. The mission itself always takes the default.
    :returns: list of `Station`, each with an absolute NED target, the yaw to
        arrive on, and its ordered `Capture` list.
    """
    if side_m <= 0:
        raise ValueError("side_m must be positive")
    if altitude_m <= 0:
        raise ValueError("altitude_m must be positive")
    if headings_fn is None and wrap_guard:
        # Only a FIXED heading set can be checked up front; a computed one is
        # checked per station below, once it exists.
        bad = [h for h in headings if not is_wrap_safe(h)]
        if bad:
            raise ValueError(
                "headings %s sit on the +/-180 discontinuity; is_at_heading "
                "cannot converge on them (see HEADINGS_8)" % (bad,))

    o_n, o_e, o_d = (float(v) for v in origin_ned)
    if stations_ned is None:
        stations_ned = [(dn, de) for dn, de in square_corners(side_m)]

    stations, yaw_cursor = [], float(initial_yaw)
    for idx, st in enumerate(stations_ned):
        dn, de = float(st[0]), float(st[1])
        alt = float(st[2]) if len(st) > 2 else float(altitude_m)
        if alt <= 0:
            raise ValueError("station %d has altitude %g; must be positive"
                             % (idx, alt))
        hs = list(headings) if headings_fn is None else list(headings_fn((dn, de)))
        if not hs:
            raise ValueError("station %d has no headings" % idx)
        # Computed headings can land on the discontinuity even when the pattern
        # looks fine on paper — an inward-aiming station due south of the target
        # commands ~180, which is exactly the target `is_at_heading` spins on
        # forever. Refuse here rather than in the air.
        bad = [h for h in hs if not is_wrap_safe(h)] if wrap_guard else []
        if bad:
            raise ValueError(
                "station %d heading(s) %s sit on the +/-180 discontinuity; "
                "is_at_heading cannot converge on them. NARROW the aim fan "
                "(see max_safe_fan_deg) or re-phase the ring." % (idx, bad))
        ordered = order_headings(hs, yaw_cursor)
        stations.append(Station(
            corner=idx,
            ned=(o_n + dn, o_e + de, o_d - alt),
            arrival_yaw=ordered[0],
            captures=[Capture(idx, h, format_tag(idx, h)) for h in ordered],
        ))
        yaw_cursor = ordered[-1]
    return stations


def plan_summary(stations):
    """One human-readable line per station, for the mission log."""
    lines = []
    for s in stations:
        lines.append("corner %d  N%+7.2f E%+7.2f D%+7.2f  arrive yaw %+7.1f  "
                     "headings %s"
                     % (s.corner, s.ned[0], s.ned[1], s.ned[2], s.arrival_yaw,
                        ",".join("%+.1f" % c.heading for c in s.captures)))
    lines.append("%d stations, %d captures"
                 % (len(stations), sum(len(s.captures) for s in stations)))
    return lines


# --------------------------------------------------------------------------- #
#  Fence
# --------------------------------------------------------------------------- #

def fence_bounds(origin_ned=(0.0, 0.0, 0.0), side_m=10.0, altitude_m=10.0,
                 margin_m=2.0, ceiling_margin_m=3.0, ground_margin_m=1.0,
                 stations=None):
    """Arguments for `RectLocalFence`, in its declared order
    ``(north_lower, north_upper, east_lower, east_upper, down_lower, down_upper)``.

    Two traps this exists to avoid:

    * The fence is in ABSOLUTE local NED, but the square is defined relative to
      where the drone started, and the EKF origin is somewhere else entirely. A
      fence built around (0, 0) would reject every waypoint.
    * `down` is negative-up and the class asserts ``lower < upper``, so the
      *ceiling* is the lower bound. The ground must stay INSIDE the box or
      takeoff and landing setpoints fall outside it, hence `ground_margin_m`.

    :param stations: the planned `Station` list. Pass it for ANY pattern that
        is not the take-off-anchored square: the box is then the bounding box
        of every station AND the take-off point, which an orbit around a
        target 10 m away needs and the square formula cannot express — it
        would put half the ring outside the fence and reject those waypoints
        in the air. Omitted, the original square arithmetic is used, which the
        station form reproduces exactly for that case.
    """
    o_n, o_e, o_d = (float(v) for v in origin_ned)
    m = float(margin_m)
    if stations is None:
        s = float(side_m)
        return (o_n - m, o_n + s + m,
                o_e - m, o_e + s + m,
                o_d - float(altitude_m) - float(ceiling_margin_m),
                o_d + float(ground_margin_m))

    if not stations:
        raise ValueError("fence_bounds: empty station list")
    ns = [o_n] + [float(s.ned[0]) for s in stations]
    es = [o_e] + [float(s.ned[1]) for s in stations]
    # NED down is negative-up, so the HIGHEST station is the most negative.
    top = min(float(s.ned[2]) for s in stations)
    return (min(ns) - m, max(ns) + m,
            min(es) - m, max(es) + m,
            top - float(ceiling_margin_m),
            o_d + float(ground_margin_m))


# --------------------------------------------------------------------------- #
#  Pose priors
# --------------------------------------------------------------------------- #

def _rot_body_to_ned(roll_deg, pitch_deg, yaw_deg):
    """Aerospace ZYX (yaw-pitch-roll) rotation from FRD body to NED, row-major."""
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def payload_position_ned(fc_ned, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
                         offset_fc_m=PAYLOAD_OFFSET_FC_M):
    """Where the LiDAR/camera actually was, given where the FC says it was.

    The lever arm rotates with the airframe, so at 10 deg of pitch the 25 cm drop
    is no longer 25 cm of altitude — it is 4 cm of horizontal offset as well.
    Level flight is the special case, not the rule.
    """
    R = _rot_body_to_ned(roll_deg, pitch_deg, yaw_deg)
    return tuple(float(fc_ned[i]) + sum(R[i][j] * offset_fc_m[j] for j in range(3))
                 for i in range(3))


# --------------------------------------------------------------------------- #
#  Geodesy and placement — where is the OBJECT, in words an operator can use
# --------------------------------------------------------------------------- #
#
# A target-centred pattern is built around something that cannot be moved
# precisely (a parked truck), so the operator either paces an offset from the
# take-off spot or marks the object with the drone's own receiver. Both end in
# a north/east offset in metres, and both need to be READ BACK as a distance
# and a true bearing, because that is the only form a person can check on a
# field without a laptop.

R_EARTH_M = 6378137.0          # WGS-84 equatorial radius; same as enu_from_llh


def ned_from_llh(lat_deg, lon_deg, lat0_deg, lon0_deg):
    """(north_m, east_m) of (lat, lon) as seen from (lat0, lon0).

    Flat tangent plane: sub-millimetre inside a hundred metres, and nothing
    the mission plans is further away than that. The same approximation
    `ulog_pose_noise.enu_from_llh` uses for the reconstruction poses, kept
    here in stdlib so the mission needs no numpy on the ground station.
    """
    dlat = math.radians(float(lat_deg) - float(lat0_deg))
    dlon = math.radians(float(lon_deg) - float(lon0_deg))
    return (R_EARTH_M * dlat,
            R_EARTH_M * math.cos(math.radians(float(lat0_deg))) * dlon)


def llh_offset(lat0_deg, lon0_deg, north_m, east_m):
    """The inverse: (lat, lon) `north_m`/`east_m` away from (lat0, lon0)."""
    dlat = float(north_m) / R_EARTH_M
    dlon = float(east_m) / (R_EARTH_M * math.cos(math.radians(float(lat0_deg))))
    return (float(lat0_deg) + math.degrees(dlat),
            float(lon0_deg) + math.degrees(dlon))


def bearing_deg(north_m, east_m):
    """True bearing 0..360, clockwise from north. 0 for a zero vector."""
    if math.hypot(float(north_m), float(east_m)) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(float(east_m), float(north_m))) % 360.0


def ascii_map(origin_ned, target_ned, stations, cell_m=1.0, pad_m=1.0,
              max_cells=60 * 120):
    """A top-down sketch, north up, east right, one character per `cell_m`.

    `D` is the take-off point, `T` the target, stations are numbered in flight
    order. Later symbols overwrite earlier ones so `D` always shows — if the
    drone stands ON the target the map says so rather than hiding it.
    """
    pts = ([(float(origin_ned[0]), float(origin_ned[1])),
            (float(target_ned[0]), float(target_ned[1]))]
           + [(float(s.ned[0]), float(s.ned[1])) for s in stations])
    n_max = max(p[0] for p in pts) + pad_m
    n_min = min(p[0] for p in pts) - pad_m
    e_max = max(p[1] for p in pts) + pad_m
    e_min = min(p[1] for p in pts) - pad_m
    cell = float(cell_m)
    while ((n_max - n_min) / cell + 1) * ((e_max - e_min) / cell + 1) > max_cells:
        cell *= 2.0                      # a wide pattern must not flood the log
    rows = int(round((n_max - n_min) / cell)) + 1
    cols = int(round((e_max - e_min) / cell)) + 1
    grid = [["." for _ in range(cols)] for _ in range(rows)]

    def put(n, e, ch):
        r = min(rows - 1, max(0, int(round((n_max - n) / cell))))
        c = min(cols - 1, max(0, int(round((e - e_min) / cell))))
        grid[r][c] = ch

    # Station symbols follow the tag numbering (c0_h..., c1_h...) so the map,
    # the card and the capture files all name the same stop the same way.
    symbols = "0123456789abcdefghijklmnopqrstuvwxyz"
    for s in stations:
        put(s.ned[0], s.ned[1], symbols[int(s.corner) % len(symbols)])
    put(target_ned[0], target_ned[1], "T")
    put(origin_ned[0], origin_ned[1], "D")
    head = ("N up, E right, 1 char = %g m   D take-off   T target   "
            "0-%d stations as tagged" % (cell, max(0, len(stations) - 1)))
    return [head] + ["".join(r) for r in grid]


def placement_card(origin_ned, target_ned, stations, fence=None,
                   target_global=None, marks=(), altitude_delta_m=None,
                   fence_margin_m=2.0, ceiling_margin_m=3.0, with_map=True):
    """Lines telling the operator where the object is and what the ring needs.

    Everything is given TWICE — from the take-off point (where the operator is
    standing) and from the target (what the operator can see) — because on a
    field the drone is at one and the truck at the other, and a bearing is
    only checkable from where you stand.
    """
    o = [float(v) for v in origin_ned]
    t = [float(v) for v in target_ned]
    dn, de = t[0] - o[0], t[1] - o[1]
    out = ["PLACEMENT",
           "  target   %6.1f m at bearing %03.0f (true) from the take-off point"
           "   N%+.1f E%+.1f" % (math.hypot(dn, de), bearing_deg(dn, de), dn, de)]
    if target_global is not None:
        out.append("           lat %.7f  lon %.7f" % (target_global[0],
                                                     target_global[1]))
    if marks:
        out.append("           from mark%s %s" % ("s" if len(marks) > 1 else "",
                                                  ", ".join(marks)))
    if altitude_delta_m is not None and abs(altitude_delta_m) > 2.0:
        out.append("  ⚠ the mark sits %.1f m %s the take-off point; altitude is "
                   "above TAKE-OFF, so the look line lands %s the target"
                   % (abs(altitude_delta_m),
                      "above" if altitude_delta_m > 0 else "below",
                      "short of" if altitude_delta_m > 0 else "past"))
    r_max, alt_max = 0.0, 0.0
    for s in stations:
        sn, se, sd = (float(v) for v in s.ned)
        ft_n, ft_e = sn - t[0], se - t[1]
        fo_n, fo_e = sn - o[0], se - o[1]
        alt = o[2] - sd
        r_max, alt_max = max(r_max, math.hypot(ft_n, ft_e)), max(alt_max, alt)
        out.append("  st %-2d  from target %5.1f m brg %03.0f  | from take-off "
                   "%5.1f m brg %03.0f  | alt %4.1f m"
                   % (s.corner, math.hypot(ft_n, ft_e), bearing_deg(ft_n, ft_e),
                      math.hypot(fo_n, fo_e), bearing_deg(fo_n, fo_e), alt))
    out.append("  clear    keep %.0f m around the target free of anything taller "
               "than the aircraft flies, up to %.0f m"
               % (r_max + fence_margin_m, alt_max + ceiling_margin_m))
    if fence is not None:
        out.append("  fence    N[%.1f,%.1f] E[%.1f,%.1f] D[%.1f,%.1f]"
                   % tuple(float(v) for v in fence))
    if with_map:
        out += ["  " + line for line in ascii_map(o, t, stations)]
    return out
