# DroneManager missions

Missions for [DroneManager](https://dronemanager.readthedocs.io) that fly the
LiveGS capture rig. They live in this repo because they are part of the capture
pipeline, but they run inside DroneManager on the ground station, not inside
LiveGS.

| file | what |
|---|---|
| `squarecapture.py` | the mission: square pattern, yaw sweep, rig capture at every stop |
| `squarecapture_plan.py` | the geometry — pure stdlib, unit-tested, no DroneManager import |

Tested by `tests/test_squarecapture_plan.py` (62) and
`tests/test_squarecapture_mission.py` (65, DroneManager stubbed, capture daemon
real).

> **The next test flight is `dry01` — read [that section](#the-next-test-flight-dry01--compass-orbit-and-a-continuous-recording)
> first. It is written for the rig as it is on 2026-09-17: ZED X instead of
> the D455, a 1 m GNSS-compass baseline, no extrinsic calibration yet, and a
> daemon that records the whole flight.**

The plan it generates, for the defaults:

```
corner 0  N  +0.00 E  +0.00 D -10.00  arrive yaw   -22.5  headings -22.5,+22.5,+67.5,+112.5,+157.5,-157.5,-112.5,-67.5
corner 1  N +10.00 E  +0.00 D -10.00  arrive yaw   -67.5  headings -67.5,-22.5,+22.5,+67.5,+112.5,+157.5,-157.5,-112.5
corner 2  N +10.00 E +10.00 D -10.00  arrive yaw  -112.5  headings -112.5,-67.5,-22.5,+22.5,+67.5,+112.5,+157.5,-157.5
corner 3  N  +0.00 E +10.00 D -10.00  arrive yaw  -157.5  headings -157.5,-112.5,-67.5,-22.5,+22.5,+67.5,+112.5,+157.5
4 stations, 32 captures
```

Each sweep runs 315° in one direction, and the next corner starts on the heading
the last one ended on — so the transit legs need no rotation at all.

## Deploy

**On this laptop the mission is deployed INTO the DroneManager repo, on its
`LiveGS` branch** (`C:\Users\Bake\Projects\DroneManager`, installed editable,
so `src/dronemanager/missions/` is what `dm` runs). That branch is main plus
these four files; the source of truth stays here and the branch receives
copies. From WSL:

```bash
DM=/mnt/c/Users/Bake/Projects/DroneManager
git -C $DM branch --show-current                      # must print LiveGS
cp Scripts/dm_missions/squarecapture.py Scripts/dm_missions/squarecapture_plan.py \
   Scripts/dm_missions/README.md Scripts/drone_capture_client.py $DM/src/dronemanager/missions/
git -C $DM add src/dronemanager/missions/squarecapture.py \
   src/dronemanager/missions/squarecapture_plan.py \
   src/dronemanager/missions/README.md src/dronemanager/missions/drone_capture_client.py
git -C $DM commit -m "missions: sync squarecapture from LiveGS <commit>"
```

⚠ Add ONLY those four paths. The rest of that working tree is CRLF-only churn
in docs that must not be swept into a commit. ⚠ The copies there were STALE
(pre-pattern) until 2026-09-07 — check with `diff --strip-trailing-cr` after
copying, and re-copy after every change to these files.

The alternative is DroneManager's user directory,
`Documents/DroneManager/missions/` (`/mnt/c/Users/<you>/Documents/DroneManager/missions/`
from WSL), which it also scans and which is empty on this laptop:

```powershell
$m = "$env:USERPROFILE\Documents\DroneManager\missions"
mkdir $m -Force
copy Scripts\dm_missions\squarecapture.py      $m
copy Scripts\dm_missions\squarecapture_plan.py $m
copy Scripts\drone_capture_client.py           $m
```

The mission puts its own directory on `sys.path` at import time, which is what
makes the two siblings importable — DroneManager loads user missions by file
path, so their directory is not otherwise importable. All three files must
travel together.

`squarecapture_plan` and `drone_capture_client` will show up in `mission-load`'s
completion list because it lists every `.py` in the directory. They are not
missions and loading them fails harmlessly.

## The next test flight: `dry01` — compass, orbit and a continuous recording

Three questions, one sortie, no truck. It flies the orbit preset around a
cone, records EVERYTHING from 30 s before arming to after landing, and leaves
a log that answers whether the 1 m compass baseline did what it should.

What changed on the rig since flight04, and what it means here:

| change | consequence for this flight |
|---|---|
| **ZED X** replaced the D455 | the daemon is a different program (`drone_capture.py` opens the ZED SDK, rectified LEFT view 1920x1200@15); `gain` is now the ZED's analog gain in **dB (1..7)**, `exposure` is still ms; the service runs from `~/calibvenv` |
| **no extrinsic** for the ZED yet | every session says `T_lidar_camera_is_seed: true` and `capture_check` warns; `depth/` maps are provisional. **This flight validates geometry, sequence and the recording — NOT a reconstruction.** Calibrate first (`Scripts/Calibration/README.md`) before quoting any PSNR |
| **1 m GNSS-compass baseline** | heading accuracy should fall from **2.0 deg to ~0.57 deg** (it scales as 9.9 mm / baseline). But `EKF2_GPS_POS_X/Y/Z` read **0/0/0** and `SENS_GPS_MASK=7` still blends both antennas, now a metre apart — set them BEFORE flying (below) |
| **continuous recording** | `sq-orbit` now records every sweep (organised grid + per-column stamps), every frame (JPEG) and the Ouster IMU for the whole flight: `record=yes`, `prearm=30` by default. ~22 MB/s, ~15-20 GB per sortie |
| **abort guard** | three consecutive refusals with one reason (each retried once) END the sortie and LAND — flight04's 64 x `no_fix` cannot happen again |

### 0. On the workstation, before driving

```bash
# the daemon on the drone must be the ZED build (the D455 one crash-loops):
./Scripts/drone_capture_deploy.sh                      # copies + md5
ssh dronetrekkers@192.168.1.55 'sudo cp ~/dronecap/drone_capture.service /etc/systemd/system/drone-capture.service && sudo systemctl daemon-reload && sudo systemctl restart drone-capture'
python Scripts/drone_capture_client.py --host 192.168.1.55 status     # camera.frames > 0, recording.active false
# the four mission files into DroneManager's LiveGS branch (see Deploy above)
```

`sq_marks.json` on the ground station still holds a SITL-era `truck` mark
in Zurich: `sq-unmark truck` before marking anything real.

### 1. Drone: the compass, and the two parameters that are wrong

1. **Tape-measure both antennas** relative to the FC (body FRD: x forward,
   y right, z DOWN). Write them into `Scripts/flight_poses.py`
   (`ANTENNA_TO_FC_M`, and the comment block above it) — `flight_report.py`
   compares the flown baseline against those numbers, and still carries
   the 0.275 m airframe.
2. **Set the lever arm and stop blending**, on the ground, disarmed:
   ```bash
   ssh dronetrekkers@192.168.1.55 '~/TestMAVLink/.venv/bin/python -' < Scripts/fc_params.py read
   # then, with the measured antenna-0 position (example values!):
   ssh dronetrekkers@192.168.1.55 '~/TestMAVLink/.venv/bin/python - set EKF2_GPS_POS_X=0.15 EKF2_GPS_POS_Y=0.0 EKF2_GPS_POS_Z=-0.08 SENS_GPS_MASK=0 SENS_GPS_PRIME=0' < Scripts/fc_params.py
   ```
   `set` reads every value back and says MISMATCH if one did not take.
   `GPS_YAW_OFFSET` stays 0 only if both antennas are still on the fore-aft
   line; a sideways boom needs the measured angle. Reboot the FC and `read`
   again.
3. **Outside, with a fix**: `gps_check.py` now prints the GNSS heading and
   its accuracy (`hdg_acc`) next to the fix. The number to write down:
   **accuracy ~0.55-0.60 deg** means the baseline did its job; ~2 deg means
   the receivers are not using the new geometry (check `nsh.py "gps
   status"`: Main must show a numeric heading and ~30 Hz RTCM injection,
   Secondary ~6 Hz from the ground base; `heading: nan` = do not fly).
4. **Log a static segment**: leave the aircraft on the ground, RTK fixed,
   for a few minutes with logging on (SDLOG_MODE 0 logs from arming — arm
   without taking off, or set SDLOG_MODE 1 for boot-to-shutdown logging for
   this test). Post-flight, `dgps_flight_analysis.py <log.ulg> --min-fix=3`
   reports `reported_acc_deg`, the measured `baseline_m` and
   `implied_perp_error_mm` — if the perpendicular error is still ~10 mm the
   compass scaled; if it grew, the boom flexes or the antenna ground planes
   are the problem.

### 2. Mission: the cone orbit, recorded

```
connect kirk udp://:14561
mission-load squarecapture --name sq
sq-add kirk
sq-unmark truck                  # the SITL mark
sq-mark cone                     # drone standing at the cone
sq-marks
sq-orbit-plan --target_mark cone # placement card: 8 stations 7 m out / 7 m up, fence
sq-check                         # daemon (ZED frames, LiDAR, pose link) + FC fix
sq-orbit dry01 --target_mark cone
sq-status
```

`sq-orbit` meters the exposure on site (`gain` 1.0 dB analog), opens the
session, **starts the recording, holds 30 s still** (do not touch the
aircraft — this is the gyro-bias segment), arms, flies 64 captures at one
scan each, returns over the take-off point, lands, stops the recording,
closes the session. `--prearm 0` skips the hold; `--record no` flies shots
only. `sq-abort` stops the recording, then the session, then lands.

If the guard fires you will see `SORTIE ABORTED: 3 consecutive captures
refused with 'no_fix'` and the aircraft returning to land on its own;
the record says `outcome: aborted`.

### 3. Daemon: check before, check after

Before: `drone_capture_client.py --host 192.168.1.55 preview` and read
`saturated_pct` and `max_pixel` (white is 255 on the ZED); `status` should
show `camera.serial 44864681`, `lidar.partial_dropped` ≥ 1 and
`recording.active false`. A bench rehearsal of the recorder: `start bench
--record`, wait 20 s, `stop`, then `capture_check.py` on the pulled session.

After, on site:

```bash
python Scripts/drone_capture_client.py --host 192.168.1.55 pull dry01 --dest recordings/dry01
cp ~/Documents/DroneManager/sq_sessions/dry01.mission.json recordings/dry01/
python Scripts/capture_check.py recordings/dry01 --expect 64 --corners 8 --headings 8
```

`capture_check` now also grades the recording: sweep count and rate, gaps,
**per-column timestamps non-zero** (they were identically zero on every
flight until 2026-09-17 — an SDK property was being called as a method and
the exception was swallowed), column stamps inside the npz, frame files and
rate, IMU rate (~100 Hz, rad/s), recorder drops. Also pull the `.ulg` off
the FC (QGC or the SD card) into the same directory — `fc_imu.py`,
`flight_report.py` and `dgps_flight_analysis.py` all glob for it there.

What the data is for: the recording is the real-rig counterpart of
`recordings/unity-truck-C2`. The consumer — a real-session branch in
`traj.load_recording` plus per-sweep prior poses from the tlog/ulg — is the
first task once this data exists; `rtk_lo.py`, `run_kiss.py` and `traj.py`
then run on it unchanged.

## Run — the chosen sortie: an aimed orbit around the truck

The pattern study (`eval/Flight/pattern-study.md`) picked an aimed orbit at
radius 7 m / altitude 7 m, 8 stations, a 15° fan, one scan per capture. It is
one command, and the only thing it needs to know is WHERE THE TRUCK IS.

**Tell it by marking the truck with the drone.** Park the truck wherever is
convenient. Carry the drone (powered, with a fix) to the truck and mark it;
walk back to the take-off spot and read the card:

```
connect Romulus udp://:14561
mission-load squarecapture --name sq
sq-add Romulus
sq-mark truck                  # drone standing at the truck's mid-side
sq-marks                       # every mark as distance + true bearing from HERE
sq-orbit-plan --target_mark truck    # the schedule, the fence, the placement card
sq-check                       # preflight: daemon, sensors, GPS
sq-orbit flight04 --target_mark truck   # arm, take off, 64 captures, return, land
sq-status
sq-abort                       # stop and land where we are
```

Metre-level is enough — a metre of target error is ~8° off frame centre at
7 m and the 6 m truck stays in frame — so one mark on the ground beside the
truck's middle is within tolerance. For the centre exactly, mark the front and
rear bumpers and pass both: `--target_mark front,rear` averages them. A mark
is the drone's own receiver, so the OFFSET between mark and take-off cancels
the common GNSS error; "not RTK fixed" at mark time is a warning, not a gate.

**Or tell it as an offset.** `sq-orbit flight04 --target_n 10 --target_e 0`
puts the target 10 m due (true) north of the take-off spot;
`sq-orbit-plan --target_n 10` prints where that is so the truck can be placed
there first. Pacing a bearing by hand is the weaker option.

Both write a record the daemon never sees —
`~/Documents/DroneManager/sq_sessions/<session>.mission.json` with the target
(GNSS and NED), the marks, the plan, the fence and every commanded pose. Marks
live in `~/Documents/DroneManager/sq_marks.json`; every save logs the absolute
path, because a OneDrive-redirected Documents folder is the one way they can
"disappear".

Then pull the session off the drone, copy the record beside it, and check:

```bash
python Scripts/drone_capture_client.py --host 192.168.1.55 pull flight04 \
    --dest recordings/flight04
cp ~/Documents/DroneManager/sq_sessions/flight04.mission.json recordings/flight04/
python Scripts/capture_check.py recordings/flight04 --expect 64
```

The original take-off-anchored square is still `sq-run flight01` (32
captures, the pattern flight01-03 flew); `sq-plan` prints its schedule.

### Options

`sq-run <session> [--side 10] [--altitude 10] [--scans 10] [--settle 2]
[--exposure <ms>] [--gain 1.0] [--gate warn] [--record yes] [--prearm 30]`

`--exposure` is milliseconds on the ZED X (its `EXPOSURE_TIME`); omitted, the
sortie meters on site. `--gain` is the analog gain in dB, 1.0..7.0 on this
sensor; 1.0 is the least noise and daylight can afford it. `--record yes`
records the whole flight continuously (sweeps with per-column stamps,
frames, Ouster IMU) from before arming to after landing; `--prearm` is the
stationary hold, in seconds, between starting the recording and arming
(30 by default; the gyro-bias walk is unmeasurable without it).

The sortie **aborts and lands** after `max_consecutive_misses` (3) captures
in a row are refused with the same reason, each already retried once —
flight04 flew all 64 as `no_fix` before this existed.

`--gate` is the daemon's motion gate: `warn` records the gyro peak and captures
anyway, `block` refuses to capture while moving, `off` ignores it. `warn` is the
default because hover vibration on this airframe is uncharted and a `block` gate
could refuse all 32 captures.

`sq-link <host> [--user u] [--port p] [--direct yes]` points the mission at a
different daemon. `--direct yes` is plain TCP instead of `ssh -W`, used only for
the SITL rehearsal.

### Patterns other than the square

`sq-run <session> --pattern orbit --anchor centre --aim yes --radius 7
--altitude 7 --stations 8 --target_n 10` flies a ring around an object 10 m
north of the take-off point. Patterns: `square`, `square+centre`, `orbit`,
`tworings` (`--ring2_radius 12 --ring2_altitude 12 --ring2_stations 6`).

Three rules, all of which refuse the sortie rather than misfly it:

- **`--radius` must equal `--altitude`.** The payload is bolted 45° nose-down
  and does not gimbal, so the look line meets the ground at a horizontal
  distance equal to the altitude. Any other pair frames the object off-centre;
  two Unity recordings were lost to this before the checker learned to say so.
- **`--target_n` / `--target_e` are where the OBJECT is**, as an offset in
  metres from the take-off point. They default to (0, 0), which is right for a
  rehearsal over empty grass and wrong for anything you cannot take off
  underneath. The aim bearing is computed to the target, not to the take-off
  point.
- **`--fan` has a hard ceiling** (`squarecapture_plan.max_safe_fan_deg`):
  17.5° on a half-step 8-station ring, 40° on a square. Wider sweeps a heading
  through ±180, which `is_at_heading` compares without wrapping and never
  converges on. 15° is safe everywhere and is the default.

`--anchor corner` is the original take-off-anchored square that `flight01`
through `flight03` flew, and stays the default; every other pattern is built
AROUND the target and needs `--anchor centre`. A centred pattern widens the
geofence to the bounding box of its stations plus the take-off point, and
returns to the take-off point rather than to station 0.

Check a plan before flying it: `sq-plan --pattern orbit --anchor centre
--aim yes --radius 7 --altitude 7 --target_n 10` prints every station, the fan
limit and the fence.

⚠ **The flags use UNDERSCORES, not hyphens** — `--target_n`, `--ring2_radius`.
DroneManager builds the command line from the parameter names verbatim
(`app.py`: `arg_name = f"--{name}"`), so `--target-n` is silently not a match,
the same way `exposure=0.4` was not a match for `--exposure 0.4` and cost
`flight02`. And `--aim` takes a VALUE (`--aim yes`): the CLI accepts only
`str`/`float`/`int`, so there are no bare boolean flags.

## Rehearse before flying

The capture side needs no drone and the flight side needs no sensors, so both
halves can be exercised on the ground.

**SITL rehearsal** — the whole sortie, no hardware at all:

```bash
python Scripts/mock_capture_daemon.py --port 5757 --fail-every 5
```
```
connect Romulus udp://:14540          # PX4 SITL
mission-load squarecapture --name sq
sq-add Romulus
sq-link 127.0.0.1 --direct yes --port 5757
sq-mark cone                          # wherever the SITL vehicle sits
sq-orbit-plan --target_mark cone
sq-orbit test01 --target_mark cone
```

`--fail-every 5` refuses every fifth shot so the retry-and-continue path runs
rather than being assumed. The mock writes empty placeholder files: nothing
downstream can train on them. Expect a warning that the target is the take-off
point — in SITL the vehicle did not move between marking and flying, which is
exactly the case the warning names.

**The dry flight** — the orbit for real, around a cone instead of the truck.
Put a cone on the field, carry the drone to it, `sq-mark cone`, walk back,
`sq-orbit-plan --target_mark cone`, `sq-orbit dry01 --target_mark cone`. The
placement procedure is rehearsed, not just the flight: marking a cone is
marking a truck.

**Bench dry run** — the real daemon and real sensors, props off, no flight:

```
sq-dryrun bench02
```

## Prerequisites

1. **A PASSPHRASE-LESS SSH key on the ground station.** The mission drives the
   daemon over `ssh -W` as a non-interactive subprocess, so there is nothing to
   type a passphrase into. A passphrase-protected key fails in a way that is
   easy to misread: verbose ssh prints `Server accepts key` and then moves on to
   the next identity, because it can authorise the key but cannot *sign* with
   it. The mission just reports "Capture daemon unreachable".
   ```powershell
   ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_drone -N '""' -C dronemanager-capture
   type $env:USERPROFILE\.ssh\id_drone.pub | ssh dronetrekkers@192.168.1.55 "cat >> .ssh/authorized_keys"
   ```
   Then pin it in `%USERPROFILE%\.ssh\config`, or a passphrase-protected key
   earlier in the search order will be offered instead:
   ```
   Host 192.168.1.55
     HostName 192.168.1.55
     User dronetrekkers
     IdentityFile ~/.ssh/id_drone
     IdentitiesOnly yes
   ```
   ⚠ Append the public key with the CR stripped — a `.pub` copied from a Windows
   file carries CRLF.
2. **Use port 14561, not 14550.** `connect Romulus udp://:14561`. QGroundControl
   defaults to listening on 14550 and DroneManager has no MAVLink router
   (`mavpassthrough.py:14`), so the two cannot share a port — whichever starts
   second fails to bind and reports the vehicle unreachable, with no hint that a
   port conflict is the cause. mavproxy now unicasts to both (14550 for QGC,
   14561 for DroneManager), so QGC can stay up as an independent safety monitor.
3. **A GPS fix.** ⚠ `SYS_STATUS`'s GPS present bit is **not** a reliable test
   for whether a receiver exists — measured False on PX4 v1.17 while both
   receivers were connected and streaming UBX at over 1 kB/s. The authority is
   the flight controller itself, `nsh> gps status`: `status: OK, port:
   /dev/ttyS0, baudrate: 115200` with a non-zero `rate reading` means the
   receiver is present and talking, and `satellites_used: 0` then means sky view
   and nothing else. `Scripts/gps_check.py` reports the fix over MAVLink and
   points at `gps status` rather than guessing at the cause.
4. **Offboard-loss failsafe.** Setpoints now cross Wi-Fi and mavproxy before
   reaching the FC. Check `COM_OF_LOSS_T` and set `COM_OBL_RC_ACT` to Hold or
   Position so a dropout produces a hover, not a flyaway. Keep the RC
   transmitter live: a mode switch overrides offboard and nothing in software
   can countermand it.

## Things that will bite

- **`is_at_heading` does not wrap.** `Drone.is_at_heading` (drone.py:399)
  compares `abs(cur - target)` after normalising the target into (-180, 180].
  A target of exactly 180 becomes -180, and a drone at +179.9 computes 359.8 and
  never converges — `fly_to` and `yaw_to` both hang forever. `HEADINGS_8` is
  offset by 22.5 deg to stay clear of it, and `capture_plan` refuses a heading
  set that is not.
- **`yaw_to` divides by a step count derived from the yaw delta**, so a delta of
  exactly zero is a `ZeroDivisionError`, not a no-op. The mission skips the call
  under 0.5 deg.
- **`yaw_to` deactivates the path follower** and streams its own setpoints. Pass
  `local=` explicitly or it re-samples a noisy `position_ned` as the hold point
  and the station drifts across the sweep.
- **`Drone.set_fence` is broken** (drone.py:420): it passes the logger as the
  first positional argument, which lands in `north_lower`. Use
  `dm.set_fence(name, RectLocalFence(...))`, which assigns an instance.
- **Manager actions return exceptions, they do not raise them.**
  `_multiple_drone_action` uses `gather(return_exceptions=True)` and returns a
  list; a name-lookup miss returns `None` outright.
- **Local NED is the EKF origin, not the takeoff point.** The square is anchored
  to `position_ned` read at run time; a fence built around (0, 0) would reject
  every waypoint.
- **The payload hangs 25 cm below the FC.** The daemon stamps FC pose. Anything
  consuming those priors must apply `PAYLOAD_OFFSET_FC_M` — it is a systematic
  offset, identical on every capture, so it will not surface as registration
  noise.
