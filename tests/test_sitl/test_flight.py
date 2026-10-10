"""Pipeline tests for flying a simulated PX4 drone through DroneManager.

The tests only use the public DroneManager API, like a script or the terminal application would. Each test starts with
a disarmed drone on the ground, see the ``sitl_dm`` fixture.
"""
import asyncio

import numpy as np
import pytest

from dronemanager.core import DroneManager
from dronemanager.drone import FlightMode
from dronemanager.utils import relative_gps

from sitl import SITL_DRONE, wait_until

pytestmark = pytest.mark.sitl

ALTITUDE = 3.0
"""Takeoff altitude for the tests, in meters."""
POSITION_TOLERANCE = 0.5
"""How close the drone must get to a target, in meters. Looser than the navigation tolerance, as the drone keeps
moving slightly after it reports reaching a target."""

LANDING_BUG = ("Offboard landing reports 'Landed!' as soon as the descent briefly stalls, even mid-air, so land() and "
               "stop() can return while the drone still hovers. Fixed in a follow-up PR.")
MOVE_WITHOUT_YAW_BUG = ("move() without a yaw crashes: the yaw is added to the current heading even if it is None. "
                        "Fixed in a follow-up PR.")
YAW_RATE_BUG = ("yaw_to() without a yaw rate crashes: DroneManager passes None on, which replaces the drone's default "
                "rate. Fixed in a follow-up PR.")
HEADING_BUG = "The heading telemetry is never subscribed to, so drone.heading stays NaN. Fixed in a follow-up PR."


async def _takeoff(dm: DroneManager, altitude: float = ALTITUDE):
    """Arm the drone and take off.

    Args:
        dm: The DroneManager.
        altitude: Takeoff altitude in meters.
    """
    drone = dm.drones[SITL_DRONE]
    # The takeoff altitude is relative to the current position. The origin of the local coordinates isn't exactly at
    # ground level, so the climb is checked, not the absolute height.
    ground = drone.position_ned[2]
    assert await asyncio.wait_for(dm.arm(SITL_DRONE), 30) == [True]
    assert drone.is_armed
    assert await asyncio.wait_for(dm.takeoff(SITL_DRONE, altitude=altitude), 60) == [True]
    assert drone.in_air
    assert ground - drone.position_ned[2] == pytest.approx(altitude, abs=POSITION_TOLERANCE)


async def _land(dm: DroneManager):
    """Land the drone with PX4's land mode and wait until it disarms by itself.

    Tests that aren't about landing use PX4's own landing, which isn't affected by :py:data:`LANDING_BUG`.

    Args:
        dm: The DroneManager.
    """
    drone = dm.drones[SITL_DRONE]
    await asyncio.wait_for(dm.change_flightmode(SITL_DRONE, "land"), 30)
    assert await wait_until(lambda: not drone.in_air, 60)
    assert await wait_until(lambda: not drone.is_armed, 30), "PX4 should disarm by itself after landing."


async def test_connection_and_telemetry(sitl_dm: DroneManager):
    """A connected drone reports its autopilot, parameters and telemetry.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    assert drone.is_connected
    assert drone.autopilot == "PX4"
    assert not drone.is_armed and not drone.in_air
    assert drone.fix_type.value >= 3
    assert drone.parameters_loaded
    assert "MPC_XY_VEL_MAX" in drone.drone_params.raw
    assert drone.drone_params.max_h_vel == pytest.approx(drone.drone_params.raw["MPC_XY_VEL_MAX"][0])
    assert np.all(np.isfinite(drone.position_ned)) and np.all(np.isfinite(drone.attitude))
    assert drone.position_global[0] != 0 and drone.position_global[1] != 0
    assert len(drone.batteries) > 0
    assert drone.flightmode != FlightMode.UNKNOWN


@pytest.mark.xfail(reason=HEADING_BUG, strict=True)
async def test_heading_telemetry(sitl_dm: DroneManager):
    """The drone reports its heading, matching the yaw of its attitude.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    assert await wait_until(lambda: np.isfinite(drone.heading), 10)
    assert (drone.heading - drone.attitude[2] + 180) % 360 - 180 == pytest.approx(0, abs=5)


async def test_takeoff_fly_move_yaw_land(sitl_dm: DroneManager):
    """The basic flight commands move the drone where they should.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    start = drone.position_ned.copy()
    await _takeoff(sitl_dm)

    target = np.array([start[0] + 5, start[1], -ALTITUDE])
    assert await asyncio.wait_for(sitl_dm.fly_to(SITL_DRONE, local=target, yaw=0), 60) == [True]
    assert drone.position_ned == pytest.approx(target, abs=POSITION_TOLERANCE)

    assert await asyncio.wait_for(sitl_dm.move(SITL_DRONE, offset=np.array([-5.0, 3.0, 0.0]), yaw=0.0,
                                               use_gps=False, schedule=False), 60) == [True]
    assert drone.position_ned == pytest.approx(target + np.array([-5, 3, 0]), abs=POSITION_TOLERANCE)

    assert await asyncio.wait_for(sitl_dm.yaw_to(SITL_DRONE, yaw=90.0, yaw_rate=30.0, schedule=False), 60) == [True]
    assert drone.attitude[2] == pytest.approx(90, abs=5)

    await _land(sitl_dm)


@pytest.mark.xfail(reason=YAW_RATE_BUG, strict=True)
async def test_yaw_to_with_default_rate(sitl_dm: DroneManager):
    """Yawing works without specifying a yaw rate.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    assert await asyncio.wait_for(sitl_dm.yaw_to(SITL_DRONE, yaw=-90.0, schedule=False), 60) == [True]
    assert drone.attitude[2] == pytest.approx(-90, abs=5)
    await _land(sitl_dm)


@pytest.mark.xfail(reason=MOVE_WITHOUT_YAW_BUG, strict=True)
async def test_move_without_yaw_keeps_heading(sitl_dm: DroneManager):
    """Moving without a yaw keeps the current heading.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    assert await asyncio.wait_for(sitl_dm.yaw_to(SITL_DRONE, yaw=45.0, yaw_rate=30.0, schedule=False), 60) == [True]
    start = drone.position_ned.copy()
    assert await asyncio.wait_for(sitl_dm.move(SITL_DRONE, offset=np.array([3.0, 0.0, 0.0]), use_gps=False,
                                               schedule=False), 60) == [True]
    assert drone.position_ned == pytest.approx(start + np.array([3, 0, 0]), abs=POSITION_TOLERANCE)
    assert drone.attitude[2] == pytest.approx(45, abs=5)
    await _land(sitl_dm)


@pytest.mark.xfail(reason=LANDING_BUG, strict=False)
async def test_land_returns_once_landed(sitl_dm: DroneManager):
    """DroneManager's landing only returns once the drone is on the ground.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    for _ in range(3):  # The bug doesn't show up on every landing
        await _takeoff(sitl_dm, altitude=2.0)
        await asyncio.wait_for(sitl_dm.land(SITL_DRONE), 90)
        assert not drone.in_air, "land() returned while the drone was still in the air."
        assert await wait_until(lambda: not drone.is_armed, 30)


async def test_gps_navigation(sitl_dm: DroneManager):
    """Flying to GPS coordinates works both in offboard mode and with PX4's own goto.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)

    north = np.array(relative_gps(drone.position_global[:3], [4.0, 0.0, 0.0]))
    assert await asyncio.wait_for(sitl_dm.fly_to(SITL_DRONE, gps=north, yaw=0), 60) == [True]
    assert drone.position_global[:2] == pytest.approx(north[:2], abs=1e-5)

    east = np.array(relative_gps(drone.position_global[:3], [0.0, 4.0, 0.0]))
    assert await asyncio.wait_for(sitl_dm.go_to(SITL_DRONE, gps=east, yaw=0, tol=POSITION_TOLERANCE, schedule=False),
                                  90) == [True]
    assert drone.flightmode == FlightMode.HOLD
    assert drone.position_global[:2] == pytest.approx(east[:2], abs=1e-5)

    await _land(sitl_dm)


async def test_scheduled_actions_run_in_order(sitl_dm: DroneManager):
    """Scheduled actions are queued and executed one after the other.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    start = drone.position_ned.copy()
    target = np.array([start[0], start[1] + 4, -ALTITUDE])
    queued = [
        asyncio.create_task(sitl_dm.arm(SITL_DRONE, schedule=True)),
        asyncio.create_task(sitl_dm.takeoff(SITL_DRONE, altitude=ALTITUDE, schedule=True)),
        asyncio.create_task(sitl_dm.fly_to(SITL_DRONE, local=target, yaw=0, schedule=True)),
    ]
    await asyncio.sleep(0.5)
    assert len(drone.action_queue) >= 1, "The later actions should still be waiting in the queue."
    results = await asyncio.wait_for(asyncio.gather(*queued), 120)
    assert results == [[True], [True], [True]]
    assert drone.position_ned == pytest.approx(target, abs=POSITION_TOLERANCE)
    await _land(sitl_dm)


async def test_pause_holds_queue_until_resumed(sitl_dm: DroneManager):
    """While paused, queued actions don't start. They continue after resuming.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    sitl_dm.pause(SITL_DRONE)
    arm = asyncio.create_task(sitl_dm.arm(SITL_DRONE, schedule=True))
    await asyncio.sleep(3)
    assert not arm.done() and not drone.is_armed
    sitl_dm.resume(SITL_DRONE)
    assert await asyncio.wait_for(arm, 30) == [True]
    assert drone.is_armed
    assert await asyncio.wait_for(sitl_dm.disarm(SITL_DRONE), 30) == [True]


async def test_immediate_command_replaces_queue(sitl_dm: DroneManager):
    """A command that isn't scheduled cancels the current action and clears the queue.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    start = drone.position_ned.copy()
    far_away = asyncio.create_task(sitl_dm.fly_to(SITL_DRONE, local=start + np.array([40, 0, 0]), yaw=0,
                                                  schedule=True))
    await asyncio.sleep(2)
    assert await asyncio.wait_for(sitl_dm.fly_to(SITL_DRONE, local=start, yaw=0), 60) == [True]
    assert far_away.done()
    assert drone.position_ned == pytest.approx(start, abs=POSITION_TOLERANCE)
    await _land(sitl_dm)


async def test_flight_modes(sitl_dm: DroneManager):
    """Flight mode changes are confirmed by the drone, and invalid modes are rejected.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    await asyncio.wait_for(sitl_dm.change_flightmode(SITL_DRONE, "hold"), 30)
    assert drone.flightmode == FlightMode.HOLD
    await asyncio.wait_for(sitl_dm.change_flightmode(SITL_DRONE, "not_a_mode"), 30)
    assert drone.flightmode == FlightMode.HOLD
    await asyncio.wait_for(sitl_dm.change_flightmode(SITL_DRONE, "land"), 30)
    assert drone.flightmode == FlightMode.LAND
    assert await wait_until(lambda: not drone.in_air, 60)
    assert await wait_until(lambda: not drone.is_armed, 30)


@pytest.mark.xfail(reason=LANDING_BUG, strict=False)
async def test_stop_lands_and_disarms(sitl_dm: DroneManager):
    """Stopping a flying drone lands and disarms it.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    assert await asyncio.wait_for(sitl_dm.action_stop([SITL_DRONE]), 120) == [True]
    assert not drone.in_air
    assert await wait_until(lambda: not drone.is_armed, 30)


async def test_kill_disarms(sitl_dm: DroneManager):
    """Killing an armed drone on the ground disarms it immediately.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    assert await asyncio.wait_for(sitl_dm.arm(SITL_DRONE), 30) == [True]
    assert await asyncio.wait_for(sitl_dm.kill([SITL_DRONE]), 30) == [True]
    assert await wait_until(lambda: not drone.is_armed, 10)


async def test_disconnect_refused_while_armed(sitl_dm: DroneManager):
    """An armed drone can't be disconnected without force, a disarmed one can.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    assert await asyncio.wait_for(sitl_dm.arm(SITL_DRONE), 30) == [True]
    await sitl_dm.disconnect(SITL_DRONE)
    assert SITL_DRONE in sitl_dm.drones
    assert await asyncio.wait_for(sitl_dm.disarm(SITL_DRONE), 30) == [True]
    await sitl_dm.disconnect(SITL_DRONE)
    assert SITL_DRONE not in sitl_dm.drones
