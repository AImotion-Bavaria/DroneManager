import asyncio
import math

from mavsdk.asyncio.plugins.gimbal import GimbalError
from mavsdk.asyncio.plugins.gimbal import ControlMode as MAVControlMode
from mavsdk.asyncio.plugins.gimbal import GimbalMode as MAVGimbalMode
from mavsdk.asyncio.plugins.gimbal import SendMode as MAVSendMode

import dronemanager.drone
from dronemanager.plugin import Plugin
from dronemanager.utils import relative_gps


ControlMode = MAVControlMode
GimbalMode = MAVGimbalMode
SendMode = MAVSendMode


class GimbalPlugin(Plugin):
    PREFIX = "gimbal"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_gimbals,
            "remove": self.remove_gimbals,
            "take": self.take_control,
            "release": self.release_control,
            "set": self.set_gimbal_angles,
            "point": self.point_gimbal_at,
            "mode": self.set_gimbal_mode,
            "status": self.status,
        }
        self.background_functions = [
        ]
        self.gimbals: dict[str, list[Gimbal]] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        self.logger.debug("Starting Gimbal plugin...")
        await super().start()
        #for drone in self.dm.drones:
        #    await self.add_gimbals(drone)

    async def close(self):
        """ Removes all gimbals """
        await super().close()
        coros = [self.remove_gimbals(drone) for drone in self.gimbals]
        await asyncio.gather(*coros)

    def check_has_gimbal(self, drone):
        if drone not in self.gimbals:
            self.logger.warning(f"No drone with gimbal {drone}!")
            return False
        return True

    async def add_gimbals(self, drone: str):
        """ Add Gimbals from/for a given drone to the plugin"""
        self.logger.info(f"Adding gimbal to drone {drone}")
        try:
            self._running_tasks.add(asyncio.create_task(self._gimbal_lister(drone)))
            await asyncio.sleep(1)
            await self.status(drone)
            return True
        except Exception as e:
            self.logger.warning(f"Couldn't add a gimbal to {drone} due to an exception!")
            self.logger.debug(repr(e), exc_info=True)
            return False

    async def remove_gimbals(self, drone: str):
        """ Remove a gimbal from the plugin"""
        self.logger.info(f"Removing gimbal to drone {drone}")
        gimbal_list = self.gimbals.pop(drone)
        for gimbal in gimbal_list:
            await gimbal.close()
            del gimbal

    async def status(self, drone: str):
        if self.check_has_gimbal(drone):
            for gimbal in self.gimbals[drone]:
                gimbal.log_status()

    async def take_control(self, drone: str):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                res = await self.gimbals[drone][0].take_control()
                if res:
                    self.logger.info(f"Took control over gimbal {drone}")
                else:
                    self.logger.info(f"Couldn't take control over gimbal on {drone}!")
            else:
                raise NotImplementedError("Only 1 gimbal per drone is allowed")

    async def release_control(self, drone: str):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                self.logger.info(f"Releasing control over gimbal for {drone}")
                await self.gimbals[drone][0].release_control()
            else:
                raise NotImplementedError("Only 1 gimbal per drone is allowed")

    async def set_gimbal_angles(self, drone: str, pitch: float, yaw: float):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                try:
                    res =  await self.gimbals[drone][0].set_gimbal_angles(pitch, yaw)
                    return res
                except Exception as e:
                    self.logger.error("Couldn't set angles due to an exception!")
                    self.logger.debug(repr(e), exc_info=True)
                    return False
            else:
                raise NotImplementedError("Only 1 gimbal per drone is allowed")
        return False

    async def set_gimbal_rate(self, drone: str, pitch_rate: float, yaw_rate: float):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                try:
                    return await self.gimbals[drone][0].set_gimbal_angles(pitch_rate, yaw_rate)
                except Exception as e:
                    self.logger.error("Couldn't set angular rates due to an exception!")
                    self.logger.debug(repr(e), exc_info=True)
                    return False
            else:
                raise NotImplementedError("Only 1 gimbal per drone is allowed")
        return False

    async def point_gimbal_at(self, drone: str, x1: float, x2: float, x3: float, relative: bool = False):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                if relative:
                    res =  await self.gimbals[drone][0].point_gimbal_at_relative(x1, x2, x3)
                else:
                    res =  await self.gimbals[drone][0].point_gimbal_at(x1, x2, x3)
                return res
            else:
                raise NotImplementedError("Only 1 gimbal per drone is allowed")
        return False

    async def set_gimbal_mode(self, drone: str, mode: str):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                res = await self.gimbals[drone][0].set_gimbal_mode(mode)
                if res:
                    self.logger.info(f"Gimbal mode changed to {mode}")
                else:
                    self.logger.warning("Couldn't change gimbal mode!")
                return res
            else:
                raise NotImplementedError("Only 1 gimbal per drone is allowed")
        return False

    async def _gimbal_lister(self, drone: str):
        drone_obj = self.dm.drones[drone]
        async for gmbl_list in drone_obj._mavgimbal.subscribe_gimbal_list():
            gimbals = gmbl_list.gimbals
            gimbal_objs = []
            for gimbalitem in gimbals:
                gimbal_objs.append(Gimbal(drone_obj.logger, self.dm, drone_obj, gimbal_id = gimbalitem.gimbal_id,
                                          device_id=gimbalitem.gimbal_device_id))
            self.gimbals[drone] = gimbal_objs
        return False


class Gimbal:

    def __init__(self, logger, dm, drone: dronemanager.drone.DroneMAVSDK, gimbal_id: int = 2, device_id: int = 154):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.gimbal_id = gimbal_id
        self.device_id = device_id  # mavlink component id of the gimbal

        self.gimbal_id_commands = self.gimbal_id  # Alternative that always works: 0
        self.roll: float = math.nan
        self.pitch: float = math.nan
        self.yaw: float = math.nan
        self.roll_absolute: float = math.nan
        self.pitch_absolute: float = math.nan
        self.yaw_absolute: float = math.nan
        self.mode: GimbalMode = GimbalMode.YAW_FOLLOW
        self.primary_control: tuple[float, float] = (math.nan, math.nan)
        self.secondary_control: tuple[float, float] = (math.nan, math.nan)
        self._running_tasks = set()
        self.update_rate = 5  # How often we request updates on control and attitude TODO: Implement
        self.start()

    def start(self):
        self._running_tasks.add(asyncio.create_task(self.take_control()))
        self._running_tasks.add(asyncio.create_task(self._gimbal_att_checker()))
        self._running_tasks.add(asyncio.create_task(self._gimbal_control_checker()))

    async def close(self):
        try:
            await self.release_control()
        except Exception as e:
            self.logger.warning("Exception while closing gimbal object, check logs")
            self.logger.debug(repr(e), exc_info=True)
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    @property
    def in_control(self):
        return self.primary_control[0] == self.dm.system_id and self.primary_control[1] == self.dm.component_id

    async def _gimbal_att_checker(self):
        async for attitude in self.drone._mavgimbal.subscribe_attitude():
            if attitude.gimbal_id == self.gimbal_id:
                self.roll = attitude.euler_angle_forward.roll_deg
                self.pitch = attitude.euler_angle_forward.pitch_deg
                self.yaw = attitude.euler_angle_forward.yaw_deg
                self.roll_absolute = attitude.euler_angle_north.roll_deg
                self.pitch_absolute = attitude.euler_angle_north.pitch_deg
                self.yaw_absolute = attitude.euler_angle_north.yaw_deg

    async def _gimbal_control_checker(self):
        async for ctrl in self.drone._mavgimbal.subscribe_control_status():
            if ctrl.gimbal_id == self.gimbal_id:
                self.primary_control = (ctrl.sysid_primary_control, ctrl.compid_primary_control)
                self.secondary_control = (ctrl.sysid_secondary_control, ctrl.compid_secondary_control)

    def log_status(self):
        self.logger.info(f"Gimbal control: {'Yes' if self.in_control else 'No'}, P:{self.primary_control}, "
                         f"S: {self.secondary_control}, "
                         f"Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}, "
                         f"AbsRoll: {self.roll_absolute}, AbsPitch: {self.pitch_absolute}, AbsYaw: {self.yaw_absolute}")

    async def take_control(self):
        gimbal_id = self.gimbal_id_commands
        return await self._error_wrapper(self.drone._mavgimbal.take_control, gimbal_id, ControlMode.PRIMARY)

    async def release_control(self):
        gimbal_id = self.gimbal_id_commands
        return await self._error_wrapper(self.drone._mavgimbal.release_control, gimbal_id)

    async def point_gimbal_at(self, lat, long, amsl):
        gimbal_id = self.gimbal_id_commands
        res = await self._error_wrapper(self.drone._mavgimbal.set_roi_location, gimbal_id, lat, long, amsl)
        if res:
            self.logger.info("Gimbal accepted ROI command!")
        else:
            self.logger.info("Gimbal didn't accept ROI command!")
        return res

    async def point_gimbal_at_relative(self, x, y, z):
        lat, long, amsl = relative_gps(self.drone.position_global, [x, y, z])
        return await self.point_gimbal_at(lat, long, amsl)

    async def set_gimbal_angles(self, pitch, yaw):
        gimbal_id = self.gimbal_id_commands
        self.logger.info(f"Setting gimbal angles for gimbal {gimbal_id} to {pitch, yaw}")
        return await self._error_wrapper(self.drone._mavgimbal.set_angles, gimbal_id, 0, pitch, yaw, self.mode,
                                         SendMode.ONCE)

    async def set_gimbal_rates(self, pitch_rate, yaw_rate):
        gimbal_id = self.gimbal_id_commands
        return await self._error_wrapper(self.drone._mavgimbal.set_angular_rates, gimbal_id, 0, pitch_rate, yaw_rate,
                                         self.mode, SendMode.ONCE)

    async def set_gimbal_mode(self, mode):
        assert mode in ["follow", "lock"]
        if mode == "follow":
            self.mode = GimbalMode.YAW_FOLLOW
            return True
        elif mode == "lock":
            self.mode = GimbalMode.YAW_LOCK
            return True
        else:
            return False

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(f"GimbalError: {e.result.name}")
            return False
        return True
