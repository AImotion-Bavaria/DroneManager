import asyncio
import math
import typing

import mavsdk.gimbal
from mavsdk.gimbal import GimbalError
from mavsdk.gimbal import ControlMode as MAVControlMode
from mavsdk.gimbal import GimbalMode as MAVGimbalMode
from mavsdk.gimbal import SendMode as MAVSendMode

from dronemanager.plugin import Plugin
from dronemanager.utils import relative_gps

# TODO: Big refactor, swap control management to gimbal class, better gimbal presence checking
# TODO: PX4 only supports a single gimbal apparently
# TODO: PX4 and our gimbal don't seem to play well together, for unknown reasons. Will probably have to talk to gimbal
#  direct with raw malvink, mavsdk not useful here. PX4 already setup to passthrough, but we currently get command
#  denied messages from FC that aren't relevant. This will also need a big rework of MAVPassthrough for more convenient
#  command/response handling
# As part of this: big rethink of what the gimbal class does and how


ControlMode = MAVControlMode
GimbalMode = MAVGimbalMode
SendMode = MAVSendMode


class GimbalPlugin(Plugin):
    PREFIX = "gimbal"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_gimbals,
            "remove": self.remove_gimbal,
            "take": self.take_control,
            "release": self.release_control,
            "set": self.set_gimbal_angles,
            "point": self.point_gimbal_at,
            "mode": self.set_gimbal_mode,
            "status": self.status,
        }
        self.background_functions = [
        ]
        self.gimbals: dict[str, Gimbal] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        self.logger.debug("Starting Gimbal plugin...")
        await super().start()

    async def close(self):
        """Removes all gimbals"""
        await super().close()
        coros = [self.remove_gimbal(drone) for drone in self.gimbals]
        await asyncio.gather(*coros)

    def check_has_gimbal(self, drone):
        if drone not in self.gimbals:
            self.logger.warning(f"No drone with gimbal {drone}!")
            return False
        return True

    async def add_gimbals(self, drone: str, device_id: int = 154):
        """Add Gimbals from/for a given drone to the plugin"""
        self.logger.info(f"Adding gimbal to drone {drone}")
        try:
            drone_object = self.dm.drones[drone]
            self.gimbals[drone] = Gimbal(drone_object.logger, self.dm, drone_object, device_id=device_id)
            return True
        except Exception as e:
            self.logger.warning(f"Couldn't add a gimbal to {drone} due to an exception!")
            self.logger.debug(repr(e), exc_info=True)
            return False

    async def remove_gimbal(self, drone: str):
        """Remove a gimbal from the plugin"""
        self.logger.info(f"Removing gimbal to drone {drone}")
        gimbal = self.gimbals.pop(drone)
        await gimbal.close()
        del gimbal

    async def status(self):
        for drone in self.gimbals:
            self.gimbals[drone].log_status()

    async def take_control(self, drone: str):
        if self.check_has_gimbal(drone):
            res = await self.gimbals[drone].take_control()
            if res:
                self.logger.info(f"Took control over gimbal {drone}")
            else:
                self.logger.info(f"Couldn't take control over gimbal on {drone}!")

    async def release_control(self, drone: str):
        if self.check_has_gimbal(drone):
            self.logger.info(f"Releasing control over gimbal for {drone}")
            await self.gimbals[drone].release_control()

    async def set_gimbal_angles(self, drone: str, pitch: float, yaw: float):
        if self.check_has_gimbal(drone):
            try:
                res = await self.gimbals[drone].set_gimbal_angles(pitch, yaw)
                return res
            except Exception as e:
                self.logger.error("Couldn't set angles due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
                return False
        return False

    async def set_gimbal_rate(self, drone: str, pitch_rate: float, yaw_rate: float):
        if self.check_has_gimbal(drone):
            try:
                return await self.gimbals[drone].set_gimbal_angles(pitch_rate, yaw_rate)
            except Exception as e:
                self.logger.error("Couldn't set angular rates due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
                return False
        return False

    async def point_gimbal_at(self, drone: str, x1: float, x2: float, x3: float, relative: bool = False):
        if self.check_has_gimbal(drone):
            if relative:
                res = await self.gimbals[drone].point_gimbal_at_relative(x1, x2, x3)
            else:
                res = await self.gimbals[drone].point_gimbal_at(x1, x2, x3)
            return res
        return False

    async def set_gimbal_mode(self, drone: str, mode: str):
        if self.check_has_gimbal(drone):
            res = await self.gimbals[drone].set_gimbal_mode(mode)
            if res:
                self.logger.info(f"Gimbal mode changed to {mode}")
            else:
                self.logger.warning("Couldn't change gimbal mode!")
            return res
        return False


class Gimbal:

    def __init__(self, logger, dm, drone, device_id: int = 154):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.device_id = device_id  # mavlink component id of the gimbal

        self.gimbal_id_commands = 0
        self.gimbal_id_messages = 0
        self.roll: float = math.nan
        self.pitch: float = math.nan
        self.yaw: float = math.nan
        self.yaw_absolute: float = math.nan
        self.mode: GimbalMode = GimbalMode.YAW_FOLLOW
        self.primary_control: tuple[float, float] = (math.nan, math.nan)
        self.secondary_control: tuple[float, float] = (math.nan, math.nan)
        self._running_tasks = set()
        self.update_rate = 5  # How often we request updates on control and attitude
        self._message_callbacks: dict[int, typing.Callable[[any], typing.Coroutine]] = {
            265: self._gimbal_attitude_callback,
            281: self._gimbal_control_callback,
        }
        self._add_callbacks()
        self.start()

    def start(self):
        self._running_tasks.add(asyncio.create_task(self.take_control()))

    def _add_callbacks(self):
        for message_id, message_callback in self._message_callbacks.items():
            self.drone.mav_conn.add_drone_message_callback(message_id, message_callback)

    def _remove_callbacks(self):
        for message_id, message_callback in self._message_callbacks.items():
            self.drone.mav_conn.remove_drone_message_callback(message_id, message_callback)

    async def close(self):
        try:
            await self.release_control()
            self._remove_callbacks()
        except Exception as e:
            self.logger.warning("Exception while closing gimbal object, check logs")
            self.logger.debug(repr(e), exc_info=True)
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    @property
    def in_control(self):
        return self.primary_control[0] == 245 and self.primary_control[1] == 190

    async def _gimbal_control_callback(self, msg):
        # Check for gimbal manager status messages (281)
        if msg.gimbal_device_id == self.device_id:
            self.primary_control = (msg.primary_control_sysid, msg.primary_control_compid)
            self.secondary_control = (msg.secondary_control_sysid, msg.secondary_control_compid)

    async def _gimbal_attitude_callback(self, msg):
        # If the message is of type MOUNT_ORIENTATION (265) and source system and component match ours: save info
        if msg.get_srcComponent() == self.device_id:
            self.roll = msg.roll
            self.pitch = msg.pitch
            self.yaw = msg.yaw
            self.yaw_absolute = msg.yaw_absolute

    def log_status(self):
        self.logger.info(f"Gimbal control: {'Yes' if self.in_control else 'No'}, P:{self.primary_control}, "
                         f"S: {self.secondary_control}, "
                         f"Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}, Absolute Yaw: {self.yaw_absolute}")

    async def take_control(self):
        gimbal_id = self.gimbal_id_commands
        return await self._error_wrapper(self.drone.system.gimbal.take_control, gimbal_id, ControlMode.PRIMARY)

    async def release_control(self):
        gimbal_id = self.gimbal_id_commands
        return await self._error_wrapper(self.drone.system.gimbal.release_control, gimbal_id)

    async def point_gimbal_at(self, lat, long, amsl):
        gimbal_id = self.gimbal_id_commands
        res = await self._error_wrapper(self.drone.system.gimbal.set_roi_location, gimbal_id, lat, long, amsl)
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
        return await self._error_wrapper(self.drone.system.gimbal.set_angles, gimbal_id, 0, pitch, yaw, self.mode,
                                         SendMode.ONCE)

    async def set_gimbal_rates(self, pitch_rate, yaw_rate):
        gimbal_id = self.gimbal_id_commands
        return await self._error_wrapper(self.drone.system.gimbal.set_angular_rates, gimbal_id, 0, pitch_rate, yaw_rate,
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
            self.logger.error(f"GimbalError: {e._result.result_str}")
            return False
        return True


class GimbalMulti:
    """Should work with properly implemented gimbal managers, but those seem rare."""

    def __init__(self, logger, dm, drone):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.gimbal_list: set[int] = set()
        self.roll: dict[int, float] = {}
        self.pitch: dict[int, float] = {}
        self.yaw: dict[int, float] = {}
        self.mode: dict[int, GimbalMode] = {}
        self.primary_control: dict[int, tuple[float, float]] = {}
        self.secondary_control: dict[int, tuple[float, float]] = {}
        self._add_gimbal(0)
        self._running_tasks = set()
        self.update_rate = 5  # How often we request updates on control and attitude
        self._start_background_tasks()

    def _start_background_tasks(self):
        self._running_tasks.add(asyncio.create_task(self._check_gimbal_attitude()))
        self._running_tasks.add(asyncio.create_task(self._check_gimbal_control()))
        self._running_tasks.add(asyncio.create_task(self._check_connected_gimbals()))

    async def close(self):
        try:
            await self.release_control(None)
        except Exception as e:
            self.logger.warning("Exception while closing gimbal object, check logs")
            self.logger.debug(repr(e), exc_info=True)
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    async def _check_connected_gimbals(self):
        async for gimballist in self.drone.system.gimbal.gimbal_list():
            new_gimbals = [gimbal.gimbal_id for gimbal in gimballist.gimbals]
            self.logger.debug(f"Found gimbals: {new_gimbals}")
            for gimbal_id in new_gimbals:
                if gimbal_id not in self.gimbal_list:
                    self._add_gimbal(gimbal_id)
            for cur_gimbal_id in self.gimbal_list:
                if cur_gimbal_id not in new_gimbals:
                    self._remove_gimbal(cur_gimbal_id)

    def _add_gimbal(self, gimbal_id):
        self.roll[gimbal_id] = math.nan
        self.pitch[gimbal_id] = math.nan
        self.yaw[gimbal_id] = math.nan
        self.primary_control[gimbal_id] = (math.nan, math.nan)
        self.secondary_control[gimbal_id] = (math.nan, math.nan)
        self.mode[gimbal_id] = GimbalMode.YAW_FOLLOW
        self.gimbal_list.add(gimbal_id)

    def _remove_gimbal(self, gimbal_id):
        self.roll.pop(gimbal_id)
        self.pitch.pop(gimbal_id)
        self.yaw.pop(gimbal_id)
        self.primary_control.pop(gimbal_id)
        self.secondary_control.pop(gimbal_id)
        self.gimbal_list.remove(gimbal_id)

    async def _check_gimbal_attitude(self):
        while True:
            try:
                for gimbal_id in self.gimbal_list:
                    self._running_tasks.add(asyncio.create_task(self._check_gimbal_attitude_gimbal(gimbal_id)))
                await asyncio.sleep(1/self.update_rate)
            except Exception as e:
                self.logger.warning("Exception in the gimbal attitude check function! See logs for details.")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_gimbal_attitude_gimbal(self, gimbal_id):
        attitude = await self.drone.system.gimbal.get_attitude(gimbal_id)
        attitude: mavsdk.gimbal.Attitude
        rpy = attitude.euler_angle_forward
        self.roll[gimbal_id] = rpy.roll_deg
        self.pitch[gimbal_id] = rpy.pitch_deg
        self.yaw[gimbal_id] = rpy.yaw_deg
        self.logger.debug(f"Desired {gimbal_id} actual {attitude.gimbal_id}, "
                          f"eulers: {rpy.roll_deg, rpy.pitch_deg, rpy.yaw_deg}, "
                          f"eulers NED: {attitude.euler_angle_north.roll_deg}, "
                          f"{attitude.euler_angle_north.pitch_deg, attitude.euler_angle_north.yaw_deg}, "
                          f"quat: {attitude.quaternion_forward.__dict__},"
                          f"quat ned {attitude.quaternion_north.__dict__}")

    async def _check_gimbal_control(self):
        while True:
            try:
                for gimbal_id in self.gimbal_list:
                    self._running_tasks.add(asyncio.create_task(self._check_gimbal_control_gimbal(gimbal_id)))
                await asyncio.sleep(1/self.update_rate)
            except Exception as e:
                self.logger.warning("Exception in the gimbal control check function! See logs for details.")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_gimbal_control_gimbal(self, gimbal_id):
        gimbal_control = await self.drone.system.gimbal.get_control_status(gimbal_id)
        self.primary_control[gimbal_id] = (gimbal_control.sysid_primary_control,
                                           gimbal_control.compid_primary_control)
        self.secondary_control[gimbal_id] = (gimbal_control.sysid_secondary_control,
                                             gimbal_control.compid_secondary_control)
        self.logger.debug(f"{gimbal_id, gimbal_control.sysid_primary_control}")

    def log_status(self):
        for gimbal_id in self.gimbal_list:
            self.logger.info(f"Gimbal {gimbal_id} control P:{self.primary_control[gimbal_id]}, "
                             f"S: {self.secondary_control[gimbal_id]}, Roll: {self.roll[gimbal_id]}, "
                             f"Pitch: {self.pitch[gimbal_id]}, Yaw: {self.yaw[gimbal_id]}")

    def _gimbal_id_check(self, gimbal_id):
        if gimbal_id is None and len(self.gimbal_list) == 1:
            return list(self.gimbal_list)[0]
        elif gimbal_id is None:
            self.logger.warning("Missing gimbal id argument for drone with multiple gimbals!")
            raise RuntimeError()
        else:
            return gimbal_id

    def in_control(self, gimbal_id: int | None):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        return self.primary_control[gimbal_id] == (self.dm.system_id, self.dm.component_id)

    async def take_control(self, gimbal_id: int | None):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        self.logger.info(f"Taking control of gimbal {gimbal_id}")
        return await self._error_wrapper(self.drone.system.gimbal.take_control, gimbal_id, ControlMode.PRIMARY)

    async def release_control(self, gimbal_id: int | None):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        return await self._error_wrapper(self.drone.system.gimbal.release_control, gimbal_id)

    async def point_gimbal_at(self, gimbal_id: int | None, lat, long, amsl):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        return await self._error_wrapper(self.drone.system.gimbal.set_roi_location, gimbal_id, lat, long, amsl)

    async def point_gimbal_at_relative(self, gimbal_id: int | None, x, y, z):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        lat, long, amsl = relative_gps(self.drone.position_global, [x, y, z])
        return await self.point_gimbal_at(gimbal_id, lat, long, amsl)

    async def set_gimbal_angles(self, gimbal_id: int | None, pitch, yaw):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        self.logger.debug(f"Setting gimbal angles for gimbal {gimbal_id} to {pitch, yaw}")
        return await self._error_wrapper(self.drone.system.gimbal.set_angles, gimbal_id, 0, pitch, yaw,
                                         self.mode.get(gimbal_id, GimbalMode.YAW_FOLLOW), SendMode.ONCE)

    async def set_gimbal_rates(self, gimbal_id: int | None, pitch_rate, yaw_rate):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        return await self._error_wrapper(self.drone.system.gimbal.set_angular_rates, 0, pitch_rate, yaw_rate,
                                         self.mode.get(gimbal_id, GimbalMode.YAW_FOLLOW), SendMode.ONCE)

    async def set_gimbal_mode(self, gimbal_id: int | None, mode):
        assert mode in ["follow", "lock"]
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if mode == "follow":
            self.mode[gimbal_id] = GimbalMode.YAW_FOLLOW
        elif mode == "lock":
            self.mode[gimbal_id] = GimbalMode.YAW_LOCK

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(f"GimbalError: {e._result.result_str}")
            return False
        return True
