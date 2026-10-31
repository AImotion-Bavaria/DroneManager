from abc import ABC, abstractmethod
import asyncio
import math

import dronemanager.drone
from dronemanager.plugin import Plugin
from dronemanager.utils import cancel_running_tasks

from mavsdk.asyncio.plugins.gimbal import ControlMode as MAVControlMode
from mavsdk.asyncio.plugins.gimbal import GimbalMode as MAVGimbalMode
from mavsdk.asyncio.plugins.gimbal import SendMode as MAVSendMode

# Reuse MAVSDK constants for now
ControlMode = MAVControlMode
YawMode = MAVGimbalMode
SendMode = MAVSendMode


class GimbalPlugin(Plugin):

    PREFIX = "gimbal"

    _MODE_MAP = {
        "follow" : YawMode.YAW_FOLLOW,
        "lock": YawMode.YAW_LOCK,
    }

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

    def check_has_gimbal(self, drone: str):
        if drone not in self.gimbals:
            self.logger.warning(f"Drone {drone} does not have a gimbal, or the gimbal has not been added!")
            return False
        return True

    async def add_gimbals(self, drone: str):
        """ Add Gimbals from/for a given drone to the plugin"""
        # Check if the drone comm protocol even supports gimbals
        drone_obj = self.dm.drones[drone]
        if "gimbal" not in drone_obj.SUPPORTS:
            self.logger.info(f"The communication protocol {drone_obj.comm} for {drone} does not support gimbals!")
            return False
        # Get any gimbals
        gimbal_class: type(Gimbal) = drone_obj.SUPPORTS["gimbal"]
        gimbal_identifiers = await gimbal_class.get_gimbals(drone_obj)
        if len(gimbal_identifiers) == 0:
            self.logger.info(f"No gimbals found for {drone}")
            return False
        else:
            all_good = True
            for gimbal_identifier in gimbal_identifiers:
                self.logger.info(f"Adding gimbal to drone {drone}")
                gimbal = None
                try:
                    gimbal = gimbal_class(drone_obj, gimbal_identifier)
                    await gimbal.start()
                    self.gimbals[drone].append(gimbal)
                except Exception as e:
                    self.logger.warning(f"Couldn't add gimbal {gimbal_identifier} to {drone} due to an exception: "
                                        f"{repr(e)}! See the log for more details.")
                    self.logger.debug(repr(e), exc_info=True)
                    all_good = False
                    if gimbal is not None:
                        await gimbal.close()
            await self.status()
            return all_good

    async def remove_gimbals(self, drone: str):
        """Remove gimbals for a drone."""
        if drone not in self.gimbals:
            self.logger.warning(f"Drone {str} doesn't have any gimbals!")
        else:
            self.logger.info(f"Removing gimbals for drone {drone}")
            gimbal_list = self.gimbals.pop(drone)
            for gimbal in gimbal_list:
                await gimbal.close()

    async def status(self):
        for drone in self.dm.drones:
            if self.check_has_gimbal(drone):
                self.logger.info(f"Gimbals for drone {drone}:")
                for gimbal in self.gimbals[drone]:
                    gimbal.log_status()

    async def take_control(self, drone: str, timeout: float = 5) -> bool:
        """Take control of gimbal(s) for this drone."""
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                self.logger.info(f"Taking control of gimbal for drone {drone}")
                res = await self.gimbals[drone][0].take_control(timeout=timeout)
                if res:
                    self.logger.info(f"Took control over gimbal {drone}")
                else:
                    self.logger.info(f"Couldn't take control over gimbal on {drone}!")
                return res
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def release_control(self, drone: str, timeout: float = 5) -> bool:
        """Release control of gimbal(s) for this drone."""
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                self.logger.info(f"Releasing control over gimbal for {drone}")
                res = await self.gimbals[drone][0].release_control(timeout=timeout)
                if res:
                    self.logger.info(f"Released control over gimbal on {drone}")
                else:
                    self.logger.info(f"Couldn't release control over gimbal on {drone}")
                return res
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def set_gimbal_angles(self, drone: str, yaw: float, pitch: float, roll: float | None = None, tolerance: float = 1):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                self.logger.info(f"Setting gimbal angles for {drone} to YPR: {yaw, pitch, roll} with tolerance {tolerance}")
                res = await self.gimbals[drone][0].set_gimbal_angles(yaw, pitch, roll=roll, tolerance=tolerance)
                if not res:
                    self.logger.info(f"Couldn't set gimbal angles!")
                else:
                    self.logger.info(f"Gimbal {drone} angles set to {yaw, pitch, roll}")
                return res
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def set_gimbal_rate(self, drone: str, yaw_rate: float, pitch_rate: float, roll_rate: float | None = None):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                self.logger.info(f"Setting gimbal angles for {drone} to YPR rates: {yaw_rate, pitch_rate, roll_rate}")
                res = await self.gimbals[drone][0].set_gimbal_rates(yaw_rate, pitch_rate, roll_rate=roll_rate)
                if not res:
                    self.logger.info(f"Couldn't set gimbal angles!")
                else:
                    self.logger.info(f"Gimbal {drone} angular rates set to {yaw_rate, pitch_rate, roll_rate}")
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def point_gimbal_at(self, drone: str, x: float, y: float, z: float, relative: bool = False):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                if relative:
                    self.logger.info(f"Pointing gimbal {drone} pointing at local {x, y, z}")
                    res =  await self.gimbals[drone][0].point_gimbal_at_relative(x, y, z)
                else:
                    self.logger.info(f"Pointing gimbal {drone} pointing at Lat {x}, Long {y}, AMSL {z}")
                    res =  await self.gimbals[drone][0].point_gimbal_at(x, y, z)
                if not res:
                    self.logger.info(f"Couldn't point gimbal {drone}!")
                else:
                    self.logger.info(f"Gimbal {drone} accepted command!")
                return res
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def set_gimbal_mode(self, drone: str, mode: str):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                res = await self.gimbals[drone][0].set_gimbal_mode(self._MODE_MAP[mode])
                if res:
                    self.logger.info(f"Gimbal mode changed to {mode}")
                else:
                    self.logger.warning("Couldn't change gimbal mode!")
                return res
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False


class Gimbal(ABC):
    def __init__(self, drone: dronemanager.drone.Drone, gimbal_identifier: str):
        self.logger = drone.logger.getChild(f"Gimbal-{gimbal_identifier}")
        self.drone = drone
        self.gimbal_identifier = gimbal_identifier

        self.roll: float = math.nan
        self.pitch: float = math.nan
        self.yaw: float = math.nan
        self.roll_absolute: float = math.nan
        self.pitch_absolute: float = math.nan
        self.yaw_absolute: float = math.nan
        self.mode: YawMode = YawMode.YAW_FOLLOW
        self.control: tuple[float, float] = (math.nan, math.nan)
        self._running_tasks = set()

    @classmethod
    @abstractmethod
    async def get_gimbals(cls, drone: dronemanager.drone.Drone) -> list[str]:
        """Method to discover any gimbals on this drone.

        Should return a list of gimbal identifiers.
        """
        raise NotImplementedError

    @abstractmethod
    async def start(self):
        """Start whatever processing is required to use this gimbal."""
        raise NotImplementedError

    @abstractmethod
    async def close(self):
        """Releases control and cancels running tasks."""
        try:
            await self.release_control()
        except Exception as e:
            self.logger.warning(f"Exception while closing gimbal object: {repr(e)}, see the logs for more details!")
            self.logger.debug(repr(e), exc_info=True)
        cancel_running_tasks(self._running_tasks)

    @property
    @abstractmethod
    def in_control(self) -> bool:
        """Whether we are controlling this gimbal."""
        raise NotImplementedError

    @abstractmethod
    def log_status(self):
        """Should write information about the gimbal status, such as position, mode, control, etc. to the logger"""
        raise NotImplementedError

    @abstractmethod
    async def take_control(self, timeout: float = 5):
        """Take control of this gimbal.

        Should return True if the gimbal accepted the command once control has changed, and False if not.
        """
        raise NotImplementedError

    @abstractmethod
    async def release_control(self, timeout: float = 5):
        """Release control of this gimbal.

        Should return True if the gimbal accepted the command once control has changed, and False if not.
        """
        raise NotImplementedError

    @abstractmethod
    async def point_gimbal_at(self, lat, long, amsl):
        """Point the gimbal at a GPS coordinate.

        The gimbal should stay locked on this coordinate as the drone moves. For hardware or protocols which do not
        support this, raise a descriptive Exception.
        Should return whether the gimbal accepted the command.
        """
        raise NotImplementedError

    @abstractmethod
    async def point_gimbal_at_relative(self, x, y, z) -> bool:
        """Point the gimbal at a local coordinate.

        The gimbal should stay locked on this coordinate as the drone moves. For hardware or protocols which do not
        support this, a warning should be logged.
        Should return whether the gimbal accepted the command.
        """
        raise NotImplementedError

    @abstractmethod
    async def set_gimbal_angles(self, yaw: float, pitch: float, roll: float | None = None, tolerance: float = 1) -> bool:
        """Set this gimbal to the given yaw, pitch and roll angles.

        If no roll is provided, it should be ignored. If an axis is provided but the gimbal doesn't support setting it,
        a warning should be logged.
        Should True when the gimbal has reached the angles after accepting, or False when the command is rejected.
        """
        raise NotImplementedError

    @abstractmethod
    async def set_gimbal_rates(self, yaw_rate: float, pitch_rate: float, roll_rate: float | None = None) -> bool:
        """Set angular movement rates for this gimbal.

        If not roll rate is provided, it should be ignored. If an axis is provided, but the gimbal doesn't support it,
        a warning should be logged.
        Should return whether the gimbal accepted the command.
        """
        raise NotImplementedError

    @abstractmethod
    async def set_gimbal_mode(self, mode: YawMode):
        """Set the yaw mode for this gimbal.

        Gimbals which do not support the provided mode should log a warning.
        Should return whether the gimbal accepted the command.
        """
        raise NotImplementedError
