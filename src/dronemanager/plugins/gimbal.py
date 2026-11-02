from abc import ABC, abstractmethod
import asyncio
import math
from typing import Generic, Type

from dronemanager.drone import DroneType
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
            "list": self.list_potential_gimbals,
            "add": self.add_gimbals,
            "remove": self.remove_gimbals,
            "take": self.take_control,
            "release": self.release_control,
            "set": self.set_gimbal_angles,
            "rate": self.set_gimbal_rate,
            "point": self.point_gimbal_at,
            "mode": self.set_gimbal_mode,
            "status": self.status,
        }
        self.background_functions = [
        ]
        self.gimbals: dict[str, list[Gimbal]] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        await super().start()
        self.logger.debug("Started Gimbal plugin.")

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

    async def _get_potential_gimbal_idents(self, drone: str):
        drone_obj = self.dm.drones[drone]
        if "gimbal" not in drone_obj.comm.SUPPORTS:
            self.logger.info(f"The communication protocol {drone_obj.comm} for {drone} does not support gimbals!")
        gimbal_class: Type[Gimbal] = drone_obj.comm.SUPPORTS["gimbal"]
        gimbal_identifiers = await gimbal_class.get_gimbals(drone_obj)
        return gimbal_identifiers

    async def list_potential_gimbals(self, drone: str):
        gimbal_identifiers = await self._get_potential_gimbal_idents(drone)
        drone_obj = self.dm.drones[drone]
        gimbal_class: Type[Gimbal] = drone_obj.comm.SUPPORTS["gimbal"]
        if len(gimbal_identifiers) == 0:
            self.logger.info(f"No gimbals found for {drone}")
        out_str = "\n".join([gimbal_class.prettify_identifier(identifier) for identifier in gimbal_identifiers])
        self.logger.info(f"Discovered gimbals for drone {drone}:\n{out_str}")

    async def add_gimbals(self, drone: str):
        """Add Gimbals from/for a given drone to the plugin"""
        gimbal_identifiers = await self._get_potential_gimbal_idents(drone)
        drone_obj = self.dm.drones[drone]
        gimbal_class: Type[Gimbal] = drone_obj.comm.SUPPORTS["gimbal"]
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
                    if drone not in self.gimbals:
                        self.gimbals[drone] = [gimbal]
                    else:
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
            self.logger.warning(f"Drone {drone} doesn't have any gimbals!")
            return False
        else:
            self.logger.info(f"Removing gimbals for drone {drone}")
            gimbal_list = self.gimbals.pop(drone)
            for gimbal in gimbal_list:
                await gimbal.close()
            await self.status()
            return True

    async def status(self):
        for drone in self.gimbals:
            self.logger.info(f"Gimbals for drone {drone}:")
            for gimbal in self.gimbals[drone]:
                gimbal.log_status()

    async def take_control(self, drone: str, timeout: float = 5) -> bool:
        """Take control of gimbal(s) for this drone."""
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                return await self.gimbals[drone][0].take_control(timeout=timeout)
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def release_control(self, drone: str, timeout: float = 5) -> bool:
        """Release control of gimbal(s) for this drone."""
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                res = await self.gimbals[drone][0].release_control(timeout=timeout)
                return res
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def set_gimbal_angles(self, drone: str, yaw: float, pitch: float, roll: float | None = None, tolerance: float = 1):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                return await self.gimbals[drone][0].set_gimbal_angles(yaw, pitch, roll=roll, tolerance=tolerance)
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def set_gimbal_rate(self, drone: str, yaw_rate: float, pitch_rate: float, roll_rate: float | None = None):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                return await self.gimbals[drone][0].set_gimbal_rates(yaw_rate, pitch_rate, roll_rate=roll_rate)
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def point_gimbal_at(self, drone: str, lat: float, long: float, amsl: float):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                return await self.gimbals[drone][0].point_gimbal_at(lat, long, amsl)
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False

    async def set_gimbal_mode(self, drone: str, mode: str):
        if self.check_has_gimbal(drone):
            if len(self.gimbals[drone]) == 1:
                return await self.gimbals[drone][0].set_gimbal_mode(self._MODE_MAP[mode])
            else:
                raise NotImplementedError("Currently only 1 gimbal per drone is supported")
        return False


class Gimbal(ABC, Generic[DroneType]):
    def __init__(self, drone: DroneType, gimbal_identifier: str):
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
        self._running_tasks = set()

    @classmethod
    @abstractmethod
    async def get_gimbals(cls, drone: DroneType) -> list[str]:
        """Method to discover any gimbals on this drone.

        Should return a list of gimbal identifiers.
        """
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def prettify_identifier(cls, identifier) -> str:
        """A pretty, human understandable version of the identifier."""
        raise NotImplementedError

    @abstractmethod
    async def start(self):
        """Start whatever processing is required to use this gimbal."""
        raise NotImplementedError

    @abstractmethod
    async def close(self):
        """Close the gimbal object, releasing any resources.

        This base implementation releases control and cancels running tasks. Implementing classes should call this
        during their own close procedure."""
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
        """Should write information about the gimbal status, such as position, mode, control, etc. to the logger."""
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
