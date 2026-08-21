"""Plugin for controlling Cameras."""
from __future__ import annotations

from abc import ABC, abstractmethod
import asyncio
from typing import Generic, Type, Any

from dronemanager.drone import DroneType
from dronemanager.plugin import Plugin
from dronemanager.utils import cancel_running_tasks


class CameraPlugin(Plugin):
    PREFIX = "cam"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "list": self.list_potential_cameras,
            "add": self.add_camera,
            "remove": self.remove_cameras,
            "status": self.status,
            "settings": self.parameters,
            "set-setting": self.set_parameter,
            "photo": self.take_picture,
            "video-start": self.start_video,
            "video-stop": self.stop_video,
            "zoom": self.set_zoom,
        }
        self.background_functions = [
        ]
        self.cameras: dict[str, list[Camera]] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        await super().start()
        self.logger.debug("Started Camera plugin.")

    async def close(self):
        """Removes all cameras"""
        await super().close()
        coros = [self.remove_cameras(drone) for drone in self.cameras]
        await asyncio.gather(*coros)

    def check_has_camera(self, drone: str):
        if drone not in self.cameras:
            self.logger.warning(f"Drone {drone} does not have a camera, or the camera has not been added!")
            return False
        return True

    async def _get_potential_camera_idents(self, drone: str) -> tuple[list[str], list[str]]:
        drone_obj = self.dm.drones[drone]
        if "camera" not in drone_obj.comm.SUPPORTS:
            self.logger.info(f"The communication protocol {drone_obj.comm} for {drone} does not support cameras!")
        cam_class: Type[Camera] = drone_obj.comm.SUPPORTS["camera"]
        cam_identifiers = await cam_class.get_cameras(drone_obj)
        return cam_identifiers

    async def list_potential_cameras(self, drone: str):
        cam_identifiers, extra_info = await self._get_potential_camera_idents(drone)
        drone_obj = self.dm.drones[drone]
        cam_class: Type[Camera] = drone_obj.comm.SUPPORTS["camera"]
        if len(cam_identifiers) == 0:
            self.logger.info(f"No cameras found for {drone}")
        identifier_entries = []
        for i, identifier in enumerate(cam_identifiers):
            identifier_entries.append(f"{cam_class.prettify_identifier(identifier, extra_info[i])}")
        out_str = "\n".join(identifier_entries)
        self.logger.info(f"Discovered cameras for drone {drone}:\n{out_str}")
        return cam_identifiers

    async def add_camera(self, drone: str, camera_identifier: str) -> Camera | None:
        """Add a camera from/for a given drone to the plugin.

        Returns the camera object if it was added successfully, None otherwise."""
        self.logger.info(f"Adding camera to drone {drone}.")
        camera_identifiers, extra_info = await self._get_potential_camera_idents(drone)
        drone_obj = self.dm.drones[drone]
        camera_class: Type[Camera] = drone_obj.comm.SUPPORTS["camera"]
        if len(camera_identifiers) == 0:
            self.logger.info(f"No cameras found for {drone}")
            return None
        elif camera_identifier not in camera_identifiers:
            self.logger.info(f"Drone {drone} doesn't have a camera {camera_identifier}!")
            return None
        else:
            self.logger.info(f"Added camera {camera_identifier} to drone {drone}.")
            camera = None
            try:
                camera = camera_class(drone_obj, camera_identifier)
                await camera.start()
                if drone not in self.cameras:
                    self.cameras[drone] = [camera]
                else:
                    self.cameras[drone].append(camera)
            except Exception as e:
                self.logger.warning(f"Couldn't add camera {camera_identifier} to {drone} due to an exception: "
                                    f"{repr(e)}! See the log for more details.")
                self.logger.debug(repr(e), exc_info=True)
                if camera is not None:
                    await camera.close()
                return None
        await self.status()
        return camera

    async def remove_cameras(self, drone: str):
        """Remove a camera from a drone."""
        if drone not in self.cameras:
            self.logger.warning(f"Drone {drone} doesn't have any cameras!")
            return False
        else:
            self.logger.info(f"Removing cameras from drone {drone}")
            camera_list = self.cameras[drone]
            for camera in camera_list:
                await camera.close()
            await self.status()
            return True

    async def status(self):
        self.logger.info("Status for all connected cameras:")
        for drone in self.cameras:
            self.logger.info(f"Cameras for drone {drone}:")
            for camera in self.cameras[drone]:
                camera.log_status(self.logger)

    async def parameters(self, drone: str, camera_identifier: str):
        if self.check_has_camera(drone):
            for camera in self.cameras[drone]:
                if camera.camera_identifier == camera_identifier:
                    self.logger.info(f"Showing parameters for camera {camera.camera_identifier} on {drone}:")
                    await camera.print_parameters()

    async def set_parameter(self, drone: str, camera_identifier: str, param_name: str, param_value: str):
        if self.check_has_camera(drone):
            found_cam = False
            for camera in self.cameras[drone]:
                if camera.camera_identifier == camera_identifier:
                    found_cam = True
                    if param_name not in camera.parameters:
                        self.logger.warning(f"Camera {camera.camera_identifier} on {drone} has no parameter {param_name}!")
                        return False
                    parsed_value = camera.parse_parameter_value(param_name, param_value)
                    if parsed_value is None:
                        self.logger.warning(f"Parameter value {param_value} couldn't be parsed!")
                    else:
                        await camera.set_parameter(param_name, parsed_value)
                        return True
            if not found_cam:
                self.logger.warning(f"Couldn't find camera {camera_identifier} for drone {drone}")
        return False

    async def take_picture(self, drone: str, camera_identifier: str):
        if self.check_has_camera(drone):
            for camera in self.cameras[drone]:
                if camera.camera_identifier == camera_identifier:
                    return await camera.take_picture()
            self.logger.warning(f"Couldn't find camera {camera_identifier} for drone {drone}")
        return False

    async def start_video(self, drone: str, camera_identifier: str):
        if self.check_has_camera(drone):
            for camera in self.cameras[drone]:
                if camera.camera_identifier == camera_identifier:
                    return await camera.start_video()
            self.logger.warning(f"Couldn't find camera {camera_identifier} for drone {drone}")
        return False

    async def stop_video(self, drone: str, camera_identifier: str):
        if self.check_has_camera(drone):
            for camera in self.cameras[drone]:
                if camera.camera_identifier == camera_identifier:
                    return await camera.stop_video()
            self.logger.warning(f"Couldn't find camera {camera_identifier} for drone {drone}")
        return False

    async def set_zoom(self, drone: str, camera_identifier: str, zoom: float):
        if self.check_has_camera(drone):
            for camera in self.cameras[drone]:
                if camera.camera_identifier == camera_identifier:
                    return await camera.set_zoom(zoom)
            self.logger.warning(f"Couldn't find camera {camera_identifier} for drone {drone}")
        return False


class Camera(ABC, Generic[DroneType]):

    def __init__(self, drone: DroneType, camera_identifier: str):
        self.logger = drone.logger
        self.drone = drone
        self.camera_identifier = camera_identifier
        self.parameters: dict[str, CameraParameter] = {}

        self._running_tasks = set()

    @classmethod
    @abstractmethod
    async def get_cameras(cls, drone: DroneType) -> tuple[list[str], list[str]]:
        """Method to discover any cameras on this drone.

        Should return a tuple of lists, where the first list stores each the camera identifier and the second
        carries extra information to help a human user identify the camera.
        """
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def prettify_identifier(cls, base_identifier: str, extra_info: str) -> str:
        """A pretty, human understandable version of the identifier.

        The extra_info attribute can be used to provide more information to a user about what the camera is, without
        bloating the identifier string itself."""
        raise NotImplementedError

    @abstractmethod
    async def start(self):
        """Start whatever processing is required to use this camera."""
        raise NotImplementedError

    @abstractmethod
    async def close(self):
        """Close the camera object, releasing any resources.

        This base implementation cancels running tasks. Implementing classes should call this during their own close
        procedure.
        """
        cancel_running_tasks(self._running_tasks)

    @property
    def parameters_loaded(self):
        """If the parameters and settings for this camera have been loaded."""
        return len(self.parameters) != 0

    def log_status(self, logger):
        """Should write information about the camera state.

        For example, this could be any key settings, if it is capable of images or videos, etc.
        """
        raise NotImplementedError

    async def take_picture(self) -> bool:
        """Take a picture (or series of pictures) with this camera.

        Exact behaviour can depend on the camera parameters.
        Should return information on the captured images, or None if None were taken or the camera refused the command.
        """
        raise NotImplementedError

    async def start_video(self) -> bool:
        """Turn video recording on or off.

        Should return True if the camera accepted the command."""
        raise NotImplementedError

    async def stop_video(self) -> bool:
        """Turn video recording on or off.

        Should return information on the captured video."""
        raise NotImplementedError

    async def set_zoom(self, zoom: float):
        """Set the zoom for this camera.

        Behaviour can depend on the camera and communication protocol."""
        raise NotImplementedError

    async def load_camera_parameters(self):
        """Load parameters and settings for the camera."""
        raise NotImplementedError

    async def print_parameters(self):
        """Log loaded parameters and settings for the camera."""
        raise NotImplementedError

    def parse_parameter_value(self, param_name: str, param_value: str) -> bool | int | float | str | None:
        """Parse a parameter value from a string (for example from the CLI).

        Exact behaviour will depend on implementing subclasses.

        Should return the parsed value, or None if the value could not be parsed."""
        raise NotImplementedError

    async def set_parameter(self, param_name: str, param_value: bool | int | float | str):
        """Set a parameter to a value."""
        raise NotImplementedError


class CameraParameter:

    def __init__(self, name: str, value: int | float | bool | str, param_type: Any, *args, **kwargs):
        self.name: str = name  # Name of the parameter as str, params will be referred to by this
        self.value: int | float | bool | str = value  # Value of the parameter
        self.param_type: Any = param_type

    def to_json_dict(self):
        """Create a json serializable dictionary for this parameter."""
        raise NotImplementedError

    @classmethod
    def from_json_dict(cls, json_dict) -> CameraParameter:
        """Create a CameraParameter object from a loaded json dict, such as from to_json_dict."""
        raise NotImplementedError
