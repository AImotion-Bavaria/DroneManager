"""Plugin for controlling MAVSDK Cameras"""
import asyncio
import os
import struct
import requests
import math
from lxml import etree

import mavsdk.camera

from dronemanager.plugin import Plugin
from dronemanager.utils import CACHE_DIR, coroutine_awaiter

import pymavlink.dialects.v20.ardupilotmega

# TODO: Overlooked the mavsdk param plugin, check if that makes the camera plugin work properly
# TODO: Parameterrange in the cam def xml parsing function


class CameraPlugin(Plugin):
    PREFIX = "cam"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_camera,
            "remove": self.remove_camera,
            "status": self.status,
            "settings": self.parameters,
            "set-setting": self.set_parameter,
            "photo": self.take_picture,
            "start": self.start_video,
            "stop": self.stop_video,
            "zoom": self.set_zoom,
        }
        self.background_functions = [
        ]
        self.cameras: dict[str, Camera] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        self.logger.debug("Starting Camera plugin...")
        await super().start()

    async def close(self):
        """Removes all cameras"""
        await super().close()
        coros = [self.remove_camera(drone) for drone in self.cameras]
        await asyncio.gather(*coros)

    def check_has_camera(self, drone):
        if drone not in self.cameras:
            self.logger.warning(f"No drone with camera {drone}!")
            return False
        return True

    async def add_camera(self, drone: str, camera_id: int = 100):
        """Add cameras from/for a given drone to the plugin"""
        self.logger.info(f"Adding camera to drone {drone}")
        try:
            drone_object = self.dm.drones.get(drone, None)
            if drone_object:
                new_cam = Camera(drone_object.logger, self.dm, drone, camera_id=camera_id)
                success = await new_cam.start()
                if success:
                    self.cameras[drone] = new_cam
                    return True
                else:
                    await new_cam.close()
                    return False
            else:
                self.logger.warning(f"No drone named {drone}")
                return False
        except Exception as e:
            self.logger.warning("Couldn't add the camera to the drone due to an exception!")
            self.logger.debug(repr(e), exc_info=True)
            return False

    async def remove_camera(self, drone: str):
        """Remove a camera from the plugin"""
        self.logger.info(f"Removing camera from drone {drone}")
        camera = self.cameras.pop(drone)
        await camera.close()
        del camera

    async def status(self):
        for drone in self.cameras:
            self.cameras[drone].log_status()

    async def parameters(self, drone: str):
        if self.check_has_camera(drone):
            self.logger.info(f"Showing parameters for camera {self.cameras[drone].camera_id} on {drone}")
            return await self.cameras[drone].print_parameters()
        return False

    async def set_parameter(self, drone: str, param_name: str, param_value: str):
        if self.check_has_camera(drone):
            if param_name not in self.cameras[drone].parameters:
                self.logger.warning(f"Camera {self.cameras[drone].camera_id} on {drone} has no parameter {param_name}")
                return False
            parsed_value = self.cameras[drone].parse_param_value(param_name, param_value)
            if parsed_value is None:
                self.logger.warning("Couldn't set parameter due to invalid input!")
            else:
                await self.cameras[drone].set_parameter(param_name, parsed_value)
                return True
        return False

    async def take_picture(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].take_picture()
        return False

    async def start_video(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].start_video()
        return False

    async def stop_video(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].stop_video()
        return False

    async def set_zoom(self, drone: str, zoom: float):
        if self.check_has_camera(drone):
            return await self.cameras[drone].set_zoom(zoom)
        return False


def _xml_cache_filepath(uri, version):
    pathsafe_uri = uri.replace("/", "_").replace("\\", "_").replace(":", "_").replace("?", "_").replace("\n", "")
    filename = f"{version}_{pathsafe_uri}.xml"
    cache_dir = CACHE_DIR.joinpath("camera_definitions")
    return cache_dir.joinpath(filename)


class ParameterOption:

    def __init__(self, name, value, excludes):
        self.name: str = name  # For display
        self.value: int | float = value  # This is used internally
        self.excludes: list[str] = excludes  # These parameters are rendered irrelevant if this option is set


class CameraParameter:

    def __init__(self, name, param_type, default, control: bool, description: str, updates: list[str],
                 options: list[ParameterOption], min_value: float | None, max_value: float | None,
                 step_size: float | None):
        self.name = name  # Name of the parameter as str, params will be referred to with this
        self.value: int | float = default  # Value of the parameter
        self.param_type = param_type  # Mavlink parameter type, as in definition file
        self.param_type_id = None  # Mavlink parameter type as used by extended parameter protocol
        self.default = default  # Default value
        self.control = control
        self.description = description  # Human readable description

        # A list of other parameters that might get updated when this parameter is changed. Changing this parameter
        # should also reqest updates for these parameters.
        self.updates = updates

        self._options: dict[int, ParameterOption] = {option.value: option for option in options}
        self._options_by_name: dict[str, ParameterOption] = {option.name: option for option in options}

        # Indicates a range of possible options, equivalent to a series of options with name = value and no excludes
        self.min = min_value
        self.max = max_value
        self.step_size = step_size

    @property
    def is_range(self):
        return self.min is not None and self.max is not None

    @property
    def is_bool(self):
        return self.param_type == "bool"

    @property
    def is_option(self):
        return bool(self._options)

    def check_option_valid(self, value):
        if self.is_bool:
            return isinstance(value, bool)
        if self.is_range:
            if self.step_size is None:
                return self.min < value < self.max
            else:
                n_steps = (self.max - self.min) // self.step_size
                return value in [self.min + self.step_size * i for i in range(int(n_steps))]
        else:
            return value in self._options

    def get_current_otion(self):
        return self._options[self.value]

    def get_option_by_name(self, option_name):
        return self._options_by_name.get(option_name, None)

    def get_option_by_value(self, option_value):
        return self._options.get(option_value)

    def get_options(self):
        return list(self._options.values())

    def to_json_dict(self):
        out_dict = {
            "name": self.name,
            "value": self.value,
            "param_type": self.param_type,
            "param_type_id": self.param_type_id,
            "default": self.default,
            "control": self.control,
            "description": self.description,
            "updates": self.updates,
            "options": [option.__dict__ for option in self._options.values()],
            "min_value": self.min,
            "max_value": self.max,
            "step_size": self.step_size,
        }
        return out_dict

    @classmethod
    def from_json_dict(cls, json_dict):
        option_list = [ParameterOption(option["name"], option["value"], option["excludes"]) for option in json_dict["options"]]

        param = cls(json_dict["name"], json_dict["param_type"], json_dict["default"], json_dict["control"],
                    json_dict["description"], json_dict["updates"], option_list, json_dict["min_value"],
                    json_dict["max_value"], json_dict["step_size"])
        param.value = json_dict["value"]
        param.param_type_id = json_dict["param_type_id"]
        return param


class Camera:

    def __init__(self, logger, dm, drone_name: str, camera_id: int = 100):
        self.logger = logger
        self.dm = dm
        self.drone_name = drone_name

        self.camera_id = camera_id
        self._running_tasks = set()
        self._start_background_tasks()

        self.vendor_name = None
        self.model_name = None
        self.cam_def_uri: str | None = None
        self.cam_def_version = None
        self.parameters: dict[str, CameraParameter] = {}
        self._received_params = set()
        self._param_count = None

    def _start_background_tasks(self):
        capture_info_task = asyncio.create_task(self._capture_info_updates())
        self._running_tasks.add(capture_info_task)
        self._running_tasks.add(asyncio.create_task(coroutine_awaiter(capture_info_task, self.logger)))

    @property
    def drone(self):
        try:
            return self.dm.drones[self.drone_name]
        except KeyError:
            return None

    async def _capture_info_updates(self):
        async for capture_info in self.drone.system.camera.capture_info():
            self.logger.info(f"Capture update: Camera {capture_info.component_id} "
                             f"{'succeeded' if capture_info.is_success else 'failed'} with photo at "
                             f"{capture_info.time_utc_us}: {capture_info.file_url}")

    async def start(self):
        have_cam = await self.init_cam_info()
        if have_cam:
            self.logger.info(f"Found camera {self.camera_id}")
            got_params = await self.get_cam_param_definition()
            if not got_params:
                self.logger.info("Connected to a camera, but couldn't load the camera definition")
            else:
                self.logger.info("Loaded camera parameter definition")
            self._get_cam_params()
            return True
        return False

    async def close(self):
        if self.drone is not None and self.drone in self.dm.drones.values():
            self.drone.mav_conn.remove_drone_message_callback(322, self._listen_param_updates)
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    @property
    def params_loaded(self):
        return len(self.parameters) != 0

    def log_status(self):
        camera_id = self.camera_id
        self.logger.info(f"Camera {camera_id}, parameters {'not ' if self.params_loaded else ''}loaded")

    async def take_picture(self,):
        res = await self.drone.mav_conn.send_cmd_long(target_component=100, cmd=2000, param3=1.0, param5=math.nan)
        if not res:
            self.logger.warning("Couldn't take picture!")
        return res

    async def start_video(self):
        res = await self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=2500, param2=1)
        if not res:
            self.logger.warning("Couldn't start video!")
        return res

    async def stop_video(self):
        res = await self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=2501)
        if not res:
            self.logger.warning("Couldn't stop video!")
        return res

    async def set_zoom(self, zoom):
        res = await self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=203, param2=zoom, param5=0)
        if not res:
            self.logger.warning("Couldn't set the zoom!")
        return res

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            res = await func(*args, **kwargs)
        except mavsdk.camera.CameraError as e:
            self.logger.error(f"CameraError: {e._result.result_str}")
            return False
        return res

    async def init_cam_info(self):
        cam_info = await self.drone.mav_conn.request_message(target_component=self.camera_id, message_id=259)
        if not cam_info:
            self.logger.warning("No camera found!")
        else:
            self.vendor_name = bytearray(cam_info.vendor_name).rstrip(b"\x00").decode("ascii")
            self.model_name = bytearray(cam_info.model_name).rstrip(b"\x00").decode("ascii")
            self.cam_def_uri = cam_info.cam_definition_uri
            self.cam_def_version = cam_info.cam_definition_version
            return True
        return False

    async def get_cam_param_definition(self):
        try:
            xml_str = await self._get_cam_definition(self.cam_def_uri, self.cam_def_version)
            if xml_str:
                self._parse_cam_definition(xml_str)
                return True
        except Exception as e:
            self.logger.warning("Couldn't get the camera definition XML due to an exception!")
            self.logger.debug(repr(e), exc_info=True)
        return False

    async def _get_cam_definition(self, uri: str, version) -> str | None:
        xml_str = None
        try:
            self.logger.info("Loading cam definition")
            cached_path = _xml_cache_filepath(uri, version)
            if cached_path.exists():
                xml_str = await asyncio.get_running_loop().run_in_executor(None, self._load_xml, uri, version)
            else:
                self.logger.debug("Camera definition file not cached, downloading...")
                xml_str = await asyncio.get_running_loop().run_in_executor(None, self._download_xml, uri)
                if xml_str:
                    await asyncio.get_running_loop().run_in_executor(None, self._save_xml, xml_str, uri, version)
        except Exception as e:
            self.logger.warning("Exception trying to load the camera definition file!")
            self.logger.debug(repr(e), exc_info=True)
        return xml_str

    def _download_xml(self, uri):
        try:
            response = requests.get(uri, verify=False)
            return response.content
        except requests.RequestException:
            self.logger.warning("Couldn't download the camera definition, probably due to no internet.")
            return None

    def _save_xml(self, xml_str, uri, version):
        filepath = _xml_cache_filepath(uri, version)
        self.logger.debug(f"Saving camera definition xml {filepath}")
        os.makedirs(filepath.parent, exist_ok=True)
        with open(filepath, "wt", encoding="utf-8") as f:
            f.write(xml_str)

    def _load_xml(self, uri, version):
        try:
            filepath = _xml_cache_filepath(uri, version)
            self.logger.debug(f"Loading camera definition xml {filepath}")
            with open(filepath, "rb") as f:
                xml_str = f.read()
            return xml_str
        except OSError as e:
            self.logger.error("Couldn't load the cached camera definition.")
            self.logger.debug(repr(e), exc_info=True)
            return None

    def _parse_cam_definition(self, cam_def_xml: str):
        try:
            root = etree.fromstring(cam_def_xml)
            params = root.findall(".//parameter")
            for param in params:
                name = param.get("name")
                param_type = param.get("type")
                param_type_id = 1
                if param_type == "float":
                    cast_type = float
                    param_type_id = 9  # Assume 9 unless we get an update with different param type
                elif param_type == "bool":
                    cast_type = bool
                    param_type_id = 1
                else:
                    param_type_id = 1  # Assume 1 until we get an update with different param type
                    cast_type = int
                default = cast_type(param.get("default"))
                control = param.get("control") is None or param.get("control") == "1"
                min_val = param.get("min")
                if min_val:
                    min_val = cast_type(min_val)
                max_val = param.get("max")
                if max_val:
                    max_val = cast_type(max_val)
                step_size = param.get("step")
                if step_size:
                    step_size = cast_type(step_size)
                description = param.find("description").text

                options = []
                options_element = param.find("options")
                if options_element:
                    for option_element in options_element:
                        option_name = option_element.get("name")
                        option_value = cast_type(option_element.get("value"))
                        excludes = []
                        exclusions_element = option_element.find("exclusions")
                        if exclusions_element:
                            for exclude_element in exclusions_element:
                                excludes.append(exclude_element.text)
                        options.append(ParameterOption(option_name, option_value, excludes))

                updates = []
                updates_element = param.find("updates")
                if updates_element:
                    for update_element in updates_element:
                        updates.append(update_element.text)

                self.parameters[name] = CameraParameter(name=name, param_type=param_type, default=default,
                                                        control=control, description=description, updates=updates,
                                                        options=options, min_value=min_val, max_value=max_val,
                                                        step_size=step_size)
        except Exception as e:
            self.logger.warning("Couldn't parse the parameter XML")
            self.logger.debug(repr(e), exc_info=True)

    def _get_cam_params(self):
        self.logger.debug("Listening to camera parameters")
        self.drone.mav_conn.send_param_ext_request_list(self.camera_id)
        self.drone.mav_conn.add_drone_message_callback(322, self._listen_param_updates)
        checker_task = asyncio.create_task(self._param_receive_checker())
        self._running_tasks.add(checker_task)
        self._running_tasks.add(asyncio.create_task(coroutine_awaiter(checker_task, self.logger)))

    async def _param_receive_checker(self):
        while len(self._received_params) < self._param_count:
            for param_index in range(self._param_count):
                if param_index not in self._received_params:
                    self.logger.debug(f"Missing parameter {param_index}, requesting again")
                    self.drone.mav_conn.send_param_ext_request_read(self.camera_id, "", param_index)
                    await asyncio.sleep(0.5)

    def _parse_param_update_values(self, msg):
        param_name = msg.param_id
        param_type = msg.param_type
        raw_param_value = msg._param_value_raw
        if param_type < 9:
            length = 2 ** int(param_type / 2)
        elif param_type == 9:
            length = 4
        elif param_type == 10:
            length = 8
        else:
            raise NotImplementedError("Custom parameter types not supported!")
        if param_type in [1, 3, 5, 7]:
            parsed_param_value = int.from_bytes(raw_param_value[:length], byteorder="little", signed=False)
        elif param_type in [2, 4, 6, 8]:
            parsed_param_value = int.from_bytes(raw_param_value[:length], byteorder="little", signed=True)
        elif param_type == 9:
            parsed_param_value = struct.unpack("<f", raw_param_value[:length])[0]
        elif param_type == 10:
            parsed_param_value = struct.unpack("<d", raw_param_value[:length])[0]
        else:
            self.logger.warning("Camera sending custom parameter type, not supported!")
            parsed_param_value = None
        self.logger.debug(f"Received parameter update message: "
                          f"{param_name, parsed_param_value, raw_param_value[:length], msg.param_type}")
        return param_name, parsed_param_value

    def _update_param_value(self, param_name, param_value, param_type_id):
        if param_name in self.parameters:
            if param_value is not None:
                self.logger.debug(f"Updating param {param_name} to {param_value}!")
                self.parameters[param_name].value = param_value
            self.parameters[param_name].param_type_id = param_type_id

    async def _listen_param_updates(self, msg:pymavlink.dialects.v20.ardupilotmega.MAVLink_param_ext_value_message):
        if msg.get_srcComponent() == self.camera_id and msg.get_srcSystem() == self.drone.mav_conn.drone_system:
            param_name, param_value = self._parse_param_update_values(msg)
            self._update_param_value(param_name, param_value, msg.param_type)
            self._param_count = msg.param_count
            self._received_params.add(msg.param_index)

    async def print_parameters(self):
        if len(self.parameters) == 0:
            self.logger.info("No parameters loaded, trying to download...")
            await self.get_cam_param_definition()
        for param_name, parameter in self.parameters.items():
            if parameter.is_option:
                info_string = " ".join([f"'{option.name}'" for option in parameter.get_options()])
                param_value = parameter.get_option_by_value(parameter.value).name
            elif parameter.is_bool:
                info_string = "True or False"
                param_value = parameter.value
            elif parameter.is_range:
                if parameter.step_size:
                    info_string = f"Range between {parameter.min} and {parameter.max} " \
                                  f"with step size {parameter.step_size}"
                else:
                    info_string = f"Range between {parameter.min} and {parameter.max}"
                param_value = parameter.value
            else:
                info_string = "Invalid option somewhow"
                param_value = "You found an edge case for the parameter type, congratulations!"
            self.logger.info(f"Parameter {param_name}: {param_value}. {parameter.description}\n"
                             f"\tOptions: {info_string}")

    def parse_param_value(self, param_name: str, param_value: str) -> bool | int | float | None:
        parameter = self.parameters[param_name]

        if param_name not in self.parameters:
            self.logger.warning(f"Don't have a parameter {param_name}!")
            return False

        # Figure out what the input should be from param_type, is_option and is_bool
        if parameter.is_option:  # Treat it as a string option name, get value (as int) from the option object
            try:
                parsed_value = parameter.get_option_by_name(param_value).value
            except KeyError:
                self.logger.warning(f"{param_value} isn't a valid option!")
                parsed_value = None
        elif parameter.is_bool:  # Treat it as a string, get appropriate bool value
            if param_value in ["True", "true", "1"]:
                parsed_value = 1
            elif param_value in ["False", "false", "0"]:
                parsed_value = 0
            else:
                parsed_value = None
                self.logger.warning("Must be True or False!")
        else:  # Try to parse it into either a float or an int depending on param_type
            try:
                if parameter.param_type_id < 9:
                    parsed_value = int(param_value)
                else:
                    parsed_value = float(param_value)
                if not parameter.check_option_valid(parsed_value):
                    self.logger.warning(f"{parsed_value} isn't a valid choice for this parameter!")
                    parsed_value = None
            except ValueError:
                parsed_value = None
                self.logger.warning("Couldn't parse the input into a valid numeric entry!")

        return parsed_value

    async def set_parameter(self, param_name: str, param_value: bool | int | float):
        # Send the change param message and request updates for any params in the updates list

        if param_name not in self.parameters:
            self.logger.warning(f"Don't have a parameter {param_name}!")
            return False

        parameter = self.parameters[param_name]

        self.logger.debug(f"Trying to set option {param_name}, {parameter.param_type_id} to {param_value}")

        self.drone.mav_conn.send_param_ext_set(self.camera_id, param_name, param_value, parameter.param_type_id)
        ack_msg = await self.drone.mav_conn.listen_message(324, self.camera_id)
        if ack_msg.param_result == 1:
            self.logger.warning("Camera denied parameter change, unsupported value!")
            return False
        elif ack_msg.param_result == 2:
            self.logger.warning("Camera denied parameter change, failed to set!")
            return False
        elif ack_msg.param_result in [0,3]:
            if ack_msg.param_result == 3:
                self.logger.info("Parameter change in progress...")
            param_name, param_value = self._parse_param_update_values(ack_msg)
            self._update_param_value(param_name, param_value, ack_msg.param_type)
            parameter = self.parameters[param_name]
            self.logger.info(f"Set Parameter {param_name} to "
                             f"{param_value if not parameter.is_option else parameter.get_current_otion().name}")

        # Request updates on any params that might have changed.
        for updated_param_name in parameter.updates:
            self.drone.mav_conn.send_param_ext_request_read(self.camera_id, updated_param_name)
        return True
