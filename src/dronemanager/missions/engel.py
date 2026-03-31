""" Mission for ENGEL data collection.

Capture images and combine with weather data and position information from the capturing drone, storing them.
Also includes functions to retake the same position as in a previous image and capture another image, as well as
assorted post-processing and meta-data handling.
"""

import asyncio
import math
import pathlib
from collections.abc import Callable

import numpy as np
import json
import datetime
import time
import os
import shutil

from dronemanager.navigation.core import Waypoint, WayPointType
from dronemanager.plugins.mission import Mission
from dronemanager.sensors.ecowitt import WeatherData
from dronemanager.plugins.camera import CameraParameter, Camera
from dronemanager.plugins.gimbal import Gimbal
from dronemanager.plugins.controllers import PS4Mapping
from dronemanager.utils import LOG_DIR, coroutine_awaiter


CAPTURE_DIR = os.path.join(LOG_DIR, "engel_data_captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)


class EngelImageInfo:

    def __init__(self, time_utc, gps: np.ndarray, drone_att: np.ndarray, gimbal_att: np.ndarray, gimbal_absolute, cam_file: str):
        self.time = time_utc
        self.gps = gps
        self.drone_att = drone_att
        self.gimbal_att = gimbal_att
        self.gimbal_yaw_absolute = gimbal_absolute
        self.file_location = cam_file

    def to_json_dict(self):
        return {
            "time": self.time.isoformat(),
            "gps": self.gps.tolist(),
            "drone_att": self.drone_att.tolist(),
            "gimbal_att": self.gimbal_att.tolist(),
            "gimbal_yaw_absolute": self.gimbal_yaw_absolute,
            "file_location": self.file_location,
        }

    @classmethod
    def from_json_dict(cls, json_dict):
        img_time = datetime.datetime.fromisoformat(json_dict["time"])
        gps = np.asarray(json_dict["gps"])
        drone_att = np.asarray(json_dict["drone_att"])
        gimbal_att = np.asarray(json_dict["gimbal_att"])
        gimbal_yaw = json_dict["gimbal_yaw_absolute"]
        cam_file = json_dict["file_location"]
        return cls(img_time, gps, drone_att, gimbal_att, gimbal_yaw, cam_file)


class ENGELCaptureInfo:

    def __init__(self, images: list[EngelImageInfo], weather_data: WeatherData, camera_parameters: list[tuple]):
        self.images = images
        self.weather_data = weather_data
        self.camera_parameters = camera_parameters
        self.capture_id = time.time_ns()
        self.reference_id = None  # Base reference image for replay captures, None if not replay capture

    def to_json_dict(self):
        out_dict = {
            "images": [image.to_json_dict() for image in self.images],
            "weather_data": self.weather_data.to_json_dict(),
            "camera_parameters": self.camera_parameters,
            "capture_id": self.capture_id,
            "reference_id": self.reference_id,
        }
        return out_dict

    @classmethod
    def from_json_dict(cls, json_dict):
        images = [EngelImageInfo.from_json_dict(image_dict) for image_dict in json_dict["images"]]
        weather_data = WeatherData.from_json_dict(json_dict["weather_data"])
        cam_params = [(entry[0], entry[1]) for entry in json_dict["camera_parameters"]]
        out = cls(images, weather_data=weather_data, camera_parameters=cam_params)
        out.capture_id = json_dict["capture_id"]
        out.reference_id = json_dict["reference_id"]
        return out

    @classmethod
    def from_json_dict_legacy(cls, json_dict):
        images = [EngelImageInfo.from_json_dict(image_dict) for image_dict in json_dict["images"]]
        weather_data = WeatherData.from_json_dict(json_dict["weather_data"])
        cam_params = [(CameraParameter.from_json_dict(entry).name, CameraParameter.from_json_dict(entry).value) for
                      entry in json_dict["camera_parameters"]]
        out = cls(images, weather_data=weather_data, camera_parameters=cam_params)
        out.capture_id = json_dict["capture_id"]
        out.reference_id = json_dict["reference_id"]
        return out


class ENGELDataMission(Mission):
    """ Data collection mission for ENGEL

    """

    DEPENDENCIES = ["gimbal", "camera", "sensor.ecowitt", "controllers"]

    def __init__(self, dm, logger, name="engel"):
        super().__init__(dm, logger, name)
        mission_cli_commands = {
            "connect": self.connect,
            "capture": self.do_capture,
            "save": self.save_captures_to_file,
            "load": self.load_captures_from_file,
            "copy": self.copy,
            "merge": self.merge,
            "configure": self.configure_cam,
            "replay": self.replay_captures,
            "transfer": self.transfer,
            "done": self.done,
        }
        self.cli_commands.update(mission_cli_commands)
        self.weather_sensor = None
        self.launch_point: Waypoint | None = None  # A dictionary with latitude and longitude and amsl values
        self.rtl_height = 10  # Height above launch point for return
        self.background_functions = [
            self._set_gimbal_rates_controller(),
        ]
        self.drone_name = None
        self.gimbal: Gimbal | None = None
        self.camera: Camera | None = None

        # Information on previous and current captures
        self.capturing = False  # Set to True while a capture is in process to only allow a single capture at a time
        self.max_capture_duration = 5  # Time in seconds after capture command that we listen for capture info messages.
        self.captures: list[ENGELCaptureInfo] = []  # Images taken this session
        self.loaded_captures: list[ENGELCaptureInfo] = []  # Images taken during a previous session, intended to be replayed
        self.loaded_file: str | None = None
        self._current_capture: ENGELCaptureInfo | None = None

        # Due to drone motion, we might have to move beyond gimbal limits during replay, leading to a deadlock. These
        # limits prevent this, by skipping any captures where the gimbal pitch would exceed these values.
        self._gimbal_max_pitch = 40
        self._gimbal_min_pitch = -44

        # Controller stuff
        self._added_controller_buttons: dict[int, Callable] = {}
        self._added_controller_axis_methods: set[Callable] = set()

        # Gimbal is controlled with triggers, press more to move more. Press square to switch between pitch and yaw control.
        self._gimbal_rate = 0
        self._gimbal_max_rate = 10  # Maximum rotation rate of the gimbal in degrees per second
        self._control_gimbal_pitch = True  # If false, control gimbal yaw instead
        self._gimbal_frequency = 20  # Default frequency until we get an actual drone

    async def close(self):
        for button, func in self._added_controller_buttons.items():
            PS4Mapping.remove_method_from_button(button, func)
        for func in self._added_controller_axis_methods:
            PS4Mapping.remove_axis_method(func)
        await super().close()

    async def connect(self):
        """ Connect to the Leitstand sensor"""
        connected = await self.dm.ecowitt.connect("192.168.1.41")
        if connected:
            self.weather_sensor = self.dm.ecowitt

    async def configure_cam(self):
        """ Set parameters for our camera (Workswell WIRIS enterprise), won't work with others"""
        # Standard Parameter Set:
        # IMG_RAD_TIFF      1
        # IMG_RAD_JPEG      0
        # IMG_IR_SUPER      "Off"
        # IMG_SCREEN        0
        # IMG_VIS           1
        # IMG_VHR           1
        # RANGE_TYPE        "Manual"
        # RANGE_MAX         40.0
        # RANGE_MIN         10.0
        # MAIN_CAM          "Visible"
        # ZOOM_THERMO_I     1.0
        # ZOOM_VISIBLE_I    1.0
        self.logger.info("Setting camera parameters to default...")
        await self.camera.set_parameter("IMG_RAD_TIFF", True)
        await self.camera.set_parameter("IMG_RAD_JPEG", False)
        await self.camera.set_parameter("IMG_IR_SUPER", self.camera.parse_param_value("IMG_IR_SUPER", "Off"))
        await self.camera.set_parameter("IMG_SCREEN", False)
        await self.camera.set_parameter("IMG_VIS", True)
        await self.camera.set_parameter("IMG_VHR", True)
        await self.camera.set_parameter("RANGE_TYPE", self.camera.parse_param_value("RANGE_TYPE", "Manual"))
        await self.camera.set_parameter("RANGE_MAX", 40.0)
        await self.camera.set_parameter("RANGE_MIN", 10.0)
        await self.camera.set_parameter("MAIN_CAM", self.camera.parse_param_value("MAIN_CAM", "Visible"))
        await self.camera.set_parameter("LAYOUT_MAIN", self.camera.parse_param_value("LAYOUT_MAIN", "Fullscreen"))
        await self.camera.set_parameter("ZOOM_THERMO_I", self.camera.parse_param_value("ZOOM_THERMO_I", "1.0"))
        await self.camera.set_parameter("ZOOM_VISIBLE_I", self.camera.parse_param_value("ZOOM_VISIBLE_I", "1.0"))

    async def _imaged_captured_callback(self, msg):
        """ Check CAMERA_IMAGE_CAPTURED messages for capture_result and save info if success, log failure otherwise

        This message contains this info:
        time_utc, milliseconds since epoch or boot (unfortunately boot for our camera). Used
        lat, latitude in degrees as integer with 7 figures after decimal
        lon, longitude in degrees as integer with 7 figures after decimal
        alt, amsl in mm
        file_url, str

        :param msg:
        :return:
        """
        if msg.capture_result == 1:
            if msg.time_utc < 1e12:  # Assume this is reporting time since boot if too small
                time_stamp = datetime.datetime.now(datetime.UTC)
            else:
                time_stamp = datetime.datetime.fromtimestamp(msg.time_utc / 1e3, datetime.UTC)
            gps = np.asarray([msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1e3])
            file_url = msg.file_url
            cur_drone_att = self.dm.drones[self.drone_name].attitude
            cur_gimbal_att = np.asarray([self.gimbal.roll, self.gimbal.pitch, self.gimbal.yaw])
            if file_url in [self._current_capture.images[i].file_location for i in range(len(self._current_capture.images))]:
                self.logger.debug("Camera saved over image it just took")
            else:
                self.logger.debug("Captured image, saving info...")
                self._current_capture.images.append(EngelImageInfo(time_stamp, gps, cur_drone_att,
                                                                   cur_gimbal_att, self.gimbal.yaw_absolute, file_url))
        else:
            self.logger.warning("Camera reports failure to capture image!")
            self.logger.debug(msg.to_dict())

    async def do_capture(self, reference_capture: ENGELCaptureInfo | None = None):
        """ Capture an image and store relevant data. """
        try:
            if self.capturing:
                self.logger.warning("Already doing a capture, skipping")
                return False

            # Make sure weather sensor has grabbed latest data
            if self.weather_sensor:
                weather_data = await self.weather_sensor.get_data()
            else:
                self.logger.warning(f"No Weather sensor, using dummy data!")
                weather_data = WeatherData()

            cam_params = [(param.name, param.value) for param in list(self.camera.parameters.values())]

            # Send capture command
            self.capturing = True
            capture = ENGELCaptureInfo([], weather_data, cam_params)
            # If we got a reference capture this is a replay capture, and we add the old id to this one
            if reference_capture is not None:
                capture.reference_id = reference_capture.capture_id

            self._current_capture = capture
            res = await self.camera.take_picture()

            # If command denied: log, return False
            if not res:
                self.logger.warning("Engel capture failed as take photo command was denied")
                self.capturing = False
                self._current_capture = None
                return False
            # If accepted: Collect metadata, listen for capture_info messages for CAMERA_IMAGE_CAPTURED using callback on mav_conn
            else:
                # Add callback, wait capture duration, remove callback
                # TODO: We should know how many images the camera will take after the configure call, maybe just wait for all of those.
                # TODO: Request images that didn't arrive using image index
                # TODO: Directly associate images with the corresponding reference image somehow, instead of the larger "capture"
                mav_conn = self.dm.drones[self.drone_name].mav_conn
                mav_conn.add_drone_message_callback(263, self._imaged_captured_callback)
                await asyncio.sleep(self.max_capture_duration)
                mav_conn.remove_drone_message_callback(263, self._imaged_captured_callback)
                self.capturing = False
                self._current_capture = None
                if len(capture.images) > 0:
                    self.logger.info(f"Captured {len(capture.images)} images!")
                    self.captures.append(capture)
                else:
                    self.logger.warning(f"No images captured! (Maybe capture duration is too short?)")
                return True
        except Exception as e:
            self.logger.warning("Exception in the capturing function!")
            self.logger.debug(repr(e), exc_info=True)
            self.capturing = False
            self._current_capture = None
            return False

    async def set_camera_parameters(self, params: list[tuple]):
        # Go through a list of camera parameters and adjust the connected camera parameters to match
        for parameter in params:
            name, value = parameter
            if self.camera.parameters[name].value != value:
                await self.camera.set_parameter(name, value)

    async def replay_captures(self):
        await self._replay_captures() # Can't actually just queue _replay captures, as it cancels itself during moves

    async def _replay_captures(self):
        """ Function to take the position from previous captures saved to file and capture them all again."""
        # For each loaded capture: Set camera parameters, fly to position, optionally refine position, take new capture
        # Currently just prints loaded info for debug purposes
        drone = self.drones[self.drone_name]
        for capture in self.loaded_captures:
            try:
                reference_image = capture.images[0]
                # Use "visible" as reference image for now. TODO: Figure out if this is best, might have to do screenshots if comparison happens against live feed
                for image in capture.images:
                    if "visible" in image.file_location:
                        reference_image = image

                # Set camera parameters
                cam_set_task = asyncio.create_task(self.set_camera_parameters(capture.camera_parameters))
                self._running_tasks.add(cam_set_task)
                # Fly to position and point gimbal
                # Have to reset gimbal position to drone-relative 0 to prevent running into gimbal limit
                await self.gimbal.set_gimbal_mode("follow")
                await self.gimbal.set_gimbal_angles(0.1, 0.1)
                if drone.is_armed and drone.in_air:
                    # Fly to position
                    # We only try to fly if we are armed an in the air. This is convenient for ground testing.
                    await self.dm.fly_to(self.drone_name, gps=reference_image.gps, yaw=reference_image.drone_att[2])

                # Wait until camera parameters are set
                await cam_set_task
                # Point gimbal
                target_gimbal_pitch = reference_image.gimbal_att[1]
                target_gimbal_yaw = reference_image.gimbal_yaw_absolute

                if self._gimbal_max_pitch < target_gimbal_pitch < self._gimbal_min_pitch:
                    self.logger.info("Replay exceeding gimbal limit, skipping...")
                    continue

                await self.gimbal.set_gimbal_mode("lock")
                await self.gimbal.set_gimbal_angles(target_gimbal_pitch, target_gimbal_yaw)
                await asyncio.sleep(3)
                # Refine position and gimbal attitude based on previous image
                # TODO: Integrate from other repo, more eval on simulation first

                await self.do_capture(capture)
                # TODO: Check for replays that didn't work

                # Reset gimbal
                await self.gimbal.set_gimbal_mode("follow")
                await self.gimbal.set_gimbal_angles(0.1, 0.1) # Set a non-zero to make sure gimbal responds
            except Exception as e:
                self.logger.warning(f"Exception with replay for capture {capture.capture_id}")
                self.logger.debug(repr(e), exc_info=True)

    async def transfer(self, drive_letter: str):
        """ Load images from camera and do assorted metadata processing.

        Loads images from camera and stores them in a folder named after their capture ID. The capture information file
        is also rewritten to account for this. This is intended to be done after flights with the camera directly
        attached to the computer.

        :param drive_letter: Drive letter of the camera
        :return:
        """
        for capture in self.loaded_captures:
            # Create directory in capture folder
            img_dir = os.path.join(CAPTURE_DIR, "images", str(capture.capture_id))
            os.makedirs(img_dir, exist_ok=True)
            for image in capture.images:
                cam_path = image.file_location
                if cam_path.startswith("/mnt/ssd/"):
                    cam_file_dir = pathlib.Path(cam_path[9:])
                    image_file_name = cam_file_dir.name
                    mounted_path = pathlib.Path(f"{drive_letter}:").resolve().joinpath(cam_file_dir)
                    if mounted_path.exists():
                        # Move images from camera to capture folder
                        local_img_path = pathlib.Path(img_dir).joinpath(image_file_name)
                        shutil.move(mounted_path, local_img_path)
                        # Change directory in capture
                        image.file_location = local_img_path.as_posix()
                    else:
                        self.logger.warning(f"File {mounted_path} on camera doesn't exist!")
        self._save_captures_to_file(self.loaded_captures, filename=self.loaded_file, make_relative=True)

    def _move(self, captures: list[ENGELCaptureInfo], json_dir: pathlib.Path, target_dir: pathlib.Path):
        for capture in captures:
            self.logger.info(f"Processing capture {capture.capture_id}")
            img_dir = target_dir.joinpath("images").joinpath(str(capture.capture_id))
            img_dir.mkdir(exist_ok=True, parents=True)
            for i, image in enumerate(capture.images):
                old_img_file = pathlib.Path(image.file_location)
                if not old_img_file.is_absolute():
                    old_img_file = json_dir.joinpath(image.file_location)
                self.logger.debug(f"Moving image {capture.capture_id, i}")
                image.file_location = img_dir.joinpath(old_img_file.name).as_posix()
                shutil.copy2(old_img_file, img_dir)

    async def copy(self, capture_file: str, target_dir: str):
        target_dir = pathlib.Path(target_dir)
        target_dir.mkdir(exist_ok=True, parents=True)
        file_path = self._normal_dir_or_other_path(capture_file)
        file_name = file_path.name
        captures_to_move = self._load_captures_from_file(file_path)
        out_file = target_dir.joinpath(file_name)
        self.logger.info(f"Copying files from {file_path.resolve()} to {out_file.resolve()}")
        await asyncio.get_running_loop().run_in_executor(None, self._move, captures_to_move, file_path.parent, target_dir)
        self._save_captures_to_file(captures_to_move, filename=out_file, make_relative=True)
        self.logger.info("Done!")

    async def merge(self, other_files: list[str], output_file: str):
        captures = []
        for other_file in other_files:
            in_file = self._normal_dir_or_other_path(other_file)
            captures.extend(self._load_captures_from_file(in_file))
        out_file = self._normal_dir_or_other_path(output_file)
        self._save_captures_to_file(captures, out_file)

    # TODO: Move/copy image functions

    def _save_captures_to_file(self, captures, filename: str | pathlib.Path = None, merge_existing = False,
                               make_relative = False):
        """ Save all capture information to a file, images will have to be downloaded separately anyway. """
        if filename is None:
            timestamp = datetime.datetime.now(datetime.UTC)
            filename = f"engel_captures_{timestamp.hour}{timestamp.minute}{timestamp.second}-{timestamp.day}-{timestamp.month}-{timestamp.year}.json"
        file_path = self._normal_dir_or_other_path(filename)
        # If the file already exists, append new captures to old
        if merge_existing and file_path.exists():
            old_captures = self._load_captures_from_file(file_path)
            captures.extend(old_captures)

        if make_relative:
            for capture in captures:
                for image in capture.images:
                    image.file_location = "./" + pathlib.Path(image.file_location).relative_to(file_path.parent).as_posix()
        self.logger.info(f"Saving info to file {file_path}")
        with open(file_path, "wt") as f:
            output = [capture.to_json_dict() for capture in captures]
            json.dump(output, f, indent=2)

    async def save_captures_to_file(self, filename: str = None):
        return self._save_captures_to_file(self.captures, filename, merge_existing=True)

    def _load_captures_from_file(self, file_path: pathlib.Path):
        with open(file_path, "rt") as f:
            json_list = json.load(f)
            captures = [ENGELCaptureInfo.from_json_dict(capture_dict) for capture_dict in json_list]
        return captures

    def _normal_dir_or_other_path(self, filestr) -> pathlib.Path:
        if str(pathlib.Path(filestr).parent) == ".":
            file_path = pathlib.Path(CAPTURE_DIR).joinpath(filestr)
        else:
            file_path = pathlib.Path(filestr)
        return file_path

    async def load_captures_from_file(self, filename: str):
        """ Load capture information from a file for the purpose of replaying it. """

        file_path = self._normal_dir_or_other_path(filename)
        captures = self._load_captures_from_file(file_path)
        self.loaded_captures = captures
        self.loaded_file = filename
        self.logger.info(f"Loaded past captures from file {file_path}")

    async def reset(self):
        """ Clear capture info """
        # Resets variables as if the mission was just loaded. Useful for replay testing.
        self.captures = []
        self.loaded_captures = []
        self.loaded_file = None

    async def done(self):
        await self._done()  # Same issue as with _replay

    async def _done(self):
        """ Save any captures, reset and fly back to base and land"""
        await self.save_captures_to_file()
        await self.reset()
        await self.dm.fly_to(self.drone_name, waypoint=self.drones[self.drone_name].return_position)
        await self.dm.land(self.drone_name)
        await self.dm.disarm(self.drone_name)

    async def status(self):
        """ Print information, such as how many captures we have taken"""
        self.logger.info(f"Drone {self.drones}. {len(self.captures)} current, {len(self.loaded_captures)} old captures.")

    def _register_controller_inputs(self):
        PS4Mapping.add_method_to_button(3, self._do_capture_controller)  # Do capture on Triangle
        self._added_controller_buttons[3] = self._do_capture_controller
        PS4Mapping.add_method_to_button(2, self._swap_gimbal_axis)
        self._added_controller_buttons[2] = self._swap_gimbal_axis
        PS4Mapping.add_axis_method(self._get_gimbal_rate, [4, 5])
        self._added_controller_axis_methods.add(self._get_gimbal_rate)

    async def add_drones(self, names: list[str]):
        """ Adds camera and gimbal objects and stores current position for rtl"""
        if len(names) + len(self.drones) > 1:
            self.logger.warning("This mission only supports single drones!")
            return False
        self.logger.info(f"Adding drone {names} to ENGEL!")
        for name in names:
            try:
                gimbal_ok = await self.dm.gimbal.add_gimbals(name)
                cam_ok = await self.dm.camera.add_camera(name)
                rtl_pos = self.dm.drones[name].position_global
                cur_yaw = self.dm.drones[name].attitude[2]
                if gimbal_ok and cam_ok:
                    self.drones[name] = self.dm.drones[name]
                    self.drone_name = name
                    self.gimbal = self.dm.gimbal.gimbals[name]
                    self.camera = self.dm.camera.cameras[name]
                    rtl_pos[2] += self.rtl_height
                    self.launch_point = Waypoint(WayPointType.POS_GLOBAL, gps=rtl_pos, yaw=cur_yaw)
                    await self.gimbal.take_control()
                    # Set the gimbal mode to lock to ensure that we are recoding absolute pitch values from gimbal
                    await self.gimbal.set_gimbal_mode("lock")
                    await self.gimbal.set_gimbal_angles(0.1, 0.1)
                    self._gimbal_frequency = self.dm.drones[self.drone_name].position_update_rate
                    self._register_controller_inputs()
                    self.dm.controllers.set_drone(self.drone_name)
                    self.logger.info(f"Added drone {name} to mission!")
                    return True
                else:
                    self.logger.info(f"Couldn't add {name} to mission: Gimbal {'OK' if gimbal_ok else 'NOT OK'}, Cam {'OK' if cam_ok else 'NOT OK'}")
                    return False
            except KeyError:
                self.logger.error(f"No drone named {name}")
        return False

    async def remove_drones(self, names: list[str]):
        """ Removes camera and gimbal objects """
        for name in names:
            try:
                self.drones.pop(name)
                self.launch_point = None
                self.drone_name = None
                self.gimbal = None
                self.camera = None
                await self.dm.gimbal.remove_gimbal(name)
                await self.dm.camera.remove_camera(name)
            except KeyError:
                self.logger.error(f"No drone named {name}")

    async def mission_ready(self, drone: str):
        return drone in self.drones

    # Controller functions

    def _do_capture_controller(self):
        capture_task = asyncio.create_task(self.do_capture())
        capture_awaiter = asyncio.create_task(coroutine_awaiter(capture_task, self.logger))
        self._running_tasks.add(capture_task)
        self._running_tasks.add(capture_awaiter)

    def _swap_gimbal_axis(self):
        self.logger.info(f"Now controlling gimbal {'Pitch' if self._control_gimbal_pitch else 'Yaw'}")
        self._control_gimbal_pitch = not self._control_gimbal_pitch

    def _get_gimbal_rate(self, values):
        # If the trigger is depressed enough to be positive only:
        l_trigger, r_trigger = values
        l_value = self._trigger_response_function(l_trigger)
        r_value = self._trigger_response_function(r_trigger)
        final_value = r_value - l_value
        self._gimbal_rate = final_value * self._gimbal_max_rate

    def _trigger_response_function(self, value):
        # Controllers start at -1 and go to +1
        value = (value + 1) / 2
        if value < 0.05:
            value = 0
        value *= value
        if value > 1:
            value = 1
        return value

    async def _set_gimbal_rates_controller(self):
        controlling_rates = False
        while True:
            try:
                if abs(self._gimbal_rate) > 0.05:
                    controlling_rates = True
                    yaw_rate = 0
                    pitch_rate = 0
                    if self._control_gimbal_pitch:
                        yaw_rate = self._gimbal_rate
                    else:
                        pitch_rate = self._gimbal_rate
                    await self.gimbal.set_gimbal_rates(pitch_rate, yaw_rate)
                else:
                    if controlling_rates:
                        await self.gimbal.set_gimbal_rates(0, 0)
                        controlling_rates = False
            except Exception as e:
                self.logger.warning("Exception setting gimbal rates from controller!")
                self.logger.debug(repr(e), exc_info = True)
            await asyncio.sleep(1/self._gimbal_frequency)

def _roll_pitch_compensation(gimbal_yaw, drone_roll, drone_pitch):
    return 0  # Seems like our gimbal might already correct for drone pitch and roll when the gimbal is in locked mode?
    #return math.sin(gimbal_yaw)*drone_roll + math.cos(gimbal_yaw) * drone_pitch
