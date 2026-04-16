""" Mission for ENGEL data collection
Capture images and combine with weather and position info from drone, storing them
Functions to retake the same position as in a previous image and take another image.
"""
import asyncio
import logging
import math
import pathlib
from asyncio import StreamReader, StreamWriter
from collections.abc import Callable
import socket
from queue import Queue, Empty

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
            "test-init": self.init_test,
            "connect-command": self.connect_command,
            "connect-data": self.connect_data,
            "test-start": self.start_test,
            "test-command": self.test_command,
            "test-stop": self.stop_test,
            "test-close": self.close_test,
            "test-rates": self.rate_test,
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

        # Controller stuff
        self._added_controller_buttons: dict[int, Callable] = {}
        self._added_controller_axis_methods: set[Callable] = set()

        # Gimbal is controlled with triggers, press more to move more. Press square to switch between pitch and yaw control.
        self._gimbal_rate = 0
        self._gimbal_max_rate = 10  # Maximum rotation rate of the gimbal in degrees per second
        self._control_gimbal_pitch = True  # If false, control gimbal yaw instead
        self._gimbal_frequency = 20  # Default frequency until we get an actual drone

        # Position refinement stuff
        self.correction_algo: PositionCorrectionHandler | None = None
        self.refining = False
        self.translation_shift = None
        self.rotation_shift = None
        self.rotation_target = None
        self._loop = asyncio.get_running_loop()

    async def close(self):
        for button, func in self._added_controller_buttons.items():
            PS4Mapping.remove_method_from_button(button, func)
        for func in self._added_controller_axis_methods:
            PS4Mapping.remove_axis_method(func)
        await self.close_test()
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
        await self.drones[self.drone_name].execute_task(self._replay_captures())

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
                await self.gimbal.set_gimbal_angles(0.0, 0.0)
                if drone.is_armed and drone.in_air:
                    # Fly to position
                    # We only try to fly if we are armed an in the air. This is convenient for ground testing.
                    await self.dm.fly_to(self.drone_name, gps=reference_image.gps, yaw=reference_image.drone_att[2])

                # Wait until camera parameters are set
                await cam_set_task
                # Point gimbal
                # We want to point the gimbal in an absolute direction, so we have to account for drone attitude
                reference_dronepitch_corrected = _roll_pitch_compensation(reference_image.gimbal_att[2]*math.pi/180,
                                                                          reference_image.drone_att[0],
                                                                          reference_image.drone_att[1])
                target_total_pitch = reference_image.gimbal_att[1] + reference_dronepitch_corrected

                # Initial coarse adjustment with time for gimbal to move
                # Pitch contribution of drone at the same gimbal yaw as original picture
                # The gimbal yaw might change a little over time, but good enough for first pass
                current_dronepitch_corrected = _roll_pitch_compensation(reference_image.gimbal_att[2]*math.pi/180,
                                                                        drone.attitude[0],
                                                                        drone.attitude[1])

                target_gimbal_pitch = target_total_pitch - current_dronepitch_corrected
                target_gimbal_yaw = reference_image.gimbal_yaw_absolute

                await self.gimbal.set_gimbal_mode("lock")
                await self.gimbal.set_gimbal_angles(target_gimbal_pitch, target_gimbal_yaw)
                await asyncio.sleep(3)
                # Fine gimbal adjustment
                while (abs(self.gimbal.pitch - target_gimbal_pitch) > 0.25
                       or abs(self.gimbal.yaw_absolute - target_gimbal_yaw) > 0.25):
                    current_dronepitch_corrected = _roll_pitch_compensation(self.gimbal.yaw*math.pi/180,
                                                                            drone.attitude[0],
                                                                            drone.attitude[1])
                    target_gimbal_pitch = target_total_pitch - current_dronepitch_corrected  # Recompute pitch target for possible drone change in pitch
                    await self.gimbal.set_gimbal_angles(target_gimbal_pitch, target_gimbal_yaw)
                    await asyncio.sleep(0.2)
                # Refine position and gimbal attitude based on previous image
                # TODO: Integrate from other repo, more eval on simulation first
                # TODO: Process: While the other algorithm is running, grab updates from there and use whatever the
                #  latest value is here
                while self.refining:
                    _, pitch_rate, yaw_rate = self.rotation_shift
                    await self.gimbal.set_gimbal_rates(pitch_rate, yaw_rate)
                    #await self.dm.move(self.drone_name,
                    #                   offset=self.translation_shift*drone.position_update_rate,
                    #                   use_gps=False,
                    #                   tolerance=0.05)

                await self.do_capture(capture)
                # TODO: Check for replays that didn't work

                # Reset gimbal
                await self.gimbal.set_gimbal_mode("follow")
                await self.gimbal.set_gimbal_angles(0.0, 0.0)
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
        await self.drones[self.drone_name].execute_task(self._done())

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
        PS4Mapping.add_method_to_button(2, self._swap_gimbal_axis)
        PS4Mapping.add_axis_method(self._get_gimbal_rate, [4, 5])
        self._added_controller_axis_methods.add(self._get_gimbal_rate)
        self._added_controller_buttons[3] = self._do_capture_controller
        self._added_controller_buttons[2] = self._swap_gimbal_axis

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
                    # Set the gimbal mode to follow and have it point straight forward to prevent drone motion from
                    # moving the gimbal into yaw limits.
                    await self.gimbal.set_gimbal_mode("follow")
                    await self.gimbal.set_gimbal_angles(0.0, 0.0)
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

    def correction_callback(self, parsed_message):
        if parsed_message:
            try:
                rotation = parsed_message["rotation"]  # roll pitch yaw in degrees per second
                translation = parsed_message["translation"]  # x y z in cm / s
                rotation_target = parsed_message["target"]
                self.logger.info(parsed_message)
                self.translation_shift = translation / 100
                self.rotation_target = rotation_target
                self.rotation_shift = rotation

                # Do this here for testing
                self.logger.info(f"Received message {parsed_message}")
                _, pitch_rate, yaw_rate = self.rotation_shift
                gimbal_task = asyncio.run_coroutine_threadsafe(self.gimbal.set_gimbal_rates(pitch_rate, yaw_rate), self._loop)
                gimbal_awaiter_task = asyncio.run_coroutine_threadsafe(coroutine_awaiter(gimbal_task, self.logger), self._loop)
            except Exception as e:
                self.logger.warning("Exception forward position message! See log for details")
                self.logger.debug(repr(e), exc_info=True)

    async def rate_test(self):
        start_time = time.time()
        while True:
            try:
                await asyncio.sleep(1/10)
                t = (time.time()-start_time) * math.pi / 180
                p_rate = math.sin(t*20)*5
                y_rate = math.cos(t*20)*5

                gimbal_task = asyncio.create_task(self.gimbal.set_gimbal_rates(p_rate, y_rate))
                gimbal_awaiter_task = asyncio.create_task(coroutine_awaiter(gimbal_task, self.logger))
            except Exception as e:
                self.logger.warning("Exception setting gimbal rates!")
                self.logger.debug(repr(e), exc_info=True)

    async def init_test(self, ip: str = "127.0.0.1", command_port: int = 9020, data_port: int = 9010):
        self.logger.info("Performing setup for position correction algorithm...")
        self.correction_algo = PositionCorrectionHandler()
        self.correction_algo.message_callback = self.correction_callback
        await self.connect_command(ip, command_port)
        await self.connect_data(ip, data_port)
        self.logger.info("Setup for position correction test completed.")

    async def connect_command(self, ip: str = "127.0.0.1", port: int = 9020):
        await self.correction_algo.connect_command_channel(ip, port)

    async def connect_data(self, ip: str = "127.0.0.1", port: int = 9010):
        await self.correction_algo.connect_data_channel(ip, port)

    async def start_test(self):
        await self.correction_algo.start()

    async def test_command(self, cmd: str):
        try:
            await self.correction_algo.send_command(cmd)
        except Exception as e:
            self.logger.warning(f"Couldn't send command {cmd} due to an exception: {repr(e)}")
            self.logger.debug(repr(e), exc_info=True)

    async def stop_test(self):
        await self.correction_algo.stop()

    async def close_test(self):
        if self.correction_algo is not None:
            await self.correction_algo.stop()
            await self.correction_algo.close()
            self.correction_algo = None


def _roll_pitch_compensation(gimbal_yaw, drone_roll, drone_pitch):
    return math.sin(gimbal_yaw)*drone_roll + math.cos(gimbal_yaw) * drone_pitch


class PositionCorrectionHandler:
    def __init__(self):
        # Initialise the channel classes
        self.running = False
        self.data_handler = DataChannel()
        self.command_handler = CommandChannel()
        self.message_callback = None
        self.handler_task = None
        self.logger = logging.getLogger("Manager.CorrectionAlgorithm")
        self.loop = asyncio.get_running_loop()
        self.valid_commands = ["start", "stop", "pause", "resume", "rotation_only", "translation_only", "status", "quit"]

    async def connect_command_channel(self, ip: str = "127.0.0.1", port: int = 9020):
        try:
            await self.command_handler.connect(ip, port)
            self.logger.info("Connected command channel")
        except ConnectionRefusedError as e:
            self.logger.warning(f"Couldn't connect to command channel: Connection refused! {repr(e)}")
            raise

    async def connect_data_channel(self, ip: str = "127.0.0.1", port: int = 9010):
        try:
            await self.data_handler.connect(ip, port)
            self.logger.info("Connected data channel")
        except ConnectionRefusedError as e:
            self.logger.warning(f"Couldn't connect to command channel: Connection refused! {repr(e)}")
            raise

    async def send_command(self, cmd):
        assert cmd in self.valid_commands, f"Invalid command {cmd}, must be one of {self.valid_commands}"
        self.logger.info(f"Sending command {cmd} to correction algo")
        await self.command_handler.send_command(cmd)

    async def start(self):
        # Start processing all the stuff
        self.logger.info("Starting correction algorithm...")
        self.data_handler.processing = True
        self.running = True
        self.handler_task = self.loop.run_in_executor(None, self._data_thread)
        await self.send_command("start")

    async def stop(self):
        self.logger.info("Stopping correction algorithm...")
        self.data_handler.processing = False
        try:
            await self.send_command("stop")
        except ConnectionAbortedError:
            self.logger.info("Couldn't send stop command: Connection aborted")

    def _data_thread(self):
        while self.running:
            try:
                message = self.data_handler.get_from_queue()
                if message:
                    if self.message_callback is not None:
                        data = self.data_handler.parse_motion_command(message)
                        self.loop.call_soon_threadsafe(self.message_callback, data)
            except Exception as e:
                self.logger.warning(repr(e), exc_info=True)

    async def close(self):
        self.logger.info("Closing correction algorithm...")
        self.running = False
        self.command_handler.close()
        if self.handler_task is not None:
            self.handler_task.cancel()
        self.data_handler.close()
        self.logger.info("Closed correction algorithm")


class CommandChannel:
    def __init__(self):
        self.ip = None
        self.port = None
        self.conn: tuple[StreamReader, StreamWriter] | None = None
        self.logger = logging.getLogger("Manager.CorrectionAlgorithm")

    async def connect(self, ip, port) -> None:
        self.ip = ip
        self.port = port
        if self.conn:
            return
        self.conn = await asyncio.open_connection(ip, port, family=socket.AF_INET)
        self.logger.info(f"[tcp] connected to {self.ip}:{self.port}")

    def close(self) -> None:
        if self.conn:
            try:
                self.conn[1].close()
            except Exception:
                pass
            self.conn = None

    async def send_command(self, cmd: str) -> None:
        msg = (cmd + "\n").encode("utf-8")
        if not self.conn:
            self.logger.warning("Can't send command, not connected!")
        else:
            self.conn[1].write(msg)
            await self.conn[1].drain()


class DataChannel:
    def __init__(self):
        self.ip = None
        self.port = None
        self.conn: tuple[StreamReader, StreamWriter] | None = None
        self.logger = logging.getLogger("Manager.CorrectionAlgorithm")
        self.message_queue = Queue()
        self.running = False  # Listening to incoming messages
        self.processing = False  # Passing incoming messages onward
        self.data_task = None

    async def connect(self, ip, port) -> None:
        self.ip = ip
        self.port = port
        if self.conn:
            return
        self.conn = await asyncio.open_connection(ip, port, family=socket.AF_INET)
        self.running = True
        self.logger.info(f"[tcp] connected to {self.ip}:{self.port}")
        self.data_task = asyncio.create_task(self._receive_msg())

    def close(self) -> None:
        try:
            if self.data_task is not None:
                self.data_task.cancel()
            self.running = False
            self.processing = False
            if self.conn:
                try:
                    self.conn[1].close()
                except Exception:
                    pass
                self.conn = None
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)

    def start(self):
        self.processing = True

    async def _receive_msg(self):
        """Receive messages from a connected client and add them to the queue."""
        self.logger.info("Listening for messages...")
        while self.running:
            try:
                message = (await self.conn[0].readline()).decode("utf-8")
                if self.processing and message:
                    self.logger.info(f"Got a message, current queue size: {self.message_queue.qsize()}")
                    self.message_queue.put(message)
                if self.conn[0].at_eof():
                    break
            except Exception as e:
                self.logger.warning("Exception receiving data over data channel!")
                self.logger.debug(repr(e), exc_info=True)

    def get_from_queue(self):
        """Get and remove the next message from the queue."""
        try:
            return self.message_queue.get(timeout=1)
        except Empty:
            return None
        except Exception as e:
            self.logger.error(f"Error getting from queue: {e}")
            self.logger.debug(f"{repr(e)}", exc_info=True)
            return None

    def parse_motion_command(self, message):
        """Parse motion command message into rotation, translation, and target components.

        Expected format: rotation_x,rotation_y,rotation_z,translation_x,translation_y,translation_z,target
        Where target: 0 = gimbal, 1 = drone

        Returns: dict with keys 'rotation', 'translation', 'target' or None if parsing fails
        """
        try:
            # Clean the message (remove newlines and extra whitespace)
            message = message.strip()

            # Split by comma
            values = message.split(',')

            if len(values) < 7:
                self.logger.warning(f"Error: Expected at least 7 values, got {len(values)}")
                return None

            # Parse rotation (first 3 values)
            rotation = np.asarray([float(values[0]), float(values[1]), float(values[2])])

            # Parse translation (next 3 values)
            translation = np.asarray([float(values[3]), float(values[4]), float(values[5])])

            # Parse target flag (last value)
            target_flag = int(values[6])
            target = 'drone' if target_flag == 1 else 'gimbal'

            result = {
                'rotation': rotation,
                'translation': translation,
                'target': target,
                'target_flag': target_flag
            }

            return result
        except (ValueError, IndexError) as e:
            self.logger.error(f"Error parsing motion command: {e}")
            self.logger.debug(f"{repr(e)}", exc_info=True)
            return None