from __future__ import annotations

import asyncio
import inspect
import json
import logging
from lxml import etree
import math
import numpy as np
import os
import struct
import time
from typing import Type, Any, Callable
import requests

import mavsdk
from mavsdk.asyncio import Mavsdk, Configuration, System
from mavsdk.asyncio.plugins.telemetry import StatusTextType, TelemetryAsync
from mavsdk.asyncio.plugins.action import ActionAsync, ActionError, OrbitYawBehavior
from mavsdk.asyncio.plugins.offboard import PositionNedYaw, PositionGlobalYaw, VelocityNedYaw, AccelerationNed, OffboardError, \
    VelocityBodyYawspeed, OffboardAsync
from mavsdk.asyncio.plugins.manual_control import ManualControlError, ManualControlAsync
from mavsdk.asyncio.plugins.mavlink_direct import MavlinkMessage, MavlinkDirectAsync
from mavsdk.asyncio.plugins.param import ParamAsync
from mavsdk.asyncio.plugins.camera import CameraAsync, CameraError
from mavsdk.asyncio.plugins.gimbal import GimbalAsync, GimbalError

from dronemanager.drone import DroneComProtocol
from dronemanager.drone import Battery, Drone, DroneConfig, DroneParams, FlightMode, FixType
from dronemanager.navigation.core import WayPointType, Waypoint
from dronemanager.navigation.directtargetgenerator import DirectTargetGenerator
from dronemanager.navigation.ruckigfollower import RuckigOfflineFollower
from dronemanager.plugins.camera import Camera, CameraParameter
from dronemanager.plugins.gimbal import Gimbal, YawMode, ControlMode, SendMode
from dronemanager.utils import CACHE_DIR, coroutine_awaiter, parse_address, relative_gps


class MAVSDKComm(DroneComProtocol):

    NAME: str = "mav"
    SUPPORTS: dict[str, type] = {}

    def __init__(self, logger, system_id: int, component_id: int):
        super().__init__(logger)
        self.system_id = system_id
        self.component_id = component_id
        mavsdk.logging.log_subscribe(self._log_callback)
        self.mavsdk = Mavsdk(Configuration.create_manual(self.system_id, self.component_id, True))

        # Passively discover new systems, including after new connections are added.
        system_checker_task = asyncio.create_task(self.new_system_handler())
        system_checker_awaiter = asyncio.create_task(coroutine_awaiter(system_checker_task, self.logger))
        self.running_tasks.add(system_checker_task)
        self.running_tasks.add(system_checker_awaiter)

        # Individual conn and system attributes
        self.conn_handles: dict[str, Any] = {}
        self.systems: dict[int, MAVSDKDrone] = {}

    async def add_connection(self, connection_string) -> bool:
        scheme, loc, appendix = parse_address(string=connection_string)
        if scheme == "udp":
            if loc == "0.0.0.0" or loc == "127.0.0.1":
                scheme = "udpin"
            else:
                scheme = "udpout"
        elif scheme == "serial":
            pass
        else:
            raise NotImplementedError("Only UDP and serial connections are implemented for MAVSDK.")
        mavsdk_conn_string = f"{scheme}://{loc}:{appendix}"
        if mavsdk_conn_string in self.conn_handles:
            self.logger.debug(f"Connection {mavsdk_conn_string} already exists")
            return False
        self.logger.info(f"Adding connection {mavsdk_conn_string}")
        conn_handle = None
        try:
            conn_handle = await self.mavsdk.add_any_connection(mavsdk_conn_string)
            self.conn_handles[mavsdk_conn_string] = conn_handle
            return True
        except Exception as e:
            self.logger.debug(repr(e), exc_info=True)
            if conn_handle is not None:
                await self.mavsdk.remove_connection(conn_handle)
            raise

    async def remove_connection(self, identifier):
        raise NotImplementedError("Manually removing connections is not supported, as there is no good way to prevent "
                                  "live drones from being disconnected with it.")
        # This function is spicy, as there is no robust way to associate drones and connections
        #await self.mavsdk.remove_connection(self.conn_handles[identifier])

    async def discovered_drones(self):
        return [sysobj for sysobj in self.systems.values()]

    async def discovered_other(self):
        raise NotImplementedError

    async def add_drone(self, drone_name: str, system_id: str, config: DroneConfig):
        drone = self.systems[int(system_id)]
        if drone.initialized:
            self.logger.warning(f"Couldn't add drone {drone_name} for ID {system_id} as a drone with that ID already "
                                f"exists: {drone.name}!")
            return None
        drone.start(drone_name, config=config)
        return drone

    async def remove_drone(self, drone_name: str):
        """Dummy implementation, as MAVSDK does not keep track of the drone objects."""
        return True

    async def new_system_handler(self):
        async for _ in self.mavsdk.on_new_system():
            for system in await self.mavsdk.get_systems():
                sys_id = await system.get_system_id()
                if sys_id in self.systems:
                    # Check that the system objects are identical
                    if system != self.systems[sys_id].system:
                        pass
                        #self.logger.debug(f"Got two different system objects with the same system ID: {sys_id}!")
                        #  It appears that the system objects change everytime this list gets updated, which happens everytime a system is discovered or lost
                        #  Seems to spawn a new system object, but also, the original system object maybe still works? At least the connected state still updates.
                        #  Should we deconstruct and reconstruct the drone objects everytime the hearbeats time out? Only the plugin objects?
                else:
                    await self.add_system(system)

    async def add_system(self, system: System):
        sys_id = await system.get_system_id()
        self.logger.info(f"Discovered new system with ID {sys_id}.")
        drone = MAVSDKDrone(self, system, str(sys_id))
        self.systems[sys_id] = drone

    async def discovered_systems(self):
        return self.systems.values()

    def _log_callback(self, level, message, file, line):
        if level == 0:
            log_level = logging.DEBUG
        elif level == 1:
            log_level = logging.INFO
        elif level == 2:
            log_level = logging.WARN
        else:
            log_level = logging.ERROR
        self.logger.log(log_level, f"{message}")
        return True

    async def close(self):
        # Check that we don't have any drones connected, do not close if we do
        can_close = True
        for system in self.systems.values():
            if system.initialized:
                can_close = False
                break
        if can_close:
            super().close()
            while len(self.conn_handles) > 0:
                _, conn_handle = self.conn_handles.popitem()
                await self.mavsdk.remove_connection(conn_handle)
            self.mavsdk.destroy()


class MAVSDKDrone(Drone[MAVSDKComm]):

    VALID_FLIGHTMODES = {"hold", "offboard", "return", "land", "takeoff", "position", "altitude"}
    # This attribute is for checking which flight modes can be changed into manually
    VALID_SETPOINT_TYPES = {WayPointType.POS_NED,
                            WayPointType.POS_VEL_NED,
                            WayPointType.POS_VEL_ACC_NED,
                            WayPointType.VEL_NED,
                            WayPointType.POS_GLOBAL}
    # What type of path setpoints this classes fly_<> commands can follow. This limits what Trajectory generators
    # can be used.

    mavdirect: MavlinkDirectAsync
    mavparam: ParamAsync
    mavtelem: TelemetryAsync
    mavaction: ActionAsync
    mavoffboard: OffboardAsync
    mavmanual: ManualControlAsync
    mavcamera: CameraAsync
    mavgimbal: GimbalAsync

    def __init__(self, comm_protocol: MAVSDKComm, system: System, drone_identifier: str):
        super().__init__(comm_protocol, drone_identifier)
        self.system_id = int(drone_identifier)
        self.system: System = system
        self.fc_component_id: int = -1  # Actual component ids are always positive, so this is a safe "invalid" number.

        self._is_connected: bool = False
        self._is_armed: bool = False
        self._flightmode: FlightMode = FlightMode.UNKNOWN
        self._in_air: bool = False
        self._gps_info: FixType = FixType.NO_GPS
        self._position_g: np.ndarray = np.zeros((4,), dtype=np.float64)     # Latitude, Longitude, AMSL, Relative altitude to takeoff
        self._position_ned: np.ndarray = np.zeros((3,), dtype=np.float64)   # NED
        self._velocity: np.ndarray = np.zeros((3,), dtype=np.float64)       # NED
        self._attitude: np.ndarray = np.zeros((3,), dtype=np.float64)       # Roll, pitch and yaw, with positives right up and right.
        self._heading: float = math.nan
        self._batteries: dict[int, Battery] = {}

        self._have_drone_info = asyncio.Event()

        self._message_callbacks: dict[str, set] = dict()
        self._message_listeners: dict[tuple[str, int], set] = dict()
        self._ack_listeners: dict[tuple[int, int, int, int, int], list] = dict()

        # Set up the connect check
        self._running_tasks.add(asyncio.create_task(self._connect_check()))

    def start(self, name, config: DroneConfig):
        super().start(name, config)

        # Init all the mav plugins
        # Do this here so that we can destroy them when shutdown happens and avoid issues with mavsdk shutdown.
        self.mavdirect = MavlinkDirectAsync(self.system)
        self.mavparam = ParamAsync(self.system)
        self.mavtelem = TelemetryAsync(self.system)
        self.mavaction = ActionAsync(self.system)
        self.mavoffboard = OffboardAsync(self.system)
        self.mavmanual = ManualControlAsync(self.system)
        self.mavcamera = CameraAsync(self.system)
        self.mavgimbal = GimbalAsync(self.system)

        # Init path generator
        try:
            # self.path_generator = GMP3Generator(self, 1/self.position_update_rate, self.logger)
            self.path_generator = DirectTargetGenerator(self, self.logger, WayPointType.POS_NED)
        except Exception as e:
            self.logger.error("Couldn't initialize path generator due to an exception!")
            self.logger.debug(repr(e), exc_info=True)
        # Init path follower
        try:
            self.path_follower = RuckigOfflineFollower(self, self.logger, 1 / self.position_update_rate,
                                                       WayPointType.POS_VEL_ACC_NED,
                                                       max_vel=self.config.max_h_vel,
                                                       max_down_vel=self.config.max_down_vel,
                                                       max_up_vel=self.config.max_up_vel, max_acc=self.config.max_h_acc,
                                                       max_v_acc=self.config.max_v_acc, max_jerk=self.config.max_h_jerk,
                                                       max_v_jerk=self.config.max_v_jerk,
                                                       max_yaw_vel=self.config.max_yaw_vel,
                                                       max_yaw_acc=self.config.max_yaw_acc,
                                                       max_yaw_jerk=self.config.max_yaw_jerk)
            # self.path_follower = DirectSetpointFollower(self, self.logger, 1/self.position_update_rate,
            #                                                  WayPointType.POS_VEL_NED)
        except Exception as e:
            self.logger.error("Couldn't initialize path follower due to an exception!")
            self.logger.debug(repr(e), exc_info=True)

        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized Drone {self.name}, {self.__class__.__name__}:\n   {attr_string}")

        self._running_tasks.add(asyncio.create_task(self._process_messages()))
        self._get_drone_info()
        self._running_tasks.add(asyncio.create_task(self._configure_message_rates()))
        self._schedule_update_tasks()

        param_load_task = asyncio.create_task(self.load_parameters())
        param_load_task_awaiter = asyncio.create_task(coroutine_awaiter(param_load_task, self.logger))
        self._running_tasks.add(param_load_task)
        self._running_tasks.add(param_load_task_awaiter)
        self.initialized = True

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_armed(self) -> bool:
        return self._is_armed

    @property
    def flightmode(self) -> FlightMode:
        return self._flightmode

    @property
    def in_air(self) -> bool:
        return self._in_air

    @property
    def fix_type(self) -> FixType:
        return self._gps_info

    @property
    def position_global(self) -> np.ndarray[np.float64]:
        return self._position_g[:3]

    @property
    def altitude_above_takeoff(self) -> float:
        return self._position_g[3]

    @property
    def position_ned(self) -> np.ndarray[np.float64]:
        return self._position_ned

    @property
    def velocity(self) -> np.ndarray[np.float64]:
        return self._velocity

    @property
    def speed(self) -> float:
        return np.linalg.norm(self._velocity, 2)

    @property
    def attitude(self) -> np.ndarray[np.float64]:
        return self._attitude

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def batteries(self) -> dict[int, Battery]:
        return self._batteries

    def add_message_callback(self, message_name, callback):
        assert not inspect.isawaitable(callback), "Message callbacks can't be coroutines!"
        if message_name in self._message_callbacks:
            self._message_callbacks[message_name].add(callback)
        else:
            self._message_callbacks[message_name] = {callback}

    def remove_message_callback(self, message_name, callback):
        if message_name in self._message_callbacks:
            self._message_callbacks[message_name].remove(callback)

    async def send_message(self, msg: MavlinkMessage):
        try:
            return await self.mavdirect.send_message(msg)
        except Exception as e:
            self.logger.warning(f"Exception sending mavlink message: {repr(e)}!")
            self.logger.debug(repr(e), exc_info=True)

    def listen_next_message(self, msg_name: str, target_component: int):
        fut = asyncio.Future()
        msg_tuple = (msg_name, target_component)
        if msg_tuple in self._message_listeners:
            self._message_listeners[msg_tuple].add(fut)
        else:
            self._message_listeners[msg_tuple] = {fut}
        return fut

    def _listen_ack(self, command_msg_id, target_component_id):
        fut = asyncio.Future()
        msg_tuple = (command_msg_id, self.system_id, target_component_id, self.comm.system_id, self.comm.component_id)
        if msg_tuple in self._ack_listeners:
            self._ack_listeners[msg_tuple].append(fut)
        else:
            self._ack_listeners[msg_tuple] = [fut]
        return fut

    def send_command(self, command_msg_id, target_component_id, confirmation, param1: int | float | None = 0,
                     param2: int | float | None = 0, param3: int | float | None = 0, param4: int | float | None = 0,
                     param5: int | float | None = 0, param6: int | float | None = 0, param7: int | float | None = 0):
        fields = {
            "target_system": self.system_id,
            "target_component": target_component_id,
            "command": command_msg_id,
            "confirmation": confirmation,
            "param1": param1,
            "param2": param2,
            "param3": param3,
            "param4": param4,
            "param5": param5,
            "param6": param6,
            "param7": param7,
        }
        self.logger.debug(f"Sending command {json.dumps(fields)}")
        msg = MavlinkMessage("COMMAND_LONG", self.comm.system_id, self.comm.component_id, self.system_id, target_component_id,
                             fields_json=json.dumps(fields))
        send_task = asyncio.create_task(self.send_message(msg))
        self._running_tasks.add(send_task)
        self._running_tasks.add(coroutine_awaiter(send_task, self.logger))
        return self._listen_ack(command_msg_id, target_component_id)

    def request_message(self, msg_id: int, message_name: str, target_component_id: int):
        listen_result = self.listen_next_message(message_name, target_component_id)
        request_ack = self.send_command(512, target_component_id, 0, param1=msg_id, param7=1)
        return listen_result, request_ack

    async def _process_messages(self):
        try:
            async for message in self.mavdirect.subscribe_message(""):
                if self.config.log_telemetry:
                    self.telem_logger.debug(f"Received {message.message_name} from {message.system_id, message.component_id} to {message.target_system_id, message.target_component_id}: {message.fields_json}")
                # Check that the message is for us
                if (self.system_id is None or message.system_id == self.system_id) \
                        and (message.target_system_id == 0 or message.target_system_id == self.comm.system_id) \
                        and (message.target_component_id == 0 or message.target_component_id == self.comm.component_id):
                    # Do command acks
                    if message.message_name == "COMMAND_ACK":
                        fields = json.loads(message.fields_json, strict=False)
                        msg_tuple = (fields["command"], message.system_id, message.component_id, fields["target_system"], fields["target_component"])
                        if msg_tuple in self._ack_listeners:
                            futs = self._ack_listeners[msg_tuple]
                            if len(futs) > 0:
                                fut = futs.pop(0)
                                fut.set_result(fields["result"])

                    # Do message waiters
                    msg_tuple = (message.message_name, message.component_id)
                    if msg_tuple in self._message_listeners:
                        futs = self._message_listeners.pop(msg_tuple)
                        for fut in futs:
                            fut.set_result(message)

                    # Do callbacks
                    if message.message_name in self._message_callbacks:
                        for callback in self._message_callbacks[message.message_name]:
                            try:
                                callback(message)
                            except Exception as e:
                                self.logger.error(f"Couldn't perform a message callback {callback} for message "
                                                  f"{message.message_name} from {message.system_id, message.component_id}: "
                                                  f"{message.fields_json}")
                                self.logger.debug(repr(e), exc_info=True)
        except Exception as e:
            self.logger.debug("An exception occurred in the _process_message function. If this happened during "
                              "disconnection, it is not concerning")
            self.logger.debug(repr(e), exc_info=True)

    def _get_drone_info_callback(self, message):
        fields = json.loads(message.fields_json)
        if "autopilot" in fields:
            if fields["autopilot"] == 8:
                self.fc_component_id = message.component_id
            match fields["autopilot"]:
                case 3:
                    self.autopilot = "ArduPilot"
                    self._have_drone_info.set()
                case 12:
                    self.autopilot = "PX4"
                    self._have_drone_info.set()

    def _get_drone_info(self):
        self.add_message_callback("HEARTBEAT", self._get_drone_info_callback)

    async def load_parameters(self):
        self.logger.info(f"Loading parameters...")
        try:
            #await self._mavparam.select_component(1, ProtocolVersion.V1)
            parameters = await self.mavparam.get_all_params()
            raw_params = {}
            for param in parameters.int_params:
                raw_params[param.name] = (param.value, int)
            for param in parameters.float_params:
                raw_params[param.name] = (param.value, float)
            for param in parameters.custom_params:
                raw_params[param.name] = (param.value, str)
            drone_params = DroneParams(raw_params)
            try:
                await asyncio.wait_for(self._have_drone_info.wait(), 5)
                if self.autopilot == "PX4":
                    drone_params.max_h_vel = drone_params.raw['MPC_XY_VEL_MAX'][0]
                    drone_params.max_up_vel = drone_params.raw['MPC_Z_VEL_MAX_UP'][0]
                    drone_params.max_down_vel = drone_params.raw['MPC_Z_VEL_MAX_DN'][0]
                    drone_params.max_yaw_rate = drone_params.raw['MPC_MAN_Y_MAX'][0]
                elif self.autopilot == "Ardupilot":
                    drone_params.max_h_vel = drone_params.raw['LOIT_SPEED'][0] * 10  # cm/s
                    drone_params.max_up_vel = drone_params.raw['PILOT_SPEED_UP'][0] * 10
                    drone_params.max_down_vel = drone_params.raw['PILOT_SPEED_DN'][0] * 10
                    drone_params.max_yaw_rate = drone_params.raw['PILOT_Y_RATE'][0]  # degrees per second
                    if drone_params.max_down_vel == 0:
                        drone_params.max_down_vel = drone_params.max_up_vel
                else:
                    self.logger.warning("Couldn't parse parameters for this autopilot, drone speeds might "
                                        "not work properly.")
            except asyncio.TimeoutError:
                self.logger.warning("Couldn't get basic drone information, can't parse drone parameters! The "
                                    "connection is likely too poor!")
            self.drone_params = drone_params
            self.logger.info(f"Loaded parameters!")
        except Exception as e:
            self.logger.warning("Couldn't load parameters!")
            self.logger.debug(repr(e), exc_info=True)
        # TODO: Should probably do some kind of parameter checking: If the drone velocity and acceleration parameters are
        #  lower than the ones set in our config, algorithms might not work properly

    async def shutdown(self, force=False):
        self.clear_queue()
        self.cancel_action()
        if force or not self._is_armed:
            if force:
                self.logger.debug("Force disconnecting from drone...")
            # Deactivate path follower
            if self.path_follower:
                if self.path_follower.is_active:
                    await self.path_follower.deactivate()
                self.path_follower.close()

            # Cancel tasks here, to prevent them from trying to use destroyed mav plugins.
            await super().shutdown(force=force)

            self.mavdirect.destroy()
            self.mavparam.destroy()
            self.mavtelem.destroy()
            self.mavaction.destroy()
            self.mavoffboard.destroy()
            self.mavmanual.destroy()
            self.mavcamera.destroy()
            self.mavgimbal.destroy()

            self.initialized = False
        else:
            self.logger.warning("Can't disconnect from an armed drone!")

    def _schedule_update_tasks(self) -> None:
        self._running_tasks.add(asyncio.create_task(self._arm_check()))
        self._running_tasks.add(asyncio.create_task(self._flightmode_check()))
        self._running_tasks.add(asyncio.create_task(self._inair_check()))
        self._running_tasks.add(asyncio.create_task(self._gps_check()))
        self._running_tasks.add(asyncio.create_task(self._g_pos_check()))
        self._running_tasks.add(asyncio.create_task(self._vel_rpos_check()))
        self._running_tasks.add(asyncio.create_task(self._att_check()))
        self._running_tasks.add(asyncio.create_task(self._battery_check()))
        self._running_tasks.add(asyncio.create_task(self._status_check()))
        self._running_tasks.add(asyncio.create_task(self._ensure_message_rates()))

    async def _configure_message_rates(self) -> None:
        if self.is_connected:
            try:
                await self.mavtelem.set_rate_position(self.position_update_rate)
                await self.mavtelem.set_rate_position_velocity_ned(self.position_update_rate)
                await self.mavtelem.set_rate_attitude_euler(self.position_update_rate)
                await self.mavtelem.set_rate_altitude(self.position_update_rate)
                await self.mavtelem.set_rate_battery(self.position_update_rate)
                await self.mavtelem.set_rate_gps_info(self.position_update_rate)
            except Exception as e:
                self.logger.warning(f"Couldn't set message rate!")
                self.logger.debug(f"{repr(e)}", exc_info=True)

    async def _ensure_message_rates(self):
        # Send our desired message rates every so often to ensure they are adhered to
        while True:
            if self.is_connected:
                await self._configure_message_rates()
            await asyncio.sleep(5)

    async def _connect_check(self):
        # Check initial state, then listen to updates.
        self._is_connected = await self.system.is_connected()
        async for connected in self.system.is_connected_state():
            self._is_connected = connected

    async def _arm_check(self):
        async for arm in self.mavtelem.subscribe_armed():
            self._is_armed = arm

    async def _flightmode_check(self):
        async for flightmode in self.mavtelem.subscribe_flight_mode():
            self._flightmode = flightmode

    async def _inair_check(self):
        async for in_air in self.mavtelem.subscribe_in_air():
            self._in_air = in_air

    async def _gps_check(self):
        async for gps in self.mavtelem.subscribe_gps_info():
            self._gps_info = gps.fix_type

    async def _g_pos_check(self):
        async for pos in self.mavtelem.subscribe_position():
            self._position_g[0] = pos.latitude_deg
            self._position_g[1] = pos.longitude_deg
            self._position_g[2] = pos.absolute_altitude_m
            self._position_g[3] = pos.relative_altitude_m

    async def _vel_rpos_check(self):
        async for pos_vel in self.mavtelem.subscribe_position_velocity_ned():
            self._velocity[0] = pos_vel.velocity.north_m_s
            self._velocity[1] = pos_vel.velocity.east_m_s
            self._velocity[2] = pos_vel.velocity.down_m_s
            self._position_ned[0] = pos_vel.position.north_m
            self._position_ned[1] = pos_vel.position.east_m
            self._position_ned[2] = pos_vel.position.down_m

    async def _att_check(self):
        async for att in self.mavtelem.subscribe_attitude_euler():
            self._attitude[0] = att.roll_deg
            self._attitude[1] = att.pitch_deg
            self._attitude[2] = att.yaw_deg

    async def _heading_check(self):
        async for heading in self.mavtelem.subscribe_heading():
            self._heading = heading.heading_deg

    async def _battery_check(self):
        async for battery in self.mavtelem.subscribe_battery():
            battery_id = battery.id
            if battery_id in self._batteries:
                own_battery = self._batteries[battery_id]
            else:
                own_battery = Battery()
                own_battery.id = battery_id
                self._batteries[battery_id] = own_battery
            own_battery.consumed = battery.capacity_consumed_ah
            own_battery.remaining = battery.remaining_percent
            own_battery.voltage = battery.voltage_v
            own_battery.temperature = battery.temperature_degc

    async def _status_check(self):
        async for message in self.mavtelem.subscribe_status_text():
            if message.type is StatusTextType.DEBUG:
                self.logger.debug(f"{message.text}")
            elif message.type in [StatusTextType.INFO, StatusTextType.NOTICE]:
                self.logger.info(f"{message.text}")
            elif message.type is StatusTextType.WARNING:
                self.logger.warning(f"{message.text}")
            else:
                self.logger.error(f"{message.text}")

    async def arm(self):
        timeout = 5
        self.logger.info("Arming!")
        await super().arm()
        result = await self._error_wrapper(self.mavaction.arm, ActionError)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while not self.is_armed:
                await asyncio.sleep(1 / self.position_update_rate)
                if time.time() - start_time > timeout:
                    self.logger.warning("Arming timed out!")
                    return False
            self.logger.info("Armed!")
        else:
            self.logger.warning("Couldn't arm!")
        return result

    async def disarm(self):
        timeout = 5
        self.logger.info("Disarming!")
        if self.path_follower is not None and self.path_follower.is_active:
            await self.path_follower.deactivate()
        await super().disarm()
        result = await self._error_wrapper(self.mavaction.disarm, ActionError)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while self.is_armed:
                await asyncio.sleep(1 / self.position_update_rate)
                if time.time() - start_time > timeout:
                    self.logger.warning("Disarming timed out!")
                    return False
            self.logger.info("Disarmed!")
        else:
            self.logger.warning("Couldn't disarm!")
        return result

    def _can_takeoff(self):
        if not self.is_armed:
            raise RuntimeError("Can't take off without being armed!")

    async def takeoff(self, altitude=2.0, allow_in_air=True) -> bool:
        """

        :param altitude:
        :return:
        """
        if not allow_in_air and self.in_air:
            self.logger.debug("Can't takeoff again, already in the air!")
            return False
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        res =  await self._takeoff_using_offboard(altitude=altitude)
        if isinstance(res, Exception) or not res:
            return False
        self.return_position = Waypoint(WayPointType.POS_NED, pos=self.position_ned, yaw=self.attitude[2])
        return res

    def _get_pos_ned_yaw(self) -> np.ndarray[np.float64]:
        pos_yaw = np.zeros((4,), dtype=np.float64)
        pos_yaw[:3] = self.position_ned
        pos_yaw[3] = self.attitude[2]
        return pos_yaw

    async def _takeoff_using_takeoffmode(self, altitude=2.0):
        """

        :param altitude: Currently ignored.
        :return:
        """
        self.logger.info("Trying to take off...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        result = await self._error_wrapper(self.mavaction.takeoff, ActionError)
        if isinstance(result, Exception):
            self.logger.warning("Takeoff denied!")
        while self.flightmode is not FlightMode.TAKEOFF:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info(f"Taking off to {altitude}m over launch!")
        while self.flightmode is FlightMode.TAKEOFF:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Completed takeoff!")
        return True

    async def _takeoff_using_offboard(self, altitude=2.0, tolerance=0.25):
        """

        :param altitude:
        :param tolerance:
        :return:
        """
        self.logger.info(f"Trying to take off to {altitude}m in offboard mode...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        target_pos_yaw = self._get_pos_ned_yaw()
        target_pos_yaw[2] = target_pos_yaw[2] - altitude
        await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=target_pos_yaw[:3], yaw=target_pos_yaw[3]))
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        self.logger.info(f"Taking off to {target_pos_yaw[2]} in local coordinates!")
        while True:
            if self.is_at_pos(target_pos_yaw, tolerance=tolerance):
                self.logger.info(f"Takeoff completed!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def change_flight_mode(self, flightmode: str, timeout: float = 5):
        self.logger.info(f"Changing flight mode to {flightmode}")
        await super().change_flight_mode(flightmode)
        start_time = time.time()
        if flightmode == "hold":
            result = await self._error_wrapper(self.mavaction.hold, ActionError)
            target_flight_mode = FlightMode.HOLD
        elif flightmode == "offboard":
            result = await self._error_wrapper(self.mavoffboard.start, OffboardError)
            target_flight_mode = FlightMode.OFFBOARD
        elif flightmode == "return":
            result = await self._error_wrapper(self.mavaction.return_to_launch, ActionError)
            target_flight_mode = FlightMode.RETURN_TO_LAUNCH
        elif flightmode == "land":
            result = await self._error_wrapper(self.mavaction.land, ActionError)
            target_flight_mode = FlightMode.LAND
        elif flightmode == "takeoff":
            self._can_takeoff()
            result = await self._error_wrapper(self.mavaction.takeoff, ActionError)
            target_flight_mode = FlightMode.TAKEOFF
        elif flightmode == "position":
            result = await self._error_wrapper(self.mavmanual.start_position_control, ManualControlError)
            target_flight_mode = FlightMode.POSCTL
        elif flightmode == "altitude":
            result = await self._error_wrapper(self.mavmanual.start_altitude_control, ManualControlError)
            target_flight_mode = FlightMode.ALTCTL
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")
        if isinstance(result, Exception):
            self.logger.warning(f"Couldn't change flight mode due to exception {repr(result)}")
            return False
        elif not result:
            self.logger.warning("Flightmode change denied!")
            return False
        else:
            while self.flightmode != target_flight_mode and time.time() < start_time + timeout:
                await asyncio.sleep(1/self.position_update_rate)
            if self.flightmode != target_flight_mode:
                self.logger.warning("Drone accepted command, but flight mode change timed out! "
                                    "Possible connection issue.")
                return False
            else:
                self.logger.info(f"New flight mode {self.flightmode.name}!")
                return True

    async def set_setpoint(self, setpoint: "Waypoint"):
        setpoint_type = setpoint.type
        if setpoint_type == WayPointType.POS_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            return await self._error_wrapper(self.mavoffboard.set_position_ned, OffboardError, point_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self.mavoffboard.set_position_velocity_ned, OffboardError,
                                             point_ned_yaw, velocity_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_ACC_NED:
            yaw = setpoint.yaw
            point_ned_yaw = PositionNedYaw(*setpoint.pos, yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, yaw)
            acc_ned = AccelerationNed(*setpoint.acc)
            return await self._error_wrapper(self.mavoffboard.set_position_velocity_acceleration_ned,
                                             OffboardError,
                                             point_ned_yaw,
                                             velocity_ned_yaw,
                                             acc_ned)
        elif setpoint_type == WayPointType.VEL_NED:
            vel_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self.mavoffboard.set_velocity_ned, OffboardError, vel_yaw)
        elif setpoint_type == WayPointType.VEL_BODY:
            vel_yawrate = VelocityBodyYawspeed(*setpoint.vel, setpoint.yaw_rate)
            return await self._error_wrapper(self.mavoffboard.set_velocity_body, OffboardError, vel_yawrate)
        elif setpoint_type == WayPointType.POS_GLOBAL:
            latitude, longitude, amsl = setpoint.gps
            alt_type = PositionGlobalYaw.AltitudeType.AMSL
            position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=amsl,
                                         yaw_deg=setpoint.yaw, altitude_type=alt_type)
            return await self._error_wrapper(self.mavoffboard.set_position_global, OffboardError, position)
        else:
            raise RuntimeError("Invalid SetPointType!")

    def _can_do_in_air_commands(self):
        # TODO: Figure out how to do this with ardupilot. Currently the in_air detection seems very poor
        # Currently not used as the in-air detection is unreliable with many of our drones.
        if self.autopilot == "ardupilot":
            return True
        if not self.is_armed or not self.in_air:
            return False
        return True

    async def yaw_to(self, target_yaw, yaw_rate=30, local=None, tolerance=2):
        """Yawing to the target heading as you do so at the specified rate, maintaining current position.

        Uses the local coordinate system for to determine and maintain position. Pausable.

        :param target_yaw: Heading as a degree fom -180 to 180, right positive, 0 forward.
        :param yaw_rate:
        :param local: Position setpoint during yaw.
        :param tolerance: How close we have to get to the heading before this function returns.
        :return:
        """
        # Add 180, take modulo 360 and subtract 180 to get proper range
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        og_yaw = self.attitude[2]
        dif_yaw = (target_yaw - og_yaw + 180) % 360 - 180
        time_required = abs(dif_yaw / yaw_rate)
        n_steps = math.ceil(time_required * self.position_update_rate)
        step_size = dif_yaw/n_steps
        if local is None:
            local = np.asarray(self.position_ned, dtype=float)
        for i in range(n_steps):
            if not self.is_paused:
                yaw = og_yaw + step_size*(i+1)
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=local, yaw=yaw))
            await asyncio.sleep(1 / self.position_update_rate)
        while not self.is_at_heading(target_heading=target_yaw, tolerance=tolerance):
            await asyncio.sleep(1 / self.position_update_rate)
        return True

    async def spin_at_rate(self, yaw_rate, duration, direction="cw"):
        """ Spin in place at the given rate for the given duration.

        Pausable.

        :param yaw_rate:
        :param duration:
        :param direction:
        :return:
        """
        await super().spin_at_rate(yaw_rate, duration, direction=direction)
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        pos = self.position_ned
        og_yaw = self.attitude[2]
        freq = 10
        n_steps = math.ceil(duration * freq)
        step_size = yaw_rate / freq
        if direction == "ccw":
            step_size = - step_size
        for i in range(n_steps):
            if not self.is_paused:
                yaw = og_yaw + step_size*(i+1)
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=pos, yaw=yaw))
            await asyncio.sleep(1/freq)

    async def fly_to(self, local: np.ndarray | None = None, gps: np.ndarray | None = None, yaw: float | None = None,
                     waypoint: Waypoint | None = None, tolerance=0.25, put_into_offboard=True, log=True):
        """ Fly to a specified point in offboard mode. Uses path generators and followers to get there.

        If multiple target are provided (for example GPS and local coordinates), we prefer coordinates in this fashion:
        Waypoint > GPS > local, i.e. in the example, the local coordinates would be ignored.

        :param local:
        :param gps:
        :param yaw:
        :param waypoint:
        :param tolerance:
        :param put_into_offboard:
        :param log:
        :return:
        """
        # Check that we have one full set of coordinates and are in a flyable state
        #if not self._can_do_in_air_commands():
        #    raise RuntimeError("Can't fly a landed or unarmed drone!")
        assert local is not None or gps is not None or waypoint is not None, \
            "Must provide a full set of either NED coordinates, GPS coordinates or a waypoint!"

        # Check that we have a path generator and follower who are compatible with each other and the drone
        assert (self.path_follower is not None
                and self.path_follower.setpoint_type in self.VALID_SETPOINT_TYPES)
        assert (self.path_generator is not None
                and self.path_generator.waypoint_type in self.path_follower.WAYPOINT_TYPES)

        # Determine target waypoint, prefering waypoint over GPS over local and using current yaw if none is provided
        if waypoint is not None:
            target = waypoint
            if waypoint.yaw is None:
                # Maintain current yaw if none given
                waypoint.yaw = self.attitude[2]
        elif gps is not None:
            if yaw is None:
                yaw = self.attitude[2]
            target = Waypoint(WayPointType.POS_GLOBAL, gps=gps, yaw=yaw)
        else:
            if yaw is None:
                yaw = self.attitude[2]
            target = Waypoint(WayPointType.POS_NED, pos=local, yaw=yaw)

        # Check that the target is valid
        assert self.check_waypoint(target), f"Invalid target position {target}, probably due to fence."

        use_gps = target.type == WayPointType.POS_GLOBAL

        if log:
            if use_gps:
                self.logger.info(f"Flying to Lat: {target.gps[0]} Long: {target.gps[1]} AMSL: {target.gps[2]} "
                                 f"facing {yaw} with tolerance {tolerance}")
            else:
                self.logger.info(f"Flying to N: {target.pos[0]} E: {target.pos[1]} D: {target.pos[2]} facing {yaw} "
                                 f"with tolerance {tolerance}")

        if put_into_offboard and self._flightmode != FlightMode.OFFBOARD:
            if use_gps:
                cur_lat, cur_long, cur_amsl = self.position_global
                cur_yaw = self.attitude[2]
                await self.set_setpoint(Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray([cur_lat, cur_long, cur_amsl]),
                                                 yaw=cur_yaw))
            else:
                cur_pos = self.position_ned
                cur_yaw = self.attitude[2]
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=cur_pos, yaw=cur_yaw))
            await self.change_flight_mode("offboard")

        # Check that both path generator and follower can handle GPS coordinates
        if use_gps:
            if not self.path_generator.CAN_DO_GPS:
                raise RuntimeError("Path generator can't use GPS coordinates!")
            if not self.path_follower.CAN_DO_GPS:
                raise RuntimeError("Path follower can't use GPS coordinates!")
        self.path_generator.set_target(target)

        # Create path and activate follower algorithm if not already active
        self.logger.debug("Creating path...")
        path_task = asyncio.create_task(self.path_generator.create_path())
        self.current_action_tasks.add(path_task)
        have_path = await path_task
        if not have_path:
            self.logger.warning("The path generator couldn't generate a path!")
            return False
        if not self.path_follower.is_active:
            self.logger.debug("Starting follower algorithm...")
            self.path_follower.activate()

        while True:
            # Check if we have arrived at target waypoint
            if use_gps:
                reached = (self.is_at_gps(target.gps, tolerance=tolerance)
                           and self.is_at_heading(target.yaw, tolerance=1))
            else:
                reached = (self.is_at_pos(target.pos, tolerance=tolerance) and
                           self.is_at_heading(target.yaw, tolerance=1))

            # Print message and stop if we have reached waypoint
            if reached:
                self.logger.info("Reached target position!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def move(self, offset, yaw: float | None = None, use_gps=True, tolerance=0.25):
        self.logger.info("Starting move")
        if yaw is None:
            target_yaw = None
        else:
            target_yaw = self.attitude[2] + yaw
        if use_gps:
            target_lat, target_long, target_amsl = relative_gps(self.position_global, offset)
            waypoint = Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray([target_lat, target_long, target_amsl]), yaw=target_yaw)
        else:
            waypoint = Waypoint(WayPointType.POS_NED, pos=self.position_ned + offset, yaw=target_yaw)
        return await self.fly_to(waypoint=waypoint, put_into_offboard=True, tolerance=tolerance)

    async def go_to(self, local: np.ndarray | None = None, gps: np.ndarray | None = None, yaw: float | None = None,
                     waypoint: Waypoint | None = None, tolerance=0.25,):
        assert local is not None or gps is not None or waypoint is not None, \
            "Must provide a full set of either NED coordinates, GPS coordinates or a waypoint!"

        # If flightmode is not Hold, goto hold
        if self.flightmode != FlightMode.HOLD:
            self.logger.info("Changing flightmode to hold")
            await self.change_flight_mode("hold")

        if waypoint is not None:
            if waypoint.type is WayPointType.POS_GLOBAL:
                gps = waypoint.gps
            else:
                offset = waypoint.pos - self.position_ned
                gps = relative_gps(self.position_global, offset)
            yaw = waypoint.yaw
        elif gps is not None:
            pass
        elif local is not None:
            offset = local - self.position_ned
            gps = relative_gps(self.position_global, offset)

        # Send goto command
        await self.mavaction.goto_location(*gps, yaw)

        while True:
            # Check if we have arrived at target waypoint
            reached = (self.is_at_gps(gps, tolerance=tolerance) and self.is_at_heading(yaw, tolerance=1))

            # Print message and stop if we have reached waypoint
            if reached:
                self.logger.info("Reached target position!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def orbit(self, radius, velocity, center_lat, center_long, amsl):
        await super().orbit(radius, velocity, center_lat, center_long, amsl)
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        yaw_behaviour = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await self._error_wrapper(self.mavaction.do_orbit, ActionError, radius, velocity, yaw_behaviour,
                                  center_lat, center_long, amsl)

    async def land(self):
        self.logger.info("Trying to land...")
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        await super().land()
        return await self._land_using_offbord_mode()

    async def _land_using_offbord_mode(self, error_thresh=0.00001, min_time=1):
        self.logger.info("Landing!")
        ema_alt_error = 0
        going_down = True
        old_alt = self.position_ned[2]
        start_time = time.time()
        target_pos = self._get_pos_ned_yaw()
        if self._flightmode != FlightMode.OFFBOARD:
            await self.set_setpoint(Waypoint(WayPointType.POS_VEL_NED, pos=target_pos[:3], yaw=target_pos[3]))
            await self.change_flight_mode("offboard")
        update_freq = 2
        try:
            await self.mavtelem.set_rate_position_velocity_ned(self.position_update_rate)
            update_freq = self.position_update_rate
        except Exception as e:
            self.logger.debug(f"Couldn't set message rate: {repr(e)}", exc_info=True)
        while going_down:
            cur_alt = self.position_ned[2]
            ema_alt_error = (cur_alt - old_alt) + 0.33 * ema_alt_error
            if ema_alt_error < error_thresh and time.time() > start_time + min_time:
                going_down = False
            old_alt = cur_alt
            target_pos[2] = cur_alt + 0.5
            await self.set_setpoint(Waypoint(WayPointType.POS_VEL_NED, pos=target_pos[:3], vel=[0, 0, 0.3], yaw=target_pos[3]))
            await asyncio.sleep(1/update_freq)
        self.logger.info("Landed!")
        return True

    async def _land_using_landmode(self):
        result = await self._error_wrapper(self.mavaction.land, ActionError)
        if isinstance(result, Exception):
            self.logger.warning("Couldn't go into land mode")
        while self.flightmode is not FlightMode.LAND:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Landing!")
        while self.in_air:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Landed!")
        await self.change_flight_mode("hold")
        return True

    async def manual_control_position(self):
        self.clear_queue()
        self.cancel_action()
        await self.path_follower.deactivate()
        result = await self._error_wrapper(self.mavmanual.start_position_control, ManualControlError)
        return result

    async def manual_control_altitude(self):
        self.clear_queue()
        self.cancel_action()
        await self.path_follower.deactivate()
        result = await self._error_wrapper(self.mavmanual.start_altitude_control, ManualControlError)
        return result

    async def set_manual_control_input(self, x, y, z, r):
        result = await self._error_wrapper(self.mavmanual.set_manual_control_input, ManualControlError, x, y, z, r)
        return result

    async def stop(self):
        # Override whatever else is going on and land
        self.clear_queue()
        self.cancel_action()
        if not self.is_connected:
            return True
        await self.land()
        await self.disarm()
        await super().stop()
        return True

    async def kill(self):
        await self._error_wrapper(self.mavaction.kill, ActionError)
        await super().kill()
        return True

    async def _error_wrapper(self, func, error_type: Type, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except error_type as e:
            self.logger.error(f"MAV{error_type.__name__}: {e.result.name}")
            return False
        return True


# ========== GIMBAL IMPLEMENTATION ====================================================================================


class MAVSDKGimbal(Gimbal[MAVSDKDrone]):
    def __init__(self, drone: MAVSDKDrone, gimbal_identifier: str):
        super().__init__(drone, gimbal_identifier)

        gimbal_id, device_id, gimbal_manager_component_id = gimbal_identifier.split(".")
        self.gimbal_id = int(gimbal_id)
        self.device_id = int(device_id)
        self.gimbal_manager_component_id = int(gimbal_manager_component_id)
        # Attributes to identify gimbals and gimbal-related components for MAVLink specifically.

        self.primary_control: tuple[int, int] = (-1, -1)
        self.secondary_control: tuple[int, int] = (-1, -1)

    @classmethod
    async def get_gimbals(cls, drone: MAVSDKDrone):
        gimbal_list = await drone.mavgimbal.gimbal_list()
        gimbal_identifiers = []
        for gimbalitem in gimbal_list.gimbals:
            gimbal_identifiers.append(f"{gimbalitem.gimbal_id}.{gimbalitem.gimbal_device_id}.{gimbalitem.gimbal_manager_component_id}")
        return gimbal_identifiers

    @classmethod
    def prettify_identifier(cls, identifier) -> str:
        gimbal_id, device_id, component_manager_component_id = identifier.split(".")
        return f"Gimbal with identifier: {identifier}: Gimbal ID {gimbal_id}, device ID {device_id}, component manager ID {component_manager_component_id}."

    async def start(self):
        self._running_tasks.add(asyncio.create_task(self._gimbal_att_checker()))
        self._running_tasks.add(asyncio.create_task(self._gimbal_control_checker()))
        # Wait until we have attitude info before finishing
        while math.isnan(self.pitch) or self.primary_control[0] < 0:
            await asyncio.sleep(0.1)

    async def close(self):
        await super().close()

    @property
    def in_control(self):
        return self.primary_control[0] == self.drone.comm.system_id and self.primary_control[1] == self.drone.comm.component_id

    async def _gimbal_att_checker(self):
        async for attitude in self.drone.mavgimbal.subscribe_attitude():
            if attitude.gimbal_id == self.gimbal_id:
                self.roll = attitude.euler_angle_forward.roll_deg
                self.pitch = attitude.euler_angle_forward.pitch_deg
                self.yaw = attitude.euler_angle_forward.yaw_deg
                self.roll_absolute = attitude.euler_angle_north.roll_deg
                self.pitch_absolute = attitude.euler_angle_north.pitch_deg
                self.yaw_absolute = attitude.euler_angle_north.yaw_deg

    async def _gimbal_control_checker(self):
        init_ctrl = await self.drone.mavgimbal.get_control_status(self.gimbal_id)
        if init_ctrl.gimbal_id == self.gimbal_id:
            self.primary_control = (init_ctrl.sysid_primary_control, init_ctrl.compid_primary_control)
            self.secondary_control = (init_ctrl.sysid_secondary_control, init_ctrl.compid_secondary_control)
        async for ctrl in self.drone.mavgimbal.subscribe_control_status():
            if ctrl.gimbal_id == self.gimbal_id:
                self.primary_control = (ctrl.sysid_primary_control, ctrl.compid_primary_control)
                self.secondary_control = (ctrl.sysid_secondary_control, ctrl.compid_secondary_control)

    def log_status(self, logger):
        intro_str = f"{self.prettify_identifier(self.gimbal_identifier)}"
        ctrl_string = f"\tIn control: {'Yes' if self.in_control else 'No'}, primary: {self.primary_control}, secondary: {self.secondary_control}"
        att_str = f"\tYaw: {self.yaw}, Pitch: {self.pitch}, Roll: {self.roll}"
        abs_att_str = f"\tAbsYaw: {self.yaw_absolute}, AbsPitch: {self.pitch_absolute}, AbsRoll: {self.roll_absolute}"
        logger.info(f"Gimbal {self.gimbal_identifier} status:\n{intro_str}\n{ctrl_string}\n{att_str}\n{abs_att_str}")

    async def take_control(self, timeout: float = 5) -> bool:
        if self.in_control:
            self.logger.info(f"Already in control of {self.gimbal_identifier}!")
            return True
        self.logger.info(f"Taking control of gimbal {self.gimbal_identifier}")
        start_time = time.time()
        res = await self._error_wrapper(self.drone.mavgimbal.take_control, self.gimbal_id, ControlMode.PRIMARY)
        if not res:
            self.logger.info(f"Couldn't take control of gimbal {self.gimbal_identifier}!")
            return False
        # Command accepted, wait for us to be in control
        while time.time() - start_time < timeout:
            await asyncio.sleep(0.1)
            if self.in_control:
                self.logger.info(f"Took control of gimbal {self.gimbal_identifier}")
                return True
        self.logger.info("Control change timed out!")
        return False

    async def release_control(self, timeout: float = 5):
        if not self.in_control:
            self.logger.info(f"Already not controlling {self.gimbal_identifier}!")
            return True
        self.logger.info(f"Releasing control of gimbal {self.gimbal_identifier}")
        start_time = time.time()
        res = await self._error_wrapper(self.drone.mavgimbal.release_control, self.gimbal_id)
        if not res:
            self.logger.info(f"Couldn't release control over gimbal {self.gimbal_identifier}!")
            return False
        # Command accepted, wait for us to be in control
        while time.time() - start_time < timeout:
            await asyncio.sleep(0.1)
            if not self.in_control:
                self.logger.info(f"Released control over gimbal {self.gimbal_identifier}")
                return True
        self.logger.info("Control change timed out!")
        return False

    async def point_gimbal_at(self, lat, long, amsl):
        self.logger.info(f"Pointing gimbal {self.gimbal_identifier} pointing at Lat {lat}, Long {long}, AMSL {amsl}")
        res = await self._error_wrapper(self.drone.mavgimbal.set_roi_location, self.gimbal_id, lat, long, amsl)
        if res:
            self.logger.info("Gimbal accepted ROI command!")
        else:
            self.logger.info("Gimbal didn't accept ROI command!")
        return res

    async def set_gimbal_angles(self, yaw: float, pitch: float, roll: float | None = None, tolerance: float = 1):
        self.logger.info(f"Setting gimbal angles for {self.gimbal_identifier} to YPR: {yaw, pitch, roll} with tolerance {tolerance}")
        if roll is not None:
            self.logger.warning("This gimbal can't accept roll angles, self levels automatically!")
        res = await self._error_wrapper(self.drone.mavgimbal.set_angles, self.gimbal_id, 0, pitch, yaw, self.mode,
                                        SendMode.ONCE)
        if not res:
            self.logger.info(f"Gimbal {self.gimbal_identifier} refused angle set command!")
            return res
        # Command accepted, wait for gimbal to have reached angle.
        while abs(self.yaw - yaw) > tolerance and abs(self.pitch - pitch) > tolerance:
            await asyncio.sleep(0.1)
        self.logger.info(f"Gimbal {self.gimbal_identifier} reached target angles!")
        return res

    async def set_gimbal_rates(self, yaw_rate: float, pitch_rate: float, roll_rate: float | None = None):
        self.logger.info(f"Setting gimbal {self.gimbal_identifier} angular rates to YPR: {yaw_rate, pitch_rate, roll_rate}")
        if roll_rate is not None:
            self.logger.warning("This gimbal can't accept roll angles, self levels automatically!")
        res = await self._error_wrapper(self.drone.mavgimbal.set_angular_rates, self.gimbal_id, 0, pitch_rate, yaw_rate, self.mode, SendMode.ONCE)
        if not res:
            self.logger.info(f"Gimbal {self.gimbal_identifier} refused angular rate command!")
        else:
            self.logger.info(f"Gimbal {self.gimbal_identifier} accepted angular rate command!")
        return res

    async def set_gimbal_mode(self, mode: YawMode):
        self.mode = mode
        self.logger.info(f"Set gimbal {self.gimbal_identifier} yaw mode to {mode.name}")
        return True

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(f"GimbalError: {e.result.name}")
            return False
        return True


# ========== CAMERA IMPLEMENTATION ====================================================================================
# TODO: Everything below this

class MAVSDKCamera(Camera[MAVSDKDrone]):

    def __init__(self, drone: MAVSDKDrone, camera_identifier: str):
        super().__init__(drone, camera_identifier)

        self.cam_component_id = int(camera_identifier)  # 100?
        self.parameters: dict[str, MAVSDKCameraParameter] = {}

        self.vendor_name = None
        self.model_name = None
        self.cam_def_uri: str | None = None
        self.cam_def_version = None
        self._received_params = set()
        self._param_count = None
        self._message_callbacks: dict[str, Callable] = {}

    @classmethod
    async def get_cameras(cls, drone: MAVSDKDrone) -> tuple[list[str], list[str]]:
        cam_list = await drone.mavcamera.camera_list()
        cam_identifiers = []
        extra_info = []
        for cam_info in cam_list.cameras:
            cam_identifiers.append(f"{cam_info.component_id}")
            extra_info.append(f"{cam_info.vendor_name}.{cam_info.model_name}")
        return cam_identifiers, extra_info

    @classmethod
    def prettify_identifier(cls, identifier, extra_info) -> str:
        vendor_name, model_name = extra_info.split(".")
        return f"Camera with identifier {identifier}: Component ID {identifier}, Vendor {vendor_name}, Model {model_name}"

    # TODO: Everything below this

    async def start(self):
        capture_info_task = asyncio.create_task(self._capture_info_updates())
        self._running_tasks.add(capture_info_task)
        self._running_tasks.add(asyncio.create_task(coroutine_awaiter(capture_info_task, self.logger)))
        await self.load_camera_parameters()

    async def close(self):
        for msg_name, callback in self._message_callbacks:
            self.drone.remove_message_callback(msg_name, callback)
        await super().close()

    async def _capture_info_updates(self):
        async for capture_info in self.drone.mavcamera.subscribe_capture_info():
            self.logger.info(f"Capture update: Camera {capture_info.component_id} "
                             f"{'succeeded' if capture_info.is_success else 'failed'} with photo at "
                             f"{capture_info.time_utc_us}: {capture_info.file_url}")

    async def load_camera_parameters(self):
        have_cam = await self.init_cam_info()
        if have_cam:
            got_params = await self.get_cam_param_definition()
            if not got_params:
                self.logger.warning("Connected to a camera, but couldn't load the camera definition")
                return False
            else:
                self.logger.info("Loaded camera parameter definition.")
            self._get_cam_params()
            return True
        else:
            self.logger.warning(f"Couldn't get basic information for camera {self.camera_identifier}")
        return False

    async def init_cam_info(self):
        cam_info, request_ok = self.drone.request_message(259, "CAMERA_INFORMATION", self.cam_component_id)
        request_ok = await request_ok
        cam_info = await cam_info
        if request_ok != 0:
            self.logger.warning(f"Camera information request failed with MAV code {request_ok}")
        if not cam_info:
            self.logger.warning("No camera found!")
        else:
            self.logger.debug(f"Raw cam info: {cam_info.fields_json}")
            fields = json.loads(cam_info.fields_json, strict=False)
            self.vendor_name = bytearray(fields["vendor_name"]).rstrip(b"\x00").decode("ascii")
            self.model_name = bytearray(fields["model_name"]).rstrip(b"\x00").decode("ascii")
            self.cam_def_uri = fields["cam_definition_uri"]
            self.cam_def_version = fields["cam_definition_version"]
            self.logger.debug(f"Received camera information: {self.vendor_name}, {self.model_name}, {self.cam_def_uri}, {self.cam_def_version}")
            return True
        return False

    @property
    def params_loaded(self):
        return len(self.parameters) != 0

    def log_status(self, logger):
        logger.info(f"Camera {self.camera_identifier}, parameters {'not ' if not self.params_loaded else ''}loaded")

    async def take_picture(self,):
        res = await self.drone.send_command(2000, self.cam_component_id, 0, param3=1.0, param5=None)
        if not res:
            self.logger.warning("Couldn't take picture!")
        return res

    async def start_video(self):
        res = await self.drone.send_command(2500, self.cam_component_id, 0, param2=1)
        if not res:
            self.logger.warning("Couldn't start video!")
        return res

    async def stop_video(self):
        res = await self.drone.send_command(2501, self.cam_component_id, 0)
        if not res:
            self.logger.warning("Couldn't stop video!")
        return res

    async def set_zoom(self, zoom):
        res = await self.drone.send_command(203, self.cam_component_id, 0, param2=zoom, param5=0)
        if not res:
            self.logger.warning("Couldn't set the zoom!")
        return res

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            res = await func(*args, **kwargs)
        except CameraError as e:
            self.logger.error(f"CameraError: {e.result.name}")
            return False
        return res

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
            self.logger.info(f"Loading cam {self.camera_identifier} definition")
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
        with open(filepath, "wb") as f:
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
                if param_type == "float":
                    cast_type = float
                elif param_type == "bool":
                    cast_type = bool
                else:
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

                self.parameters[name] = MAVSDKCameraParameter(name=name, param_type=param_type, default=default,
                                                        control=control, description=description, updates=updates,
                                                        options=options, min_value=min_val, max_value=max_val,
                                                        step_size=step_size)
        except Exception as e:
            self.logger.warning("Couldn't parse the parameter XML")
            self.logger.debug(repr(e), exc_info=True)

    def _get_cam_params(self):
        self.logger.debug("Listening to camera parameters")
        fields = {
            "target_system": self.drone.system_id,
            "target_component": self.cam_component_id,
        }
        request_ext_params_msg = MavlinkMessage("PARAM_EXT_REQUEST_LIST", self.drone.comm.system_id, self.drone.comm.component_id,
                                                self.drone.system_id, self.cam_component_id, json.dumps(fields))
        request_params_task = asyncio.create_task(self.drone.send_message(request_ext_params_msg))
        self._running_tasks.add(request_params_task)
        self._running_tasks.add(coroutine_awaiter(request_params_task, self.logger))
        self.drone.add_message_callback("PARAM_EXT_VALUE", self._listen_param_updates)
        checker_task = asyncio.create_task(self._param_receive_checker())
        self._running_tasks.add(checker_task)
        self._running_tasks.add(coroutine_awaiter(checker_task, self.logger))

    def _make_param_ext_request_read_msg(self, param_name = "", param_index = -1):
        fields = {
            "target_system": self.drone.system_id,
            "target_component": self.cam_component_id,
            "param_id": param_name,
            "param_index": param_index,
        }
        request_param_ext_read_msg = MavlinkMessage("PARAM_EXT_REQUEST_READ", self.drone.comm.system_id,
                                                    self.drone.comm.component_id, self.drone.system_id,
                                                    self.cam_component_id, json.dumps(fields))
        return request_param_ext_read_msg

    async def _param_receive_checker(self):
        while len(self._received_params) < self._param_count:
            for param_index in range(self._param_count):
                await asyncio.sleep(1)
                if param_index not in self._received_params:
                    self.logger.debug(f"Missing parameter {param_index}, requesting again")
                    request_param_ext_read_msg = self._make_param_ext_request_read_msg(param_index=param_index)
                    await self.drone.send_message(request_param_ext_read_msg)

    def _parse_param_update_values(self, fields):
        param_name = fields["param_id"]
        param_type_id = fields["param_type"]
        raw_param_value = fields["param_value"]  # Expect a list of ints representing bytes
        #param_name = msg.param_id
        #param_type = msg.param_type
        #raw_param_value = msg._param_value_raw
        if param_type_id < 9:
            length = 2 ** int(param_type_id / 2)
        elif param_type_id == 9:
            length = 4
        elif param_type_id == 10:
            length = 8
        else:
            raise NotImplementedError("Custom parameter types not supported!")
        if param_type_id in [1, 3, 5, 7]:
            parsed_param_value = int.from_bytes(raw_param_value[:length], byteorder="little", signed=False)
        elif param_type_id in [2, 4, 6, 8]:
            parsed_param_value = int.from_bytes(raw_param_value[:length], byteorder="little", signed=True)
        elif param_type_id == 9:
            parsed_param_value = struct.unpack("<f", bytes(raw_param_value[:length]))[0]
        elif param_type_id == 10:
            parsed_param_value = struct.unpack("<d", bytes(raw_param_value[:length]))[0]
        else:
            self.logger.warning("Camera sending custom parameter type, not supported!")
            parsed_param_value = None
        self.logger.debug(f"Received parameter update message: "
                          f"{param_name, parsed_param_value, raw_param_value[:length], param_type_id}.")
        return param_name, parsed_param_value

    def _update_param_value(self, param_name, param_value, param_type_id):
        if param_name in self.parameters:
            if param_value is not None:
                self.logger.debug(f"Updating param {param_name} to {param_value}.")
                self.parameters[param_name].value = param_value
            self.parameters[param_name].param_type_id = param_type_id

    def _listen_param_updates(self, message):
        if message.component_id == self.cam_component_id and message.system_id == self.drone.system_id:
            fields = json.loads(message.fields_json, strict=False)
            param_name, param_value = self._parse_param_update_values(fields)
            self._update_param_value(param_name, param_value, fields["param_type"])
            self._param_count = fields["param_count"]
            self._received_params.add(fields["param_index"])

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

    def parse_parameter_value(self, param_name: str, param_value: str) -> int | float | None:
        parameter = self.parameters[param_name]

        if param_name not in self.parameters:
            self.logger.warning(f"Don't have a parameter {param_name}!")
            return None

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
                self.logger.warning("Boolean parameter values must be True or False!")
        else:  # Try to parse it into either a float or an int depending on param_type
            try:
                if parameter.param_type_id is not None and parameter.param_type_id < 9:
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

    async def set_parameter(self, param_name: str, param_value: bool | int | float | str):
        # Send the change param message and request updates for any params in the updates list

        if param_name not in self.parameters:
            self.logger.warning(f"Don't have a parameter {param_name}!")
            return False

        parameter = self.parameters[param_name]

        self.logger.debug(f"Trying to set option {param_name}, {parameter.param_type_id} to {param_value}")

        if isinstance(param_value, bool):
            if param_value:
                param_value = 1
            else:
                param_value = 0
        elif isinstance(param_value, str):
            param_value = self.parse_parameter_value(param_name, param_value)
        elif isinstance(param_value, float):
            if parameter.param_type_id == 9:
                param_value = list(struct.pack("<f", param_value))
            elif parameter.param_type_id == 10:
                param_value = list(struct.pack("<d", param_value))
            else:
                self.logger.warning(f"Parameter {param_name} doesn't accept float values!")
                return False

        if param_value is None:
            self.logger.warning(f"Couldn't parse value while trying to set parameter {param_name}")
            return False

        fields = {
            "target_system": self.drone.system_id,
            "target_component": self.cam_component_id,
            "param_id": param_name,
            "param_value": param_value,
            "param_type": parameter.param_type_id,
        }
        param_ext_set_msg = MavlinkMessage("PARAM_EXT_SET", self.drone.comm.system_id, self.drone.comm.component_id,
                                           self.drone.system_id, self.cam_component_id, json.dumps(fields))
        await self.drone.send_message(param_ext_set_msg)
        ack_msg = await self.drone.listen_next_message("PARAM_EXT_ACK", self.cam_component_id)
        ack_fields = json.loads(ack_msg.fields_json, strict=False)
        ack_res = ack_fields["param_result"]
        if ack_res == 1:
            self.logger.warning("Camera denied parameter change, unsupported value!")
            return False
        elif ack_res == 2:
            self.logger.warning("Camera denied parameter change, failed to set!")
            return False
        elif ack_res in [0,3]:
            if ack_res == 3:
                self.logger.info("Parameter change in progress...")
            param_name, param_value = self._parse_param_update_values(ack_fields)
            self._update_param_value(param_name, param_value, ack_fields["param_type"])
            parameter = self.parameters[param_name]
            self.logger.info(f"Set Parameter {param_name} to "
                             f"{param_value if not parameter.is_option else parameter.get_current_otion().name}")

        # Request updates on any params that might have changed.
        for updated_param_name in parameter.updates:
            request_param_ext_read_msg = self._make_param_ext_request_read_msg(param_name=updated_param_name)
            await self.drone.send_message(request_param_ext_read_msg)
        return True

class ParameterOption:

    def __init__(self, name, value, excludes):
        self.name: str = name  # For display
        self.value: int | float = value  # This is used internally
        self.excludes: list[str] = excludes  # These parameters are rendered irrelevant if this option is set


class MAVSDKCameraParameter(CameraParameter):

    def __init__(self, name, param_type, default, control: bool, description: str, updates: list[str],
                 options: list[ParameterOption], min_value: float | None, max_value: float | None,
                 step_size: float | None):
        super().__init__(name, default, param_type=param_type)
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


def _xml_cache_filepath(uri, version):
    pathsafe_uri = uri.replace("/", "_").replace("\\", "_").replace(":", "_").replace("?", "_").replace("\n", "")
    filename = f"{version}_{pathsafe_uri}"
    cache_dir = CACHE_DIR.joinpath("camera_definitions")
    return cache_dir.joinpath(filename)


# COMM SUPPORTS
MAVSDKComm.SUPPORTS["gimbal"] = MAVSDKGimbal
MAVSDKComm.SUPPORTS["camera"] = MAVSDKCamera
