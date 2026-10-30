import asyncio
import inspect
import json
import logging
import math
import numpy as np
import time
from typing import Any

import mavsdk
from mavsdk.asyncio import Mavsdk, Configuration, System
from mavsdk.asyncio.plugins.telemetry import StatusTextType, TelemetryAsync
from mavsdk.asyncio.plugins.action import ActionAsync, ActionError, OrbitYawBehavior
from mavsdk.asyncio.plugins.offboard import PositionNedYaw, PositionGlobalYaw, VelocityNedYaw, AccelerationNed, OffboardError, \
    VelocityBodyYawspeed, OffboardAsync
from mavsdk.asyncio.plugins.manual_control import ManualControlError, ManualControlAsync
from mavsdk.asyncio.plugins.mavlink_direct import MavlinkMessage, MavlinkDirectAsync
from mavsdk.asyncio.plugins.param import ParamAsync
from mavsdk.asyncio.plugins.camera import CameraAsync
from mavsdk.asyncio.plugins.gimbal import GimbalAsync

from dronemanager.drone import DroneComProtocol
from dronemanager.drone import Battery, Drone, DroneConfig, DroneParams, FlightMode, FixType
from dronemanager.navigation.core import WayPointType, Waypoint
from dronemanager.navigation.directtargetgenerator import DirectTargetGenerator
from dronemanager.navigation.gmp3generator import GMP3Generator
from dronemanager.navigation.ruckigfollower import RuckigOfflineFollower
from dronemanager.utils import relative_gps, coroutine_awaiter
from dronemanager.utils import parse_address


# TODO: Complete drone connection, object handling, etc...
# TODO: Add auto-connection to drone if there is only one system at a connection within a certain timeframe.


class MAVSDKComm(DroneComProtocol):

    NAME = "mav"

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

    async def add_connection(self, connection_string):
        scheme, loc, appendix = parse_address(string=connection_string)
        if scheme == "udp":
            if loc == "0.0.0.0" or loc == "":
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
            return None
        self.logger.info(f"Adding connection {mavsdk_conn_string}")
        conn_handle = None
        try:
            conn_handle = await self.mavsdk.add_any_connection(mavsdk_conn_string)
            self.conn_handles[mavsdk_conn_string] = conn_handle
            return None
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
            self.logger.warning(f"Couldn't add drone {drone_name} for ID {system_id} as a drone with that ID already"
                                f"exists: {drone.name}!")
            return None
        drone.initialize(drone_name, config=config)
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
        drone = MAVSDKDrone(self, system, system_id=sys_id)
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


class MAVSDKDrone(Drone):

    VALID_FLIGHTMODES = {"hold", "offboard", "return", "land", "takeoff", "position", "altitude"}
    # This attribute is for checking which flight modes can be changed into manually
    VALID_SETPOINT_TYPES = {WayPointType.POS_NED,
                            WayPointType.POS_VEL_NED,
                            WayPointType.POS_VEL_ACC_NED,
                            WayPointType.VEL_NED,
                            WayPointType.POS_GLOBAL}
    # What type of path setpoints this classes fly_<> commands can follow. This limits what Trajectory generators
    # can be used.

    def __init__(self, comm_protocol: MAVSDKComm, system: System, system_id: int):
        super().__init__(comm_protocol, system_id)
        self.system: System = system
        self.fc_component_id: int = -1  # Actual component ids are always positive, so this is a safe "invalid" number.

        # Temporary attributes for new mavsdk version
        self._mavdirect = MavlinkDirectAsync(self.system)
        self._mavparam = ParamAsync(self.system)
        self._mavtelem = TelemetryAsync(self.system)
        self._mavaction = ActionAsync(self.system)
        self._mavoffboard = OffboardAsync(self.system)
        self._mavmanual = ManualControlAsync(self.system)
        self._mavcamera = CameraAsync(self.system)
        self._mavgimbal = GimbalAsync(self.system)

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

    def initialize(self, name, config: DroneConfig):
        super().initialize(name, config)
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
            return await self._mavdirect.send_message(msg)
        except Exception as e:
            self.logger.warning("Exception sending mavlink message!")
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
        msg_tuple = (command_msg_id, self.drone_identifier, target_component_id, self.comm.system_id, self.comm.component_id)
        if msg_tuple in self._ack_listeners:
            self._ack_listeners[msg_tuple].append(fut)
        else:
            self._ack_listeners[msg_tuple] = [fut]
        return fut

    def send_command(self, command_msg_id, target_component_id, confirmation, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        fields = {
            "target_system": self.drone_identifier,
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
        msg = MavlinkMessage("COMMAND_LONG", self.comm.system_id, self.comm.component_id, self.drone_identifier, target_component_id,
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
            async for message in self._mavdirect.subscribe_message(""):
                if self.config.log_telemetry:
                    self.telem_logger.debug(f"Received {message.message_name} from {message.system_id, message.component_id} to {message.target_system_id, message.target_component_id}: {message.fields_json}")
                # Check that the message is for us
                if (self.drone_identifier is None or message.system_id == self.drone_identifier) \
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
            parameters = await self._mavparam.get_all_params()
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
            # Cleanup other stuff that doesn't reliably get cleaned up otherwise
            await super().shutdown(force=force)
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
                await self._mavtelem.set_rate_position(self.position_update_rate)
                await self._mavtelem.set_rate_position_velocity_ned(self.position_update_rate)
                await self._mavtelem.set_rate_attitude_euler(self.position_update_rate)
                await self._mavtelem.set_rate_altitude(self.position_update_rate)
                await self._mavtelem.set_rate_battery(self.position_update_rate)
                await self._mavtelem.set_rate_gps_info(self.position_update_rate)
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
        async for arm in self._mavtelem.subscribe_armed():
            self._is_armed = arm

    async def _flightmode_check(self):
        async for flightmode in self._mavtelem.subscribe_flight_mode():
            self._flightmode = flightmode

    async def _inair_check(self):
        async for in_air in self._mavtelem.subscribe_in_air():
            self._in_air = in_air

    async def _gps_check(self):
        async for gps in self._mavtelem.subscribe_gps_info():
            self._gps_info = gps.fix_type

    async def _g_pos_check(self):
        async for pos in self._mavtelem.subscribe_position():
            self._position_g[0] = pos.latitude_deg
            self._position_g[1] = pos.longitude_deg
            self._position_g[2] = pos.absolute_altitude_m
            self._position_g[3] = pos.relative_altitude_m

    async def _vel_rpos_check(self):
        async for pos_vel in self._mavtelem.subscribe_position_velocity_ned():
            self._velocity[0] = pos_vel.velocity.north_m_s
            self._velocity[1] = pos_vel.velocity.east_m_s
            self._velocity[2] = pos_vel.velocity.down_m_s
            self._position_ned[0] = pos_vel.position.north_m
            self._position_ned[1] = pos_vel.position.east_m
            self._position_ned[2] = pos_vel.position.down_m

    async def _att_check(self):
        async for att in self._mavtelem.subscribe_attitude_euler():
            self._attitude[0] = att.roll_deg
            self._attitude[1] = att.pitch_deg
            self._attitude[2] = att.yaw_deg

    async def _heading_check(self):
        async for heading in self._mavtelem.subscribe_heading():
            self._heading = heading.heading_deg

    async def _battery_check(self):
        async for battery in self._mavtelem.subscribe_battery():
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
        async for message in self._mavtelem.subscribe_status_text():
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
        result = await self._error_wrapper(self._mavaction.arm, ActionError)
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
        result = await self._error_wrapper(self._mavaction.disarm, ActionError)
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
        result = await self._error_wrapper(self._mavaction.takeoff, ActionError)
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
            result = await self._error_wrapper(self._mavaction.hold, ActionError)
            target_flight_mode = FlightMode.HOLD
        elif flightmode == "offboard":
            result = await self._error_wrapper(self._mavoffboard.start, OffboardError)
            target_flight_mode = FlightMode.OFFBOARD
        elif flightmode == "return":
            result = await self._error_wrapper(self._mavaction.return_to_launch, ActionError)
            target_flight_mode = FlightMode.RETURN_TO_LAUNCH
        elif flightmode == "land":
            result = await self._error_wrapper(self._mavaction.land, ActionError)
            target_flight_mode = FlightMode.LAND
        elif flightmode == "takeoff":
            self._can_takeoff()
            result = await self._error_wrapper(self._mavaction.takeoff, ActionError)
            target_flight_mode = FlightMode.TAKEOFF
        elif flightmode == "position":
            result = await self._error_wrapper(self._mavmanual.start_position_control, ManualControlError)
            target_flight_mode = FlightMode.POSCTL
        elif flightmode == "altitude":
            result = await self._error_wrapper(self._mavmanual.start_altitude_control, ManualControlError)
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
                self.logger.info(f"New flight mode {self.flightmode}!")
                return True

    async def set_setpoint(self, setpoint: "Waypoint"):
        setpoint_type = setpoint.type
        if setpoint_type == WayPointType.POS_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            return await self._error_wrapper(self._mavoffboard.set_position_ned, OffboardError, point_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self._mavoffboard.set_position_velocity_ned, OffboardError,
                                             point_ned_yaw, velocity_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_ACC_NED:
            yaw = setpoint.yaw
            point_ned_yaw = PositionNedYaw(*setpoint.pos, yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, yaw)
            acc_ned = AccelerationNed(*setpoint.acc)
            return await self._error_wrapper(self._mavoffboard.set_position_velocity_acceleration_ned,
                                             OffboardError,
                                             point_ned_yaw,
                                             velocity_ned_yaw,
                                             acc_ned)
        elif setpoint_type == WayPointType.VEL_NED:
            vel_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self._mavoffboard.set_velocity_ned, OffboardError, vel_yaw)
        elif setpoint_type == WayPointType.VEL_BODY:
            vel_yawrate = VelocityBodyYawspeed(*setpoint.vel, setpoint.yaw_rate)
            return await self._error_wrapper(self._mavoffboard.set_velocity_body, OffboardError, vel_yawrate)
        elif setpoint_type == WayPointType.POS_GLOBAL:
            latitude, longitude, amsl = setpoint.gps
            alt_type = PositionGlobalYaw.AltitudeType.AMSL
            position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=amsl,
                                         yaw_deg=setpoint.yaw, altitude_type=alt_type)
            return await self._error_wrapper(self._mavoffboard.set_position_global, OffboardError, position)
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
        await self._mavaction.goto_location(*gps, yaw)

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
        await self._error_wrapper(self._mavaction.do_orbit, ActionError, radius, velocity, yaw_behaviour,
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
            await self._mavtelem.set_rate_position_velocity_ned(self.position_update_rate)
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
        result = await self._error_wrapper(self._mavaction.land, ActionError)
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
        result = await self._error_wrapper(self._mavmanual.start_position_control, ManualControlError)
        return result

    async def manual_control_altitude(self):
        self.clear_queue()
        self.cancel_action()
        await self.path_follower.deactivate()
        result = await self._error_wrapper(self._mavmanual.start_altitude_control, ManualControlError)
        return result

    async def set_manual_control_input(self, x, y, z, r):
        result = await self._error_wrapper(self._mavmanual.set_manual_control_input, ManualControlError, x, y, z, r)
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
        await self._error_wrapper(self._mavaction.kill, ActionError)
        await super().kill()
        return True

    async def _error_wrapper(self, func, error_type, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except error_type as e:
            self.logger.error(e.result.name)
            return False
        return True