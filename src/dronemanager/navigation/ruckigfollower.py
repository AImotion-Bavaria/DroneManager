import time

import ruckig

from dronemanager.navigation.core import PathFollower, WayPointType, Waypoint
from dronemanager.utils import ned_from_gps
import dronemanager


class RuckigOfflineFollower(PathFollower):

    CAN_DO_GPS = True

    SETPOINT_TYPES = {WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED,
                      WayPointType.POS_GLOBAL}

    def __init__(self, drone: "dronemanager.drone.Drone", logger, dt: float, setpoint_type,
                 max_vel = 10.0, max_acc = 2.0, max_jerk = 1.0,
                 max_down_vel = 1.0, max_up_vel = 3.0, max_v_acc = 0.5, max_v_jerk = 1.0,
                 max_yaw_vel = 60, max_yaw_acc = 30, max_yaw_jerk = 30):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized path follower {self.__class__.__name__}:\n   {attr_string}")
        self.planner = None
        self.planner_input = None
        self.planner_output = None
        self.yaw_planner = None
        self.yaw_input = None
        self.yaw_output = None
        self.max_velocity = [max_vel, max_vel, max_down_vel]
        self.min_velocity = [-max_vel, -max_vel, -max_up_vel]
        self.max_acceleration = [max_acc, max_acc, max_v_acc]
        self.max_jerk = [max_jerk, max_jerk, max_v_jerk]
        self.max_yaw_vel = max_yaw_vel
        self.max_yaw_acc = max_yaw_acc
        self.max_yaw_jerk = max_yaw_jerk

        self.traj_start_time = None
        self.init_yaw = 0.0

    def activate(self):
        self.planner = ruckig.Ruckig(3)
        self.planner_input = ruckig.InputParameter(3)
        self.planner_output = ruckig.Trajectory(3)
        self.planner_input.max_velocity = self.max_velocity
        self.planner_input.min_velocity = self.min_velocity
        self.planner_input.max_acceleration = self.max_acceleration
        self.planner_input.max_jerk = self.max_jerk
        self.yaw_planner = ruckig.Ruckig(1)
        self.yaw_input = ruckig.InputParameter(1)
        self.yaw_output = ruckig.Trajectory(1)
        self.yaw_input.max_velocity = [self.max_yaw_vel]
        self.yaw_input.max_acceleration = [self.max_yaw_acc]
        self.yaw_input.max_jerk = [self.max_yaw_jerk]
        super().activate()

    async def deactivate(self):
        del self.planner_input
        del self.planner_output
        del self.planner
        del self.yaw_planner
        del self.yaw_input
        del self.yaw_output
        self.planner_input = None
        self.planner_output = None
        self.planner = None
        self.yaw_input = None
        self.yaw_output = None
        self.yaw_planner = None
        await super().deactivate()

    def get_next_waypoint(self) -> bool:
        if self.current_waypoint is None:
            return True
        elif self.current_waypoint.type is WayPointType.POS_GLOBAL:
            return self.drone.is_at_waypoint(self.current_waypoint)
        else:
            return self.drone.is_at_pos(self.current_waypoint.pos) and self.drone.is_at_heading(self.current_waypoint.yaw)
        #return self.current_waypoint is None or self.drone.is_at_waypoint(self.current_waypoint)

    async def set_setpoint(self, waypoint):
        # If we don't have a trajectory, create one
        try:
            if self._is_waypoint_new:
                # Position
                # If we have a GPS waypoint, determine NED offset between current GPS pos and target GPS pos
                # Save result in waypoint
                if waypoint.type is WayPointType.POS_GLOBAL:
                    ned_offset = ned_from_gps(self.drone.position_global, waypoint.gps)
                    waypoint.pos = self.drone.position_ned + ned_offset

                self.planner_input.current_position = self.drone.position_ned
                self.planner_input.current_velocity = self.drone.velocity
                self.planner_input.current_acceleration = [0, 0, 0]
                self.planner_input.target_position = waypoint.pos
                # Unfortunately ruckig trajectories increase in position indefinitely if target velocity isn't 0 and we
                # don't have another cancel condition
                #if waypoint.type in [WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
                #    self.planner_input.target_velocity = waypoint.vel
                #if waypoint.type is WayPointType.POS_VEL_ACC_NED:
                #    self.planner_input.target_acceleration = waypoint.acc
                res = self.planner.calculate(self.planner_input, self.planner_output)
                # Yaw
                self.init_yaw = self.drone.attitude[2]
                yaw_offset = (waypoint.yaw - self.init_yaw + 180) % 360 - 180  # Offset from current yaw in +- 180°
                self.yaw_input.current_position = [0]
                self.yaw_input.current_velocity = [0]
                self.yaw_input.current_acceleration = [0]
                self.yaw_input.target_position = [yaw_offset]
                yaw_res = self.yaw_planner.calculate(self.yaw_input, self.yaw_output)
                # Result handling
                if res == ruckig.Result.Working and yaw_res == ruckig.Result.Working:
                    self.logger.debug("Generated trajectory...")
                    self.traj_start_time = time.time()
                else:
                    self.logger.warning("Path follower is failing to produce setpoints, switching to 'HOLD' and deactivating")
                    await self.drone.change_flight_mode("hold")
                    await self.deactivate()
            # Set setpoint
            cur_time = time.time()-self.traj_start_time
            pos, vel, acc = self.planner_output.at_time(cur_time)
            setpoint_offset, yaw_vel, yaw_acc = self.yaw_output.at_time(cur_time)
            setpoint_offset = setpoint_offset[0]
            yaw_setpoint = (setpoint_offset + self.init_yaw + 180) % 360 - 180  # Actual target yaw in -180 - +180
            setpoint = Waypoint(WayPointType.POS_VEL_ACC_NED, pos=pos, vel=vel, acc=acc, yaw=yaw_setpoint)
            if self.drone.check_waypoint(setpoint):
                await self.drone.set_setpoint(setpoint)
                #self.logger.debug(f"Set setpoint: {setpoint}")
            else:
                self.logger.warning("Generated setpoint is invalid, keeping previous.")
        except Exception as e:
            self.logger.warning("Exception in follower with determining setpoint!")
            self.logger.debug(repr(e), exc_info=True)


class RuckigOnlineFollower(PathFollower):
    # TODO: Doesn't work right atm, find out why

    CAN_DO_GPS = False

    SETPOINT_TYPES = {WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    def __init__(self, drone: "dronemanager.drone.Drone", logger, dt: float, setpoint_type,
                 max_vel = 10.0, max_acc = 2.0, max_jerk = 1.0,
                 max_v_vel = 1.0, max_v_acc = 0.5, max_v_jerk = 1.0):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized path follower {self.__class__.__name__}:\n   {attr_string}")
        self.planner = None
        self.planner_input = None
        self.planner_output = None
        self.max_velocity = [max_vel, max_vel, max_v_vel]
        self.max_acceleration = [max_acc, max_acc, max_v_acc]
        self.max_jerk = [max_jerk, max_jerk, max_v_jerk]

        self.error_count = 0
        self.max_error_count = 5

    def activate(self):
        self.planner = ruckig.Ruckig(3)
        self.planner_input = ruckig.InputParameter(3)
        self.planner_output = ruckig.OutputParameter(3)
        self.planner_input.max_velocity = self.max_velocity
        self.planner_input.max_acceleration = self.max_acceleration
        self.planner_input.max_jerk = self.max_jerk
        super().activate()

    async def deactivate(self):
        del self.planner_input
        self.planner_input = None
        del self.planner_output
        self.planner_output = None
        del self.planner
        self.planner = None
        await super().deactivate()

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        # TODO: yaw + fence
        self.planner_input.current_position = self.drone.position_ned
        self.planner_input.current_velocity = self.drone.velocity
        self.planner_input.current_acceleration = self.planner_output.new_acceleration  # Don't get acceleration from drone so use last target acceleration as value
        self.planner_input.target_position = waypoint.pos
        if waypoint.type in [WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            self.planner_input.target_velocity = waypoint.vel
        if waypoint.type == WayPointType.POS_VEL_ACC_NED:
            self.planner_input.target_acceleration = waypoint.acc
        res = self.planner.update(self.planner_input, self.planner_output)
        if res is ruckig.Result.Working:
            pos = self.planner_output.new_position
            vel = self.planner_output.new_velocity
            acc = self.planner_output.new_acceleration
            yaw = waypoint.yaw
            setpoint = Waypoint(WayPointType.POS_VEL_ACC_NED, pos=pos, vel=vel, acc=acc, yaw=yaw)
            await self.drone.set_setpoint(setpoint)
        else:
            # TODO: Come up with something better to do here
            self.logger.warning("Path follower is failing to produce setpoints, switching to 'HOLD' and deactivating")
            await self.drone.change_flight_mode("hold")
            await self.deactivate()


