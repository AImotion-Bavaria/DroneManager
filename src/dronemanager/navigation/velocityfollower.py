import math
import numpy as np

import dronemanager
from dronemanager.navigation.core import PathFollower, WayPointType, Waypoint
from dronemanager.utils import dist_ned, heading_ned


class VelocityFollower(PathFollower):
    """Flies directly toward the waypoint facing towards it along the way. Turning towards the target yaw happens
    after we reach the waypoint. Control happens only through velocity setpoints.

    Currently very WIP, drifts off as soon as target positions are reached.
    """
    # TODO: Figure out better way to handle yaw rate

    SETPOINT_TYPES = {WayPointType.VEL_NED}
    WAYPOINT_TYPES = {WayPointType.POS_NED}
    CAN_DO_GPS = False

    def __init__(self, drone: "dronemanager.drone.Drone", logger, dt, max_vel_h=1.0, max_vel_z=0.5, max_acc_h=0.5,
                 max_acc_z=0.25, max_yaw_rate=60):
        super().__init__(drone, logger, dt, setpoint_type=WayPointType.VEL_NED)
        self.max_vel_h = max_vel_h
        self.max_vel_z = max_vel_z
        self.max_acc_h = max_acc_h
        self.max_acc_z = max_acc_z
        self.max_yaw_rate = max_yaw_rate

        self.fudge_yaw = 1
        self.fudge_xy = 1
        self.fudge_z = 1

        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized path follower {self.__class__.__name__}:\n   {attr_string}")

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        """Always move towards target. Accelerates if we are slower than the max speed and have space to accelerate,
        keep speed if we are at max velocity and still some distance away from target, decelerate when we approach
        target.

        :return:
        """
        # Yaw
        target_yaw = waypoint.yaw
        if self.drone.is_at_pos(waypoint.pos, tolerance=1):
            temp_yaw_target = target_yaw
        else:
            temp_yaw_target = heading_ned(self.drone.position_ned, waypoint.pos)
        cur_yaw = self.drone.attitude[2]
        dif_yaw = (temp_yaw_target - cur_yaw + 180) % 360 - 180
        step_size = self.max_yaw_rate * self.dt * self.fudge_yaw
        if abs(dif_yaw) < step_size:
            yaw = temp_yaw_target
        else:
            if dif_yaw > 0:
                yaw = cur_yaw + step_size
            else:
                yaw = cur_yaw - step_size

        # Vertical movement
        cur_z = self.drone.position_ned[2]
        cur_speed_z = abs(self.drone.velocity[2])
        dist_z = abs(waypoint.pos[2] - cur_z)
        speed_z_lim = min(math.sqrt(abs(2 * self.max_acc_z * dist_z)), self.max_vel_z)
        speed_z = min(cur_speed_z + self.max_acc_z * self.dt * self.fudge_z, speed_z_lim)
        vel_z = speed_z if waypoint.pos[2] - cur_z > 0 else -speed_z  # Speed is not velocity -> manually set sign

        # Horizontal
        cur_xy = self.drone.position_ned[:2]
        dist_xy = dist_ned(waypoint.pos[:2], cur_xy)
        cur_vel_xy = self.drone.velocity[:2]
        cur_speed_xy = np.sqrt(cur_vel_xy.dot(cur_vel_xy))
        dist_xy_v = waypoint.pos[:2] - cur_xy
        speed_xy_limit = min(math.sqrt(abs(2 * self.max_acc_h * dist_xy)), self.max_vel_h)
        speed_xy = min(cur_speed_xy + self.max_acc_h * self.dt * self.fudge_xy, speed_xy_limit)
        dir_xy = math.atan2(dist_xy_v[1], dist_xy_v[0])
        vel_x = math.cos(dir_xy) * speed_xy
        vel_y = math.sin(dir_xy) * speed_xy

        vel_yaw_setpoint = Waypoint(WayPointType.VEL_NED, vel=np.asarray([vel_x, vel_y, vel_z]), yaw=yaw)
        await self.drone.set_setpoint(vel_yaw_setpoint)
