import numpy as np
import math

from dronemanager.navigation.core import Fence, Waypoint, WayPointType


class RectLocalFence(Fence):
    """ Class for rectangular fences in the local coordinate frame.

    Works by defining yis limits: One upper and lower limit for each axis. Waypoints will only be accepted
    if they use local (NED) coordinates and lie within the box between these limits.
    Lower limits but be smaller than upper limits.
    Safety level is used only by the controller functionality.
    """
    def __init__(self, north_lower, north_upper, east_lower, east_upper, down_lower, down_upper, safety_level = 0):
        super().__init__()
        assert north_lower < north_upper and east_lower < east_upper and down_lower < down_upper, \
            "Lower fence limits must be less than the upper ones!"
        assert 0 <= safety_level <= 5 , "Safety Level must be between 0 and 5 and int"
        self.north_lower = north_lower
        self.north_upper = north_upper
        self.east_lower = east_lower
        self.east_upper = east_upper
        self.down_lower = down_lower
        self.down_upper = down_upper
        self.safety_level = int(safety_level)
        self._warned_on_params = set()

    def check_waypoint_compatible(self, point: Waypoint):
        if self.active and point.type in [WayPointType.POS_NED, WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            coord_north, coord_east, coord_down = point.pos
            if (self.north_lower < coord_north < self.north_upper
                    and self.east_lower < coord_east < self.east_upper
                    and self.down_lower < coord_down < self.down_upper):
                return True
        return False

    def controller_safety(self, drone, forward_input, right_input, vertical_input, yaw_input, *args, **kwargs):
        # Common parameters for all axes
        if not drone.parameters_loaded:
            if not drone.name in self._warned_on_params:
                self.logger.warning(f"Drone {drone.name} parameters not loaded, can't perform fence check. Controller "
                                    "inputs may exceed fence!")
                self._warned_on_params.add(drone)
            return forward_input, right_input, vertical_input, yaw_input
        # This needs the params to be loaded, provide a warning if they aren't loaded yet and then just pass on
        max_speed_h = drone.drone_params.max_h_vel if drone.drone_params.max_h_vel >= drone.config.max_h_vel else drone.config.max_h_vel
        max_speed_down = drone.config.max_down_vel if drone.config.max_down_vel >= drone.config.max_down_vel else drone.config.max_down_vel
        max_speed_up = drone.config.max_up_vel if drone.config.max_up_vel >= drone.config.max_up_vel else drone.config.max_up_vel

        speed_limit_horizontal = drone.config.max_h_vel
        speed_limit_down = drone.config.max_down_vel
        speed_limit_up = drone.config.max_up_vel

        acceleration_horizontal = drone.config.max_h_acc / (self.safety_level + 1)
        acceleration_vertical = drone.config.max_v_acc / (self.safety_level + 1)

        x, y, z = drone.position_ned #+ drone.velocity  # Add the speed to compensate for existing motion a little better
        yaw = drone.attitude[2]

        # Vertical is easy: Just check distance and scale accordingly

        def _limit_speed(distance, speed_limit, acceleration):
            # If a distance is negative, we are on the wrong side of the fence
            if distance < 0:
                speed_limit = 0
            else:
                speed_limit = min(speed_limit, math.sqrt(2 * acceleration * (distance + 0.000001)))  # Avoid /0 error
            return speed_limit

        speed_limit_down = _limit_speed(self.down_upper - z, speed_limit_down, acceleration_vertical)
        speed_limit_up = _limit_speed(z - self.down_lower, speed_limit_up, acceleration_vertical)

        def _adjust_input(c_input, speed, speed_limit_a, speed_limit_b, max_speed_a, max_speed_b):
            if speed > 0 and speed > speed_limit_a:
                c_input = c_input * speed_limit_a / max_speed_a
            elif speed < 0 and speed < speed_limit_b:
                c_input = c_input * speed_limit_b / max_speed_b
            return c_input

        vertical_speed = vertical_input * max_speed_up if vertical_input < 0 else vertical_input * max_speed_down
        vertical_input = _adjust_input(vertical_input, vertical_speed, speed_limit_down, speed_limit_up,
                                       max_speed_down, max_speed_up)

        # for horizontal motion, determine the input heading in the fence coordinate system and then determine the
        # distance to the fence following the line from the current position along the input heading, then scale both
        # inputs accordingly

        input_angle_to_drone = -math.atan2(-right_input, forward_input)
        input_angle_to_fence = input_angle_to_drone + yaw * math.pi / 180

        # Distance along line to each boundary
        def _distance_to_edge(limit, pos, trig):
            try:
                distance_to_limit = (limit - pos) / trig
            except ZeroDivisionError:
                distance_to_limit = math.inf
            if distance_to_limit < 0:
                distance_to_limit = math.inf
            return distance_to_limit

        distance_x_upper = _distance_to_edge(self.north_upper, x, math.cos(input_angle_to_fence))
        distance_x_lower = _distance_to_edge(self.north_lower, x, math.cos(input_angle_to_fence))
        distance_y_upper = _distance_to_edge(self.east_upper, y, math.sin(input_angle_to_fence))
        distance_y_lower = _distance_to_edge(self.east_lower, y, math.sin(input_angle_to_fence))

        distance_horizontal = min(distance_x_upper, distance_x_lower, distance_y_lower, distance_y_upper)

        # If both entries for an axis are positive, we are outside the boundary and trying to move in and should use
        # the larger distance, but the minimum of the other axis as the speed limit
        # If both entries are inf, then we are outside and trying to move further outside, and we should have speed limit 0
        if not math.isinf(distance_x_lower) and not math.isinf(distance_x_upper):
            distance_horizontal = min(max(distance_x_lower, distance_x_upper), distance_y_lower, distance_y_upper)
        elif math.isinf(distance_x_lower) and math.isinf(distance_x_upper):
            distance_horizontal = 0
        if not math.isinf(distance_y_lower) and not math.isinf(distance_y_upper):
            distance_horizontal = min(max(distance_y_lower, distance_y_upper), distance_x_lower, distance_x_upper)
        elif math.isinf(distance_y_lower) and math.isinf(distance_y_upper):
            distance_horizontal = 0

        speed_limit_horizontal = _limit_speed(distance_horizontal, speed_limit_horizontal, acceleration_horizontal)
        scaling_factor = speed_limit_horizontal / max_speed_h

        forward_input *= scaling_factor 
        right_input *= scaling_factor

        def clamp_output_values(output):
            if output > .99:
                output = .99
            if output < -.99:
                output = -.99
            return output

        forward_input = clamp_output_values(forward_input)
        right_input = clamp_output_values(right_input)
        vertical_input = clamp_output_values(vertical_input)

        #self.logger.info(f"{scaling_factor, speed_limit_horizontal, drone.config.max_h_vel, distance_horizontal, acceleration_horizontal}")

        return forward_input, right_input, vertical_input, yaw_input

    @property
    def bounding_box(self) -> np.ndarray:
        return np.asarray([self.north_lower, self.north_upper, self.east_lower, self.east_upper, self.down_lower, self.down_upper])

    def __str__(self):
        return (f"{self.__class__.__name__}, with limits N {self.north_lower, self.north_upper}, "
                f"E {self.east_lower, self.east_upper} and D {self.down_lower, self.down_upper}, "
                f"Safety Level: {self.safety_level}")


def _clamp_axis(lower_limit, upper_limit, raw_input, drone_position, dt):
    if raw_input > 0 and drone_position + raw_input * dt >= upper_limit:
        clamped_input = max(0.0, (upper_limit - drone_position) / dt)
    elif raw_input < 0 and drone_position + raw_input * dt <= lower_limit:
        clamped_input = min(0.0, (lower_limit - drone_position) / dt)
    else:
        clamped_input = raw_input
    return clamped_input