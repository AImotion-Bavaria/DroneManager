import dronemanager
from dronemanager.navigation.core import PathGenerator, WayPointType


class DirectTargetGenerator(PathGenerator):
    """Simply sends the target waypoint as static setpoints.
    """

    CAN_DO_GPS = True
    WAYPOINT_TYPES = {WayPointType.POS_NED, WayPointType.POS_GLOBAL}

    def __init__(self, drone: "dronemanager.drone.Drone", logger, waypoint_type, *args, **kwargs):
        super().__init__(drone, logger=logger, waypoint_type=waypoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized path generator {self.__class__.__name__}:\n   {attr_string}")
        self.new_target = False

    async def create_path(self):
        self.new_target = True
        return True

    def next(self):
        """Returns the target position exactly once and then None."""
        if self.new_target:
            self.new_target = False
            return self.target_position
        else:
            return None
