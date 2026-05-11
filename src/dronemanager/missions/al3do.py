import asyncio
import enum
from dronemanager.plugins.mission import Mission, MissionStage
from dronemanager.navigation.core import Waypoint, WayPointType

class Al3doStages(MissionStage):
    Uninitialized = enum.auto()
    Start = enum.auto()
    Running = enum.auto()
    Finished = enum.auto()

class Al3doMission(Mission):
    """
    Mission to fly a specific square pattern:
    (0,0,-2) -> (0,-1,-2) -> (2,-1,-2) -> (2,1,-2) -> (0,0,-2)
    """
    
    DEPENDENCIES = ["external"]

    def __init__(self, dm, logger, name="al3do"):
        super().__init__(dm, logger, name)
        
        self.waypoints = [
            [0, 0, -2],
            [0, -1, -2],
            [2, -1, -2],
            [2, 1, -2],
            [0, 0, -2]
        ]
        
        self.cli_commands.update({
            "run": self.run_mission,
        })
        
        self.current_stage = Al3doStages.Uninitialized
        self.drone_tasks = set()
        self.position_tolerance = 0.25

    # --- REQUIRED ABSTRACT METHODS ---

    async def reset(self):
        """Mandatory abstract method: Resets mission state."""
        self.logger.info("Resetting square mission.")
        for task in self.drone_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()
        self.current_stage = Al3doStages.Start
        return True

    async def status(self):
        """Mandatory abstract method: Logs current mission status."""
        self.logger.info(f"Mission {self.name}: Stage {self.current_stage.name}. "
                         f"Drones: {list(self.drones.keys())}")

    async def remove_drones(self, names: list[str]):
        """Mandatory abstract method: Removes drones from the mission dictionary."""
        for name in names:
            if name in self.drones:
                self.drones.pop(name)
                self.logger.info(f"Removed {name} from mission.")

    def mission_ready(self, drone):
        """Mandatory abstract method: Logic to check if a specific drone is ready."""
        return self.dm.drones[drone].is_connected

    # --- MISSION LOGIC ---

    async def run_mission(self):
        if not self.ready():
            self.logger.error("Mission not ready. Ensure drones are added.")
            return
        
        self.current_stage = Al3doStages.Running
        self.logger.info("Starting Square Mission Sequence")
        
        new_task = asyncio.create_task(self._fly_sequence())
        self.drone_tasks.add(new_task)

    async def _fly_sequence(self):
        try:
            drone_name = list(self.drones.keys())[0]
            
            # Arm and Takeoff
            self.logger.info(f"Arming {drone_name}...")
            if not await self.dm.arm([drone_name]):
                return

            # Execute Square
            for point in self.waypoints:
                self.logger.info(f"Heading to {point}")
                await self.dm.fly_to(drone_name, local=point, tol=self.position_tolerance)
                await asyncio.sleep(0.5)

            self.logger.info("Pattern finished. Landing.")
            await self.dm.land([drone_name])
            self.current_stage = Al3doStages.Finished
            
        except Exception as e:
            self.logger.error(f"Flight error: {e}")
            self.current_stage = Al3doStages.Uninitialized

    def ready(self):
        drones_ready = all([self.mission_ready(d) for d in self.drones])
        return drones_ready and len(self.drones) > 0

    async def add_drones(self, names: list[str]):
        self.logger.info(f"Adding drones {names} to mission!")
        for name in names:
            if name in self.dm.drones:
                self.drones[name] = self.dm.drones[name]
        self.current_stage = Al3doStages.Start
        return True