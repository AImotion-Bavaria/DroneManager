"""Plugin for integrating OptiTrack systems with Motive into DM.

Connects to a Motive app with streaming enabled, receives pose information for the drones through the stream, performs
coordinate transformations from the Motive to a drone compatible system, assigns it to connected drones and forwards it
as MAVLink messages.

This plugin requires a Motive app with streaming enabled, and drones with tracking markers which are set up as
rigid bodies to be tracked by Motive. Generally, the flight controllers of the drones also need to be configured to
accept external pose information over MAVLink.

Additionally, allows for arbitrary flipping or mirroring across axes to convert the Y-up or Z-up coordinate systems from
Motive to something that common FCs expect.

Usage:
    This plugin is intended to work with an OptiTrack tracking system and will not function without.
    Setup the OptiTrack system and at least one drone to be tracked by the system. Refer to the tracking systems
    documentation for details. The markers on the drone should be configured in Motive as rigid bodies, and Motive
    should be configured to stream these rigid bodies.

    Boot up DroneManager and load the Optitrack plugin. Connect to the Motive stream with
    ``opti-connect --remote=<server_ip> --local=<own_ip>``, where <server_ip> is the IP of the Motive server, and
    <own_ip> is the IP of the machine running DroneManager in the same network.
    Check that the track for the drone is being received with ``opti-list`` by cross-checking the reported position
    against the rigid bodies position in Motive.

    Next, determine your coordinate conversions. There are two key conversions: The first between the rigid body
    system from Motive and the drones body system, and the second between the Motive world and your desired flight
    world coordinate systems. Conversions aren't free rotations, but must be permutations or mirrors along the
    axes.

    For the body conversion, set the drone down in the flight area such that Motive reports no pitch, yaw or roll,
    and determine which of the axis of the Motive rigid body correspond to the drones forward, right and down axes.
    For example, if the drone forward axis is aligned with the rigid body x-axis, its right axis antiparallel to the
    rigid body y-axis and the down axis antiparallel to the rigid body z axis, your body conversion is
    ["x", "-y", "-z"].

    For the world conversion, you can theoretically pick any permutation you desire, but in practice flight
    controllers expect a SO(3) system with the z-axis down. An example system with Motives Z-up setting might be
    ``["y", "-x", "-z"]``.

    The conversion axes can be used with the commands "set-coords-body" or "set-coords-world", e.g.
    ``opti-set-coords-body --x=x --y=-y --z=-z``, or in the configuration file.
    Note that changes in the configuration file require DroneManger to be rebooted to take effect.
    The same set of coordinate conversions is used for all drones, so they should have the same setup in Motive.

    With the coordinate conversions set, you can do ``opti-check-conv <track_id>`` with the track for your drone
    to check that the output angles are what you would expect. Try this with the drone in a few different
    orientations.

    Connect to the drone with DroneManager and start the data forwarding with ``opti-add <drone_name> <track_id>``.
    You should see the position and orientation of the drone jump to that reported by ``opti-check-conv``
"""
import asyncio
from concurrent.futures import Future
import logging
import numpy as np
from scipy.spatial.transform import Rotation
from typing import Any, Callable

import dronemanager.core
from dronemanager.plugin import Plugin
from dronemanager.plugins.NatNet.NatNetClient import NatNetClient


class OptitrackPlugin(Plugin):
    """The plugin for processing data from OptiTrack Motive.

    Receives pose information about tracked rigid bodies from Motive, performs coordinate transforms and forwards the
    information to specified drones. Key parameters of the plugin are the IP address and port of the Motive server, the
    coordinate transforms and a mapping from Motive track IDs to connected drones. These can be managed through CLI
    commands or set in the DroneManager configuration file.

    This plugin has 8 CLI commands:

    * "connect" - :py:meth:`connect`: Connect to a running Motive stream.
    * "list" - :py:meth:`log_available_bodies`: List the available tracks from Motive and their positions in the Motive
      coordinate system. The positions are useful to determine which track belongs to which physical object.
    * "add" - :py:meth:`add_drone`: Add a drone to the tracking plugin. Data from the track is forward to the drone
      until removed.
    * "remove" - :py:meth:`remove_drone`: Remove a drone from the tracking plugin, stopping the data forwarding for
      that drone.
    * "status" - :py:meth:`status`: Log general information about the plugin. Includes configured drones, available
      tracks, and the configuration of the coordinate transformations.
    * "set-coords-world" - :py:meth:`set_coordinates_world`: Set new axes for the world coordinate transformation.
    * "set-coords-body" - :py:meth:`set_coordinates_body`: Set new axes for the world coordinate transformation.
    * "check-conv" - :py:meth:`check_conv`: Get current position and orientation information from one track and
      log extra information about the transformation. Useful to debug coordinate transform issues.

    This plugin uses the NatNetClient code provided by OptiTrack for receiving and decoding the Motive data stream.
    """

    PREFIX: str = "opti"
    """The prefix for the CLI commands."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str,
                 server_ip: str | None = None, local_ip: str | None = None, world_axes: list[str] | None = None,
                 body_axes: list[str] | None = None, log_frames: bool = False):
        """Create the OptiTrack plugin.

        Args:
            dm: The associated DroneManager instance.
            logger: The logger for output and errors.
            name: The name of the plugin.
            server_ip: The default IP for the Motive server. Used by :py:meth:`connect_server` when the corresponding
                argument is not supplied.
            local_ip: The default IP for the machine running DroneManager. Used by :py:meth:`connect_server` when the
                corresponding argument is not supplied.
            world_axes: A list of axes mapping the Motive coordinate system to a target world system for the drones.
            body_axes: A list of axes mapping the Motive rigid bodies coordinate system to the drones body system.
            log_frames: If ``True``, log received data frames from Motive every so often. Default ``False``.
        """
        super().__init__(dm, logger, name)
        #: Available CLI commands.
        self.cli_commands: dict[str, Callable] = {
            "connect": self.connect_server,
            "list": self.log_available_bodies,
            "add": self.add_drone,
            "remove": self.remove_drone,
            "status": self.status,
            "set-coords-world": self.set_coordinates_world,
            "set-coords-body": self.set_coordinates_body,
            "check-conv": self.check_conv,
        }
        self.client: NatNetClient | None = None  #: The NatNetClient object

        #: The IP of machine running the Motive app.
        self.server_ip: str = server_ip if server_ip is not None else "127.0.0.1"
        #: Our own IP.
        self.local_ip: str = local_ip if local_ip is not None else "127.0.0.1"

        #: Dictionary mapping track IDs to connected drone names.
        self._drone_id_mapping: dict[int, str] = {}
        #: A dictionary listing position and orientation (in the motive system) for each track ID.
        #: Useful to determine which track belongs to which drone.
        self.available_bodies: dict[int, tuple[np.ndarray, np.ndarray]] = {}

        self.frame_count: int = 0  #: Count of the number of received frames from Motive. Resets every million frames.
        self.log_rigid_frames: bool = log_frames  #: Logs every few rigid frames, for debugging purposes.
        self.log_every: int = 99  #: Every this many frames, one gets logged for debugging purposes.

        if world_axes is None:
            world_axes = ["x", "z", "-y"]
        if body_axes is None:
            body_axes = ["x", "z", "-y"]
        #: The :py:class:`CoordinateConversion` object that performs the coordinate transforms.
        self.coordinate_transform = CoordinateConversion(*world_axes, *body_axes)
        self._event_loop = asyncio.get_running_loop()  #: Reference to main event loop for callbacks to use.
        self._stopping = False  #: Set to `True` when we close.

        #: Error counter for MAVLink mocap error. These tend to occur in clusters, so this reduces log spam.
        self._err_count = 0

    async def connect_server(self, remote: str = None, local: str = None) -> bool:
        """Connect to a Motive server.

        Note that the Motive app must be configured for streaming.

        Args:
            remote: The IP address of the Motive server.
            local: The IP of the machine running DroneManager.

        Returns:
            Whether we connected to the server or not.
        """
        if self.client is not None:
            self.logger.warning("Already connected to a NatNet server, aborting.")
            return False
        if remote is not None:
            self.server_ip = remote
        if local is not None:
            self.local_ip = local
        self.logger.info(f"Connecting to NatNet Server @ {self.server_ip}")
        client = NatNetClient()
        client.set_client_address(self.local_ip)
        client.set_server_address(self.server_ip)
        client.new_frame_with_data_listener = self._new_frame_callback
        client.set_use_multicast(True)
        conn_good = False
        try:
            is_running = client.run("d")
            if not is_running:
                self.logger.error("Couldn't start the client!")
            else:
                await asyncio.sleep(1)
                if not client.connected():
                    self.logger.error("Couldn't connect to the server!")
                else:
                    conn_good = True
        except ConnectionResetError as e:
            self.logger.warning("Couldn't connect to the server!")
            self.logger.debug(repr(e), exc_info=True)
        if conn_good:
            self.client = client
            self.logger.info("Connected to NatNet Server!")
            return True
        else:
            client.shutdown()
            return False

    async def add_drone(self, name: str, track_id: int):
        """Assign a track to a drone to forward data from the track to the drone.

        Available tracks can be viewed with :py:meth:`log_available_bodies`.

        Args:
            name: The name of the drone.
            track_id: The track ID of the track.
        """
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named {name}!")
        elif track_id not in self.available_bodies:
            self.logger.warning(f"No track {track_id} available!")
            await self.log_available_bodies()
        else:
            self._drone_id_mapping[track_id] = name
            self.logger.info(f"Added drone {name} with track {track_id} to the tracking system.")

    async def remove_drone(self, name: str):
        """Stop forwarding information for a drone.

        Args:
            name: The name of the drone
        """
        if name not in list(self._drone_id_mapping.values()):
            self.logger.warning(f"No drone named {name} in the tracking system!")
        else:
            self._remove_drone(name)

    def _remove_drone(self, name: str):
        """Stop forwarding information for a drone.

        Args:
            name: The name of the drone
        """
        to_remove = None
        for key, value in self._drone_id_mapping.items():
            if value == name:
                to_remove = key
        if to_remove is not None:
            self._drone_id_mapping.pop(to_remove)

    async def log_available_bodies(self):
        """Print available rigid bodies from the Motive server.

        Also shows the position, in the Motive coordinate system, for each body, to allow disambiguation.
        """
        if self.client is None:
            self.logger.warning("Not connected to a NatNet server!")
        body_str = "\n".join([f"Track ID: {track_id}, Position {array[0]}"
                              for track_id, array in self.available_bodies.items()])
        self.logger.info("Available Rigid Bodies and their Motive positions:\n" + body_str)

    def log_coordinate_system(self):
        """Log the coordinate configuration."""
        self.logger.info(f"World axes: {self.coordinate_transform.world_axes}, "
                         f"body axes: {self.coordinate_transform.body_axes}")

    async def status(self):
        """Log status information about the plugin.

        Shows which drones have tracks associated, which tracks are available, and the coordinate configuration.
        """
        if len(self._drone_id_mapping) > 0:
            out_str = "Streaming configured for following drones:\nNAME\tTRACK ID\n"
            for track_id, name in self._drone_id_mapping.items():
                out_str += f"{name}\t{track_id}\n"
            self.logger.info(out_str)
        await self.log_available_bodies()
        self.log_coordinate_system()

    async def set_coordinates_world(self, x: str = "x", y: str = "y", z: str = "z",):
        """Set new axes for the Motive-World coordinate conversion.

        See the :py:class:`CoordinateConversion` documentation for details.

        Args:
            x: The Motive coordinate axis corresponding to the drones bodies x-axis.
            y: The Motive coordinate axis corresponding to the drones bodies y-axis.
            z: The Motive coordinate axis corresponding to the drones bodies z-axis.
        """
        try:
            self.coordinate_transform.set_rotation_world([x, y, z])
        except Exception as e:
            self.logger.error("Couldn't change world coordinate system, see log for details!")
            self.logger.debug(repr(e), exc_info=True)

    async def set_coordinates_body(self, x: str = "x", y: str = "y", z: str = "z",):
        """Set new axes for the rigid-body to drone-body coordinate conversion.

        See the :py:class:`CoordinateConversion` documentation for details.

        Args:
            x: The Motive rigid body axis corresponding to the drones bodies x-axis.
            y: The Motive rigid body axis corresponding to the drones bodies y-axis.
            z: The Motive rigid body axis corresponding to the drones bodies z-axis.
        """
        try:
            self.coordinate_transform.set_rotation_body([x, y, z])
        except Exception as e:
            self.logger.error("Couldn't change body coordinate system, see log for details!")
            self.logger.debug(repr(e), exc_info=True)

    async def check_conv(self, track_id: int, out_sequence: str = "xyz", in_sequence: str = "XYZ"):
        """Log coordinate conversion information for the given track id.

        Useful to check that the conversion is happening properly. All axes sequences follow scipy rotation convention.

        Args:
            track_id: The Motive track ID to be used.
            out_sequence: The axis sequence for the output euler angles, default "xyz" for roll, pitch and yaw.
            in_sequence: The axis sequence to translate the quaternion from Motive to euler angles. This conversion is
                not done during normal operation, but convenient for manual tests. Default, "XYZ".
        """
        if track_id not in self.available_bodies:
            self.logger.warning(f"No track ID {track_id}")
            await self.log_available_bodies()
        else:
            conv = self.coordinate_transform
            pos, quat = self.available_bodies[track_id]
            euler = Rotation.from_quat(quat).as_euler(in_sequence, degrees=True)
            conv_pos, _ = conv.convert_quat(pos, quat, out_sequence=out_sequence, degrees=False)
            _, conv_rot_deg = conv.convert_quat(pos, quat, out_sequence=out_sequence, degrees=True)
            self.logger.info(f"Rotation matrices from axes listing:"
                             f"\nWorld matrix {conv.rotation_world.as_matrix()}\t"
                             f"Body matrix {conv.rotation_body.as_matrix()}")
            self.logger.info(f"Initial and converted position: "
                             f"\nInitial {pos}\nConverted: {conv_pos}")
            self.logger.info(f"Initial and converted angles: "
                             f"\nQuat from Motive: {quat}\nEuler Motive: {euler}"
                             f"\nEuler (RPY) converted {conv_rot_deg}")

    def _new_frame_callback(self, data_dict: dict[str, Any]):
        """Callback function on received data frames from Motive.

        Calls :py:meth:`_process_rigid_body` for each track with an ID in :py:attr:`_drone_id_mapping`.

        Args:
            data_dict: The data from Motive.
        """
        try:
            if not self._stopping:
                body_dict = {}
                rigid_body_list = data_dict["mocap_data"].rigid_body_data.rigid_body_list
                for rb in rigid_body_list:
                    track_id = rb.id_num
                    position = rb.pos
                    rotation = rb.rot
                    body_dict[track_id] = (position, rotation)
                    if track_id in self._drone_id_mapping:
                        self._process_rigid_body(track_id, position, rotation)
                self.available_bodies = body_dict
        except Exception as e:
            self.logger.error("Exception in new frame callback, see log for details.")
            self.logger.debug(repr(e), exc_info=True)

    def _process_rigid_body(self, track_id: int, position: np.ndarray, rotation: np.ndarray):
        """Handle information for a single rigid body.

        Receives a track ID, position and rotation, performs the coordinate transformation from the optitrack coordinate
        system and then forwards the converted position and rotation to the drone linked with the track id.

        Args:
            track_id: The track id whose position and rotation we are receiving.
            position: The drone position received from Optitrack.
            rotation: The drone rotation received from Optitrack.
        """
        try:
            self.frame_count += 1
            self.frame_count = self.frame_count % 1000000
            if self.log_rigid_frames and self.frame_count % self.log_every == 0:
                self.logger.debug(f"Logging every {self.log_every}th rigid body frame RAW: "
                                  f"{track_id} - {position, rotation}")
            if track_id in self._drone_id_mapping:
                drone_name = self._drone_id_mapping[track_id]
                conv_position, conv_rotation = self.coordinate_transform.convert_quat(position,
                                                                                      rotation,
                                                                                      out_sequence="xyz",
                                                                                      degrees=False)
                try:
                    drone = self.dm.drones[drone_name]
                    if drone.is_connected:

                        if self.log_rigid_frames and self.frame_count % self.log_every == 0:
                            self.logger.info(f"Logging every {self.log_every}th rigid body frame CONVERTED: "
                                             f"{track_id} - {conv_position, conv_rotation}")
                        send_task = asyncio.run_coroutine_threadsafe(drone.send_external_tracking_data(conv_position,
                                                                                                       conv_rotation),
                                                                     self._event_loop)
                        asyncio.run_coroutine_threadsafe(self._mocap_awaiter(send_task, self.logger), self._event_loop)
                except KeyError:
                    self.logger.warning(f"Received tracking data for drone '{drone_name}' "
                                        f"which is no longer connected, removing...")
                    self._remove_drone(drone_name)
        except UnboundLocalError:
            pass
        except Exception as e:
            self.logger.error("Exception in rigid body callback, see log for details.")
            self.logger.debug(repr(e), exc_info=True)

    async def close(self):
        """Close the plugin, removing the NatNet callback and shutting down the NatNet client."""
        if self.client is not None:
            self.client.new_frame_with_data_listener = None
        self._stopping = True
        await super().close()
        if self.client is not None:
            self.client.shutdown()

    async def _mocap_awaiter(self, task: asyncio.Future, logger: logging.Logger):
        """Awaits the mocap sending coroutine.

        Used to track exceptions while reducing log spam, as tracking exceptions tend to come in clusters.

        Args:
            task: The task being awaited.
            logger: The logger for errors and output.
        """
        try:
            if isinstance(task, Future) or isinstance(task, asyncio.Future):
                res, _ = await asyncio.wait(task)
                self.logger.error(f"{res}")
                if res:
                    self._err_count = 0
        except asyncio.CancelledError:
            pass
        except Exception as e:
            if self._err_count % self.log_every == 0:
                logger.error("Encountered an exception in a coroutine! See the log for more details")
                logger.debug(repr(e), exc_info=True)
            self._err_count += 1


class CoordinateConversion:
    """Class for coordinate conversions.

    Allows for axes to arbitrarily permutated and flipped and provides conversion functions to convert input positions
    and rotations to be converted to the permuted and flipped coordinate system.

    There are two transformations happening, first between the Motive world coordinate system and the target system,
    and then between the drone body system and the Motive rigid body system. Transforms are defined by providing an
    axes in the origin system for each target axis.

    Consider a lab setup with Motive configured with Y-up and the X- and Z-axis in the horizontal plane. The rigid body
    for the drone was configured such that the forward direction of the drone and the X-axis of the rigid body align.
    For the drone-rigid-body conversion we would then have the drone forward parallel to the body X-axis, its right
    parallel to the Z-axis and its down direction antiparallel to the Y-Axis.
    A desired world coordinate system might have world-X parallel to Motive-Z, world-Y antiparallel to Motive-X and
    world-Z antiparallel to Motive-Y.

    Axes permutations are specified by stating the corresponding Motive axes for each world or drone axis, as a string.
    Antiparallel axis are indicated with a `-` sign.
    So the configuration for the example above would be ``CoordinateConversion("z", "-x", "-y", "x", "z", "-y")``.

    In theory, the axes can be freely chosen, but drone firmware usually expects specific coordinate conventions, such
    as vertical down positive.
    Note that the drone axes are not the current physical rotation of the drone, but rather the conversion between the
    Motive rigid body associated with that drone and the body coordinate system of the drone.
    """

    def __init__(self, world_x: str, world_y: str, world_z: str, drone_forward: str, drone_right: str, drone_down: str):
        """Create the CoordinateConversion.

        Args:
            world_x: Which Motive world axis corresponds to the desired world-X axis.
            world_y: Which Motive world axis corresponds to the desired world-Y axis.
            world_z: Which Motive world axis corresponds to the desired world-Z axis.
            drone_forward: Which Motive rigid body axis corresponds to the drones bodies X axis.
            drone_right: Which Motive rigid body axis corresponds to the drones bodies Y axis.
            drone_down: Which Motive rigid body axis corresponds to the drones bodies Z axis.
        """
        self._choices: list[str] = ["x", "-x", "y", "-y", "z", "-z"]  #: The possible choices for the inputs.
        self._choices.extend([choice.upper() for choice in self._choices])
        assert world_x in self._choices and world_y in self._choices and world_z in self._choices, \
            f"Invalid axis for coordinate conversion, must be one of {self._choices}"
        self.world_axes: list[str] = [world_x, world_y, world_z]  #: The axes of the target world coordinate system.
        #: The axes of the drone coordinate system.
        self.body_axes: list[str] = [drone_forward, drone_right, drone_down]
        self.rotation_world: Rotation | None = None  #: Scipy rotation for the motive-world conversion.
        self.rotation_body: Rotation | None = None  #: Scipy rotation for the rigid-body to drone conversion.
        self._make_rotation()

    def convert_euler(self, tracking_pos: np.ndarray, tracking_euler: np.ndarray, in_sequence: str = "XYZ",
                      out_sequence: str = "ZYX", out_degrees: bool = False, in_degrees: bool = True) \
            -> tuple[np.ndarray, np.ndarray]:
        """Perform the coordinate conversions for a position and euler angles.

        We use the scipy Rotation class for the conversions. For euler angles, the sequence strings define the order in
        which the axis are applied and whether the rotations are extrinsic or intrinsic. See the scipy documentation
        for more details.

        Args:
            tracking_pos: The input position.
            tracking_euler: The input orientation as euler angles.
            in_sequence: The sequence string for the input euler angles.
            out_sequence: The sequence string for the output euler angles.
            out_degrees: Whether the output angles should be in degrees.
            in_degrees: Whether the input angles are in degrees.

        Returns:
            A tuple with the converted position and orientation as euler angles.
        """
        tracking_rot = Rotation.from_euler(in_sequence, tracking_euler, degrees=in_degrees)
        return self._convert(tracking_pos, tracking_rot, out_sequence=out_sequence, degrees=out_degrees)

    def convert_quat(self, tracking_pos: np.ndarray, tracking_quat: np.ndarray,
                     out_sequence: str = "ZYX", degrees: bool = False) \
            -> tuple[np.ndarray, np.ndarray]:
        """Perform the coordinate conversions for a position and a quaternion.

        We use the scipy Rotation class for the conversions. For euler angles, the sequence strings define the order in
        which the axis are applied and whether the rotations are extrinsic or intrinsic. See the scipy documentation
        for more details.

        Args:
            tracking_pos: The input position.
            tracking_quat: The input orientation as a quaternion.
            out_sequence: The sequence string for the output euler angles.
            degrees: Whether the output angles should be in degrees.

        Returns:
            A tuple with the converted position and orientation as euler angles.
        """
        tracking_rot = Rotation.from_quat(tracking_quat)
        return self._convert(tracking_pos, tracking_rot, out_sequence=out_sequence, degrees=degrees)

    def _convert(self, tracking_pos: np.ndarray, tracking_rot: Rotation, out_sequence: str, degrees: bool = False) \
            -> tuple[np.ndarray, np.ndarray]:
        """Performs the actual conversion.

        Args:
            tracking_pos: The input position.
            tracking_rot: The input orientation as a Rotation object.
            out_sequence: The sequence string for the output euler angles.
            degrees: Whether the output angles should be in degrees.

        Returns:
            A tuple with the converted position and orientation as euler angles.
        """
        converted_pos = self.rotation_world.apply(tracking_pos)
        converted_rot = np.asarray((self.rotation_world * tracking_rot * self.rotation_body).as_euler(out_sequence,
                                                                                                      degrees=degrees))
        return converted_pos, converted_rot

    def set_rotation_world(self, world_axes: list[str]):
        """Set new axes for the world-Motive coordinate conversion.

        Args:
            world_axes: The new world axes. Should be a list of three strings, e.g. ``["z", "-x", "-y"]``.
        """
        self.world_axes = world_axes
        self._make_rotation()

    def set_rotation_body(self, body_axes: list[str]):
        """Set new axes for the rigid-body-drone coordinate conversion.

        Args:
            body_axes: The new body axes. Should be a list of three strings, e.g. ``["z", "-x", "-y"]``.
        """
        self.body_axes = body_axes
        self._make_rotation()

    def _make_perm_matrix(self, axes: list[str]) -> np.ndarray:
        """Creates a 3x3 permutation/rotation matrix from a list of axes strings.

        Args:
            axes: The axes from which the matrix will be generated.

        Returns:
            The rotation matrix.
        """
        mat = np.zeros((3, 3))
        for i, axis in enumerate(axes):
            axis = axis.lower()
            neg = axis.startswith("-")
            if axis.endswith("x"):
                pos = 0
            elif axis.endswith("y"):
                pos = 1
            else:
                pos = 2
            mat[i, pos] = -1 if neg else 1
        return mat

    def _make_rotation(self):
        """Create the rotation matrices."""
        self.rotation_world = Rotation.from_matrix(self._make_perm_matrix(self.world_axes))
        self.rotation_body = Rotation.from_matrix(self._make_perm_matrix(self.body_axes).T)
