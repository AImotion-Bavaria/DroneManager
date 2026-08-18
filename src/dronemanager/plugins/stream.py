"""Module for a basic stream receiving plugin using TCP and OpenCV."""
import asyncio
import logging
import struct
from collections.abc import Callable

import cv2
import numpy as np
from dronemanager.plugin import Plugin
import dronemanager.core


class StreamPlugin(Plugin):
    """Plugin to receive video stream via TCP.

    Designed to work the Holodeck unity plugin out of the box. Images are received, decoded and then sent to
    registered callback functions.

    This plugin has three CLI commands:

    * :py:meth:`start`: Connect to and start processing a stream.
    * :py:meth:`display`: Toggles a live display of the images from the stream in a separate window.
    * :py:meth:`stop`: Close the stream.
    """

    PREFIX = "stream"
    """Used for CLI commands (e.g., 'stream start')"""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str, ip: str = "127.0.0.1",
                 port: int = 5000):
        """Create StreamPlugin.

        Args:
            dm: The assocciated DroneManager instance.
            logger: The logger for error output.
            name: The name of the plugin.
            ip: Default IP, can be set via config.json 'plugin_settings'.
            port: Default Port, can be set via config.json 'plugin_settings'.
        """
        super().__init__(dm, logger, name)
        self.default_ip = ip
        self.default_port = int(port)
        self.running = False
        self.stream_task = None
        self.display_stream = False
        self.callbacks: set[Callable] = set()

        # Register CLI commands
        self.cli_commands = {
            "start": self.start_stream,
            "display": self.display,
            "stop": self.stop_stream
        }

    async def start_stream(self, ip: str = None, port: int = None) -> bool:
        """Starts the Video Stream.

        Args:
            ip: Override the default IP (optional).
            port: Override the default Port (optional).

        Returns:
            Whether the stream started successfully.
        """
        # Resolve IP/Port: CLI Arg -> Config Default -> Hardcoded Default
        target_ip = ip if ip is not None else self.default_ip
        target_port = port if port is not None else self.default_port

        if self.running:
            self.logger.warning("Stream is already running.")
            return False

        self.running = True
        self.logger.info(f"Starting Stream Listener on {target_ip}:{target_port}...")

        # Run the loop as a background task so we don't block the CLI or DroneManager
        self.stream_task = asyncio.create_task(self._stream_loop(target_ip, target_port))
        self.running_tasks.add(self.stream_task)
        return True

    async def stop_stream(self):
        """Stops the running stream and closes the window."""
        self.running = False
        if self.stream_task:
            self.stream_task.cancel()
            try:
                await self.stream_task
            except asyncio.CancelledError:
                pass
        cv2.destroyAllWindows()
        self.logger.info("Stream stopped.")

    async def display(self):
        """Toggles the display of the stream."""
        self.display_stream = not self.display_stream

    async def close(self):
        """Cleanup when plugin is unloaded."""
        await self.stop_stream()
        await super().close()

    async def status(self):
        """Log whether we are currently streaming, displaying, and the registered callbacks."""
        self.logger.info(f"Active: {self.running}, Display: {self.display_stream}, Callbacks: {self.callbacks}")

    def add_callback(self, callback_function: Callable):
        """Add a callback function to be executed on every received frame.

        The callback function must be a plain function, not a coroutine. It is called for every frame, so it should
        complete very quickly. If you wish to have intensive computations, the callback should send the image onward
        to another thread to be processed.

        Args:
            callback_function: The callback function.
        """
        self.callbacks.add(callback_function)

    def remove_callback(self, callback_function: Callable):
        """Remove a callback function from the stream.

        Args:
            callback_function: The callback function to be removed.
        """
        self.callbacks.remove(callback_function)

    async def _stream_loop(self, ip: str, port: int):
        """The main async receiving loop.

        Opens a TCP stream to the target ip and port and continuously reads incoming images. Also processes callbacks.

        Args:
            ip: The IP from which we are receiving the stream.
            port: The port from which we are receiving the stream.
        """
        while self.running:
            reader = None
            writer = None
            try:
                # Open async TCP connection
                reader, writer = await asyncio.open_connection(ip, port)
                self.logger.info("Connected to Stream!")

                while self.running:
                    # 1. Read Length (4 bytes)
                    # readexactly ensures we don't get partial packets
                    length_data = await reader.readexactly(4)
                    (length,) = struct.unpack('<I', length_data)

                    # 2. Read Image Data
                    image_data = await reader.readexactly(length)

                    # 3. Decode and Show
                    nparr = np.frombuffer(image_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                    if frame is not None and self.display_stream:
                        # Display window title includes plugin name
                        cv2.imshow(f"Stream ({self.name})", frame)
                        # 4. Handle UI events without blocking asyncio
                        # waitKey(1) processes GUI events. We assume this runs on Main Thread.
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.running = False
                            break

                    for callback in self.callbacks:
                        try:
                            callback(frame)
                        except Exception as e:
                            self.logger.error(f"Couldn't process stream callback {callback}! See log for details.")
                            self.logger.debug(repr(e), exc_info=True)

                    # Yield control to allow other DroneManager tasks to run
                    await asyncio.sleep(0)

            except (asyncio.IncompleteReadError, ConnectionRefusedError, ConnectionResetError):
                self.logger.debug("Waiting for Stream connection...", exc_info=False)
                # If connection fails, wait 2 seconds before retrying
                await asyncio.sleep(2)
            except Exception as e:
                self.logger.error("Exception in the stream receiving loop! Check logs for details.")
                self.logger.debug(repr(e), exc_info=True)
                await asyncio.sleep(1)
            finally:
                if writer:
                    writer.close()
                    try:
                        await writer.wait_closed()
                    except Exception:
                        pass

        cv2.destroyAllWindows()
