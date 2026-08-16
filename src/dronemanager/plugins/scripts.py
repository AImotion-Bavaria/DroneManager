"""Plugin to allow running arbitrary scripts from the CLI.

Sometimes useful for testing, as output and control are bundled in the interface. The scripts must be located in the
"scripts" folder. There is no safety or verification of the scripts at all.
"""
import asyncio
import logging
import os
import pathlib
import subprocess
from concurrent.futures import ProcessPoolExecutor
from subprocess import CompletedProcess, CalledProcessError

import dronemanager.core
from dronemanager.plugin import Plugin


SCRIPT_DIR = pathlib.Path(__file__).parent.parent.joinpath("scripts")


class ScriptsPlugin(Plugin):
    """The plugin for running scripts.

    This plugin has only one CLI command:

    * :py:meth:`execute_script`: Run the given script file. The file can be a path as well, but it must lead to a file
      and be relative to the "scripts" directory.
    """

    PREFIX = "script"
    """The prefix for the CLI commands, "script" by default."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str):
        """Create the ScriptsPlugin.

        Args:
            dm: The associated DroneManager instance.
            logger: The logger used for output and errors.
            name: The name of the plugin.
        """
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "execute": self.execute_script
        }

    async def start(self):
        """Dummy implementation."""
        await super().start()

    async def close(self):
        """Dummy implementation."""
        await super().close()

    async def status(self):
        """Dummy implementation."""
        pass

    async def execute_script(self, script_name: str, script_args: list[str]) \
            -> CompletedProcess | CalledProcessError | None:
        """Run the specified script in a subprocess.

        Args:
            script_name: The name of the script file.
            script_args: Arguments which can be passed to the script file.

        Returns:
            The process result or None if there was an error.
        """
        self.logger.info(f"Executing Script {script_name}")
        script_path = SCRIPT_DIR.joinpath(script_name).as_posix()
        # Ensure script exists
        if not os.path.isfile(script_path):
            self.logger.warning(f"Script {script_name} not found in ./scripts")
            return None
        try:
            # Execute the script
            with ProcessPoolExecutor(max_workers=2) as executor:
                result = await asyncio.get_running_loop().run_in_executor(executor, _script_function,
                                                                          script_path, script_args)
            self.logger.info(f"script Output:\n{result.stdout}")
            return result
        except subprocess.CalledProcessError as e:
            self.logger.warning(f"Script execution failed: {e.stderr}")
            return e
        except Exception as e:
            self.logger.error(f"Exception while executing the script: {repr(e)}")
            return None


def _script_function(script_path: str, script_args: list[str]) -> CompletedProcess:
    """Runs the script in a subprocess.

    Args:
        script_path: The path to the script.
        script_args: Arguments to the script.

    Returns:
        The process result.
    """
    input_args = ["python3", script_path]
    input_args.extend(script_args)
    result = subprocess.run(input_args, capture_output=True, text=True, check=True)
    return result
