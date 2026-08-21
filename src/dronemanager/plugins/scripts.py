import asyncio
import os
import subprocess
from concurrent.futures import ProcessPoolExecutor

from dronemanager.plugin import Plugin


class ScriptsPlugin(Plugin):

    PREFIX = "script"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "execute": self.execute_script
        }

    async def start(self):
        self.logger.debug("Starting Script plugin...")
        await super().start()

    async def close(self):
        self.logger.debug("Closing Script plugin...")
        await super().close()

    async def execute_script(self, script_name: str):
        """Run Script in ./Scripts with given Name"""
        self.logger.info(f"Executing Script {script_name}")
        script_path = os.path.join("Scripts", script_name)
        # Ensure script exists
        if not os.path.isfile(script_path):
            self.logger.warning(f"Script {script_name} not found in ./Scripts")
            return
        try:
            # Execute the script
            with ProcessPoolExecutor(max_workers=2) as executor:
                result = await asyncio.get_running_loop().run_in_executor(executor, script_function, script_path)
            self.logger.info(f"Script Output:\n{result.stdout}")
        except subprocess.CalledProcessError as e:
            self.logger.warning(f"Script execution failed: {e.stderr}")
        except Exception as e:
            self.logger.warning(f"Unexpected error: {repr(e)}")


def script_function(script_path):
    result = subprocess.run(["python3", script_path], capture_output=True, text=True, check=True)
    return result
