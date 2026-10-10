"""PX4 software-in-the-loop (SITL) instances for the pipeline tests.

PX4 runs with its built-in SIH simulator (airframe ``10040_sihsim_quadx``), so no Gazebo is needed. On Linux, PX4 is
started directly. On Windows, PX4 runs inside WSL, which is how DroneManager is usually used with simulated drones.

Configuration with environment variables:

- ``PX4_DIR``: The PX4-Autopilot checkout with a built SITL target (``make px4_sitl_default``). On Windows, this is a
  path inside WSL. Defaults to ``~/PX4-Autopilot``.
- ``PX4_WSL_DISTRO``: The WSL distribution with PX4, Windows only. Defaults to the default distribution.

PX4 exits when it is rebooted, i.e. by ``param-apply --verify``. A small runner script therefore restarts PX4 with the
same working directory until the instance is stopped, just like a flight controller boots again after a reboot.
"""
import asyncio
import logging
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
import os
from typing import Callable

LOGGER = logging.getLogger("sitl")

SITL_DRONE = "sim"
"""Name of the simulated drone in the SITL tests."""

PX4_READY_MESSAGE = "Ready for takeoff"
"""PX4 prints this once the vehicle is armable, i.e. the estimator has converged."""

_RUNNER_SCRIPT = """#!/usr/bin/env bash
# Runs PX4 SITL in WORKDIR and starts it again after it exits, i.e. after a reboot, until the file "stop" exists.
WORKDIR="$1"; BUILD="$2"; INSTANCE="$3"
cd "$WORKDIR" || exit 1
echo $$ > runner.pid
while [ ! -f stop ]; do
    echo "=== PX4 start $(date +%T)" >> out.log
    PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim "$BUILD/bin/px4" -i "$INSTANCE" -d "$BUILD/etc" -w "$WORKDIR" \\
        >> out.log 2>&1 < /dev/null &
    echo $! > px4.pid
    wait $!
    sleep 1
done
"""


class SitlSetupError(RuntimeError):
    """PX4 SITL is not installed or can't be started."""


class Px4Sitl:
    """A PX4 SIH SITL instance, either native (Linux) or in WSL (Windows)."""

    def __init__(self, instance: int = 0, px4_dir: str | None = None, wsl_distro: str | None = None):
        """Create the instance description. PX4 is started with :py:meth:`start`.

        Args:
            instance: The PX4 instance number. Each instance uses its own ports, so several can run at once.
            px4_dir: The PX4-Autopilot checkout, defaults to the PX4_DIR environment variable or ~/PX4-Autopilot. On
              Windows a path inside WSL.
            wsl_distro: The WSL distribution on Windows, defaults to the PX4_WSL_DISTRO environment variable or the
              default distribution.
        """
        self.instance = instance
        self.use_wsl = sys.platform == "win32"
        self.px4_dir = px4_dir or os.environ.get("PX4_DIR") or "~/PX4-Autopilot"
        self.wsl_distro = wsl_distro or os.environ.get("PX4_WSL_DISTRO")
        self.workdir: str | None = None
        self._process: subprocess.Popen | None = None
        self._local_tmp: tempfile.TemporaryDirectory | None = None
        self._log_offset = 0

    # Shell helpers ###################################################################################################

    def _command(self, script: str) -> list[str]:
        """Build the command to run a bash script natively or in WSL.

        Args:
            script: The bash script.

        Returns:
            The command as a list for subprocess.
        """
        if self.use_wsl:
            command = ["wsl.exe"]
            if self.wsl_distro:
                command += ["-d", self.wsl_distro]
            return command + ["--", "bash", "-c", script]
        return ["bash", "-c", script]

    def _run(self, script: str, check: bool = True, input_text: str | None = None) -> str:
        """Run a bash script natively or in WSL and return its output.

        Args:
            script: The bash script.
            check: Raise an exception if the script fails.
            input_text: Text passed to the script on stdin.

        Returns:
            The standard output of the script.

        Raises:
            SitlSetupError: If the script fails and check is True.
        """
        # Bytes instead of text mode: On Windows, text mode turns "\n" into "\r\n", which breaks bash scripts in WSL.
        stdin = input_text.encode() if input_text is not None else None
        result = subprocess.run(self._command(script), capture_output=True, input=stdin, timeout=60)
        stdout = result.stdout.decode(errors="replace")
        if check and result.returncode != 0:
            raise SitlSetupError(f"Command failed ({result.returncode}): {script}\n{stdout}"
                                 f"{result.stderr.decode(errors='replace')}")
        return stdout

    @property
    def build_dir(self) -> str:
        """The PX4 SITL build directory, with ``~`` expanded by the shell.

        Returns:
            The path of the build directory, inside WSL on Windows.
        """
        return f"{self.px4_dir}/build/px4_sitl_default"

    # Life cycle ######################################################################################################

    def check_installation(self):
        """Check that PX4 SITL is built, with a helpful message if not.

        Raises:
            SitlSetupError: If WSL or the PX4 SITL build is missing.
        """
        if self.use_wsl and shutil.which("wsl.exe") is None:
            raise SitlSetupError("The SITL tests need PX4 in WSL, but WSL is not installed. See 'Running the tests' "
                                 "in the developer guide, or deselect them with -m \"not sitl\".")
        build = self.build_dir.replace("~", "$HOME", 1)
        output = self._run(f'test -x "{build}/bin/px4" && echo found || echo missing', check=False)
        if "found" not in output:
            where = f"WSL ({self.wsl_distro or 'default distribution'})" if self.use_wsl else "this machine"
            raise SitlSetupError(f"No PX4 SITL build at {self.build_dir} on {where}. Build it with "
                                 f"'make px4_sitl_default' in the PX4-Autopilot directory, set PX4_DIR, or "
                                 f"deselect the SITL tests with -m \"not sitl\". See 'Running the tests' in the "
                                 f"developer guide.")

    def start(self, timeout: float = 60):
        """Start PX4 in a fresh working directory and wait until it is ready for takeoff.

        A fresh working directory means all parameters start at their defaults.

        Args:
            timeout: How long to wait for PX4 to be ready, in seconds.
        """
        self.check_installation()
        name = f"dm_sitl_{self.instance}_{uuid.uuid4().hex[:8]}"
        if self.use_wsl:
            self.workdir = f"/tmp/{name}"
        else:
            self._local_tmp = tempfile.TemporaryDirectory(prefix=f"{name}_")
            self.workdir = self._local_tmp.name
        build = self.build_dir.replace("~", "$HOME", 1)
        self._run(f'mkdir -p "{self.workdir}" && cat > "{self.workdir}/run_px4.sh"', input_text=_RUNNER_SCRIPT)
        script = f'exec bash "{self.workdir}/run_px4.sh" "{self.workdir}" "{build}" {self.instance}'
        LOGGER.info(f"Starting PX4 SITL instance {self.instance} in {self.workdir}")
        self._process = subprocess.Popen(self._command(script), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
        self.wait_ready(timeout)

    def wait_ready(self, timeout: float = 60):
        """Wait until PX4 reports that it is ready for takeoff, i.e. after starting or after a reboot.

        Only output since the last call is considered, so this also detects that PX4 is ready again after a reboot.

        Args:
            timeout: How long to wait, in seconds.

        Raises:
            SitlSetupError: If PX4 isn't ready in time.
        """
        deadline = time.monotonic() + timeout
        position = -1
        while position < 0:
            log = self.log()
            position = log.find(PX4_READY_MESSAGE, self._log_offset)
            if position < 0:
                if self._process is not None and self._process.poll() is not None:
                    raise SitlSetupError(f"PX4 SITL exited during startup:\n{log[-3000:]}")
                if time.monotonic() > deadline:
                    raise SitlSetupError(f"PX4 SITL was not ready after {timeout}s:\n{log[-3000:]}")
                time.sleep(0.5)
        self._log_offset = position + len(PX4_READY_MESSAGE)

    def log(self) -> str:
        """Get the PX4 console output of all runs of this instance.

        Returns:
            The console output, or an empty string before the first start.
        """
        if self.workdir is None:
            return ""
        return self._run(f'cat "{self.workdir}/out.log" 2>/dev/null', check=False)

    def stop(self):
        """Stop PX4 and the runner, then remove the working directory. Does nothing if PX4 isn't running."""
        if self.workdir is not None:
            self._stop_running()

    def _stop_running(self):
        """Stop the running PX4 and the runner and remove the working directory."""
        LOGGER.info(f"Stopping PX4 SITL instance {self.instance}")
        self._run(f'cd "{self.workdir}" && touch stop && kill $(cat runner.pid px4.pid 2>/dev/null) 2>/dev/null; '
                  f'sleep 1; kill -9 $(cat px4.pid 2>/dev/null) 2>/dev/null; true', check=False)
        if self._process is not None:
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._process.kill()
        if self.use_wsl:
            self._run(f'rm -rf "{self.workdir}"', check=False)
        elif self._local_tmp is not None:
            self._local_tmp.cleanup()
        self.workdir = None

    # Connection ######################################################################################################

    def address(self) -> str:
        """Get the DroneManager connection string for this instance.

        On Linux, and with mirrored WSL networking, DroneManager listens on the port that PX4's onboard MAVLink
        instance sends to. With WSL's default NAT networking, PX4 runs on a different IP address, so DroneManager
        connects to PX4's GCS MAVLink port on the WSL IP and PX4 answers to the Windows host.

        Returns:
            The connection string.
        """
        if self.use_wsl and self._wsl_networking_mode() != "mirrored":
            wsl_ip = self._run("hostname -I").split()[0]
            return f"udp://{wsl_ip}:{18570 + self.instance}"
        return f"udp://:{14540 + self.instance}"

    def _wsl_networking_mode(self) -> str:
        """Get the WSL networking mode.

        Returns:
            "mirrored", "nat" or another mode reported by WSL. "nat" if it can't be determined.
        """
        output = self._run("wslinfo --networking-mode 2>/dev/null", check=False).strip()
        return output or "nat"


def px4_log_excerpt(sitl: Px4Sitl, lines: int = 60) -> str:
    """Get the end of the PX4 console output, for test failure reports.

    Args:
        sitl: The SITL instance.
        lines: How many lines to return.

    Returns:
        The last lines of the PX4 output.
    """
    return "\n".join(sitl.log().splitlines()[-lines:])


async def wait_until(condition: Callable[[], bool], timeout: float, interval: float = 0.2) -> bool:
    """Wait until a condition is true.

    Args:
        condition: The condition to check.
        timeout: How long to wait, in seconds.
        interval: How often to check, in seconds.

    Returns:
        Whether the condition became true in time.
    """
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while loop.time() < deadline:
        if condition():
            return True
        await asyncio.sleep(interval)
    return condition()
