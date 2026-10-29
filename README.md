[![CI](https://github.com/AImotion-Bavaria/DroneManager/actions/workflows/ci.yml/badge.svg)](https://github.com/AImotion-Bavaria/DroneManager/actions/workflows/ci.yml)
[![Release](https://github.com/AImotion-Bavaria/DroneManager/actions/workflows/release.yml/badge.svg)](https://github.com/AImotion-Bavaria/DroneManager/actions/workflows/release.yml)

# DroneManager

A package and terminal interface to connect and control multiple drones using MAVLink.

![Screenshot of the tools terminal interface](https://github.com/AImotion-Bavaria/DroneManager/blob/main/docs/imgs/tool-screenshot.png)

[A full API documentation and user guides are available on ReadTheDocs.](https://dronemanager.readthedocs.io)

> [!NOTE]  
> The documentation is currently a work in progress.


## Table of Contents:

- [Installation](#installation)
- [Usage](#usage)
  - [Terminal Interface](#terminal-interface)
  - [Configuration and logs](#configuration-and-logs)
  - [Plugins](#plugins)
- [Examples](#examples)
  - [Holodeck](#holodeck)
  - [UAM](#uam-demo)
- [Citation](#citation)

## Installation

This package is available on PyPI:
```
pip install DroneManager
```

If you plan to develop with DroneManager we recommend an editable install from the repository:
1. Clone this repository
2. Move into the root directory of the repository
3. Install with pip:
```
pip install --upgrade pip
pip install -e .
```

As part of the installation a command called ```dm``` is installed, which starts the terminal interface. 
Alternatively you can run the app.py script.

## Usage

### Terminal interface

There are a large number of possible commands. The basic ones are listed below. Feedback is provided in the log pane.
Exception information, such as stack traces, is additionally logged in the log files. 

Many commands can be "scheduled" by adding the flag `-s`, which means that the drone will finish any previous commands 
before proceeding to the scheduled command. Multiple commands can be scheduled at once. Entering a command that isn't 
scheduled clears the schedule, i.e. the drone will follow it immediately. 

A command is considered "complete" when some condition is met, depending on the command, or when it errors out, either 
because of an exception or because the flight controller denied the command, for example when trying to arm a drone 
without a GPS fix. Commands for multiple drones complete independently, i.e. if you schedule a takeoff and a move for 
two drones, but one of the drones doesn't reach the takeoff altitude, the other one will still start its move once its 
takeoff has completed.

The syntax below is as follows: 
- `<Parameters>` are mandatory positional parameters. 
- `<Parameters?>` are optional positional parameters.
- `-p` are boolean flags.
- `-p <parameter: defaultValue>` are optional parameters with a flag to indicate that they are being supplied. Usually, 
these have a default value.

You can also add `-h` or `--help` to print a help string, either for the whole interface or a specific command. The 
help string for plugin commands is sparser than core commands.

#### Commanding drones

- `connect <name> <connection-string?> -t <timeout: 30> -f <frequency: None>`: Connect to a drone. The parameter name is an arbitrary label that 
is used to refer to the drone with other commands. If a name matching an entry in the config file is used, the configuration from the file will
be loaded. The connection string, for example "udp://192.168.0.143:14550", defines how to connect to the drone. This parameter is optional,
by default "udp://:14540" is used. With `-t` a timeout in seconds can be specified, the default is 30s. The parameter `-f` specifies the
telemetry frequency from the drone. A number of messenges from the drone are requested at this rate, such as position. A number of
components also use this frequency for their own purposes, such as time discretization of trajectories. If this parameter is omitted, the
default value from the config file is used.
- `disconnect <names> -f`: Close the connection to the specified drones. This command will refuse if the drones are 
armed or flying, add the `-f` flag to force disconnect.
- `arm <names> -s`: Arm one or more drones. Multiple drones can be armed at once by listing their name with a space 
between, i.e. `arm drone1 drone2 drone3`. This command is considered complete when the drone is armed and can be 
scheduled.
- `disarm <names> -s`: Disarm one or more drones. This command is complete when the drone has disarmed and 
can be scheduled.
- `takeoff <names> -a <altitude: 2> -s`: Perform a takeoff with the specified drones. The optional altitude 
parameter specifies the target altitude above the launch point. This command is complete when the drone has reached the 
target position and can be scheduled.
- `land <names> -s`: Land the specified drones at their current locations. This puts the drones into offboard mode! 
This command is complete when the drone has landed and can be scheduled.
- `flyto <name> <x> <y> <z> <yaw?> -t <tolerance: 0.25> -s`: Fly the specified drone to the position x, y, z in the 
local coordinate system. This puts the drones into offboard mode! The optional parameter `yaw` defines the facing of
the drone. The heading change and movement usually happen simultaneously. The optional parameter `-t` can be used to
specify a tolerance for how close the drone must be to the target position to have "reached" it. By default, this is
0.25m. This command is complete when the drone is within the tolerance of the target position and can be scheduled.
- `flytogps <name> <lat> <long> <amsl> <yaw?> -t <tolerance: 0.25> -s`: Fly the specified drone the provided GPS position.
Otherwise identical to `flyto`.

And many more!

### Configuration and logs

To simply working with a variety of drones, a number of parameters can be set per-drone in a config file. This allows you 
to save a name with connection string and a number of other parameters, such as acceleration limits, which will be loaded
and used automatically when `connect` with a corresponding name is called.
To view the location of the configuration file, either enter `config` in the CLI or check CONFIG_FILE attribute in utils.
All information in the log pane as well as extra information, such as stacktraces, are saved in log files. Check their 
location by entering `logs` in the CLI.

### Plugins

DroneManager comes with a plugin system for adding extra functionality! The core element are plugin modules, located in 
the "plugins" folder. Each plugin module contains one plugin class, which is a subclass of `plugin.Plugin`, and defines 
extra behaviour. The base class provides a framework for automatically generating CLI commands and booting up any 
background functions. See the plugin documentation (TODO) for more information. A number of plugins are shipped with 
DroneManager.

- `plugins`: Shows a list of available plugins.
- `load <name>`: Load a plugin by name. This must match one of the names shown by `plugins`.
- `unload <name>`: Unload a plugin by name.
- `loaded`: List currently loaded plugins.

By default, the `mission`, `controllers` and `external` plugins are loaded at startup. 
Controllers allows for manual control of drones using common gamepads. It uses pygame and provides functions for creating 
custom button and axis bindings.
Missions are essentially a special kind of plugin. They go into their own folder "missions". Do not try out missions with 
real drones without understanding what they do first!

- `mission-status`: Prints a list of all available missions, as well as an overview for each currently running mission.
- `mission-load <name> <label?>`: Load a mission by name. This must match one of the missions returned by 
`mission-status`. The optional parameter `label` can be used to assign the mission a specific name. Each mission must 
have a unique name, so this allows multiple missions of the same "type".

The plugin `external` creates a local UDP server that sends out information about connected drones and any running missions 
on request using json format. We provide a dummy client script that sends the request messages and prints the information
from DM to console.

#### Other Plugins

- `gimbal`: A plugin for controlling gimbals connected to the drone FC.
- `camera`: A plugin for controlling cameras connected to the drone FC. Quite specific for our camera and probably not generally applicable.
- `sensor`: A plugin for managing generic extra data sources. We also have example code for a specific sensor: An Ecowitt weather station.
- `scripts`: A plugin that allows executing arbitrary scripts from the terminal interface. They are executed in a separate process.
- `optitrack`: A plugin using the python code from the NatNetSDK. Can be used to connect to a running motive server and
  forward tracking information to any connected drones. This also requires that motive be configured to stream the information and that
  the flight controller on the drones themselves be configured to use this information.

## Examples

### Holodeck

Holodeck is an example for the integration of DroneManager with modern simulation environments like Unity. 
It allows Users to control a real drone with a Playstation Controller via DroneManager while experiencing the virtual 
flight in a Unity Environment with enhanced perception through VR-Glasses (e.g. Meta-Quest Pro). Read the [Tutorial](https://dronemanager.readthedocs.io/en/latest/holodeck.html) for more information.

### UAM Demo

This is a showcase in which multiple drones search for and continuously observe a point of interest, for example during a
rescue mission.
To hightlight the advantage of multiple autonomously coordinating drones, there is also a phase with only a single drone
performing both the search and the observation phase.
Setup instructions for both real and simulated drones are available [here](https://dronemanager.readthedocs.io/en/latest/usage.html#example-mission).


## Citation

If you use DroneManager in your work, please cite the following:

```
@ARTICLE{11372659,
  author={Salamat, Babak and Mattern, Dominik and Olzem, Sebastian-Sven and Elsbacher, Gerhard and Seidel, Christian and Tonello, Andrea M.},
  journal={IEEE Access}, 
  title={GMP3: Learning-Driven, Bellman-Guided Trajectory Planning for AAVs in Real-Time on SE(3)}, 
  year={2026},
  volume={14},
  number={},
  pages={22720-22733},
  doi={10.1109/ACCESS.2026.3661193}}
```
