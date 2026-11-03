Installation
============

We recommend installing DroneManager in its own virtual environment: https://docs.python.org/3/library/venv.html

This package is available on PyPI::

    pip install DroneManager

If you plan to develop with DroneManager we recommend an editable install from the repository:

1. Clone this repository
2. Move into the root directory of the repository
3. Install with pip::

    pip install --upgrade pip
    pip install -e .

After installation, DroneManager can be started by typing "dm" in a terminal with the venv active. The first start-up
can take a few moments, and then you should have a blank instance of DroneManager to work with!

.. image:: imgs/tool-blank.png


Configuration and log files are placed in a folder in the users Documents directory by default. The default config
file is copied over form the repository. If an update changes the config structure, you should rename your old config,
boot up DM and then migrate any changes you had made over to the newly created config.

Simulated Drones
----------------

It is often more convenient to work with simulated drones, rather than real ones. DroneManager should work with any
simulation that also supports MAVLink. We use the PX4 Gazebo SITL, which can be installed on Ubuntu or in Windows WSL
following the `instructions here`_.

.. _instructions here: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu#simulation-and-nuttx-pixhawk-targets

For a WSL install, we recommend mirrored networking, with the Windows firewall configured to allow traffic from WSL.
Connecting between Gazebo and DroneManager then works as if both were running on the same OS.

PX4 SITL also supports multi drones in one environment, please refer to `their documentation`_ for details.

.. _their documentation: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
