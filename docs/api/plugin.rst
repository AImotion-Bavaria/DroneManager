Plugins
=======

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none


Plugins exist to extend the functionality of DroneManager in a straightforward way. They define a list of commands that
they make available for the user interface, a list of background functions that should run continuously while the plugin
is running. They can also have dependencies on other plugins, which are then loaded automatically when the plugin is
loaded.

They are implemented as special classes in modules. Each module in the ``plugins`` folder is inspected for potential
classes. There can one plugin per module. The name of the module defines the name of the plugin. For the command-line,
they additionally provide a prefix, which is prepended to the commands to prevent collisions, i.e. multiple plugins can
have a ``connect`` command. For plugin ``abc``, this command becomes ``abc-connect``.

There are two special types of plugins: :doc:`Missions <mission>` and :doc:`Sensors <sensor>`.


Plugin Base Class
-----------------

.. automodule:: dronemanager.plugin
   :members:
   :undoc-members:
   :show-inheritance:


Plugin list
-----------

Camera
^^^^^^

.. automodule:: dronemanager.plugins.camera
   :members:
   :undoc-members:
   :show-inheritance:


Controllers
^^^^^^^^^^^

.. automodule:: dronemanager.plugins.controllers
   :members:
   :undoc-members:
   :show-inheritance:


External
^^^^^^^^

.. automodule:: dronemanager.plugins.external
   :members:
   :undoc-members:
   :show-inheritance:


Gimbal
^^^^^^

.. automodule:: dronemanager.plugins.gimbal
   :members:
   :undoc-members:
   :show-inheritance:


Mission
^^^^^^^

Please see :doc:`the mission documentation page <mission>`.


Optitrack
^^^^^^^^^

.. automodule:: dronemanager.plugins.optitrack
   :members:
   :undoc-members:
   :show-inheritance:


Scripts
^^^^^^^

.. automodule:: dronemanager.plugins.scripts
   :members:
   :undoc-members:
   :show-inheritance:


Sensor
^^^^^^

Please see :doc:`the sensor documentation page <sensor>`.


Stream
^^^^^^

.. automodule:: dronemanager.plugins.stream
   :members:
   :undoc-members:
   :show-inheritance:
