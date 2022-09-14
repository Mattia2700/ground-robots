# Navigation package for ground robots

Here there is all the code and the setup related to the navigation of the robot.

## Custom packages

- **ground_robots_models**: contains the models used both in simulation and with the real robot

- **ground_robots_navigation**: contains launch files, maps, navigation stack parameters, and rviz configurations to perform navigation related tasks

> **_Note:_** when using SLAM, you may use the ``teleop_twist_joy`` package to control and move the robot around with a bluetooth joystick. To be able to use bluetooth inside a container ``dbus`` must be started as the first process, and also you have to run the bluetooth daemon. To this end, you have to modify the ``Dockerfile`` entrypoint from ``["bash"]`` to ``sh docker_entrypoint.sh``

- **planning_bridge**: contains the interface between the planning and the ground robot navigation package

- **planning_bridge_msgs**: contains custom msg and services used by the planning bridge package

## Documentation

Documentation for this package can be found on ``docs`` folder, both in html and latex format, or more easily, you can just open ``documentation.html`` file (pointing to the index page of the documentation) in your browser.