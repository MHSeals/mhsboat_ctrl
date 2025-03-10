# mhsboat_ctrl

This is the control system for the MHSeals boat.  
This document reflects the current state of the codebase.

## Dependencies

- ROS 2 Humble
- Ubuntu 22.04
- Python >= 3.8
- numpy ~= 1.24.4
- [Custom messages](https://github.com/MHSeals/boat_interfaces)  
  (replace `ros_ws` with the path to your ROS 2 workspace)  
  `cd ~/ros_ws/src && git clone https://github.com/MHSeals/boat_interfaces`

## Building

You should build this package using symlink install so that you can easily modify the code and see the changes without having to rebuild the package. To build the package, run the following commands (substitute with the path to your ROS 2 workspace):

```bash
cd ~/ros_ws
colcon build --symlink-install --packages-select mhsboat_ctrl
```

## Running the Control System

[See commands_to_run_boat.md](./commands_to_run_boat.md)

## Packages Description

[See packages_description.md](./packages_description.md)

## Principles

1. **Boat-Centric Coordinates**:  
   All coordinates and movements are relative to the boat. The boat is always considered the origin (`0,0`) of the coordinate system. This simplifies calculations and ensures that all sensor data and task decisions are made with respect to the boat's current position and orientation.

2. **Modularity**:  
   The system is modular, allowing for easy addition and modification of tasks. Each task is implemented as a separate class that inherits from the base `Task` class in `mhsboat_ctrl/tasks/task.py`. This design allows new tasks to be added without affecting existing components.

3. **Simulation Support**:  
   The system supports running in simulation mode for testing and development without physical hardware. This is achieved by passing the `use_simulated_map` and `map_file` parameters to the `mhsboat_ctrl` node.

## Task System

Task files are stored in the `mhsboat_ctrl/tasks` directory. Each task is implemented as a separate class that extends the base `Task` class in `mhsboat_ctrl/tasks/task.py`. The base `Task` class defines a `run` method that is called by the main loop of the control system and should return a `TaskCompletionStatus` object. It also defines a `search` method that:

- Returns `None` if no task is found.
- Returns a tuple `(x, y)` when a task is detected.

Tasks are automatically detected and loaded by the control system. To add a new task, simply create a new file in the `mhsboat_ctrl/tasks` directory that defines a class inheriting from `Task`. At the bottom of the file, include a main function that accepts a `BoatController` object and calls `BoatController.add_task` with an instance of your new task.

## Main Loop

The main loop of the control system is found in `mhsboat_ctrl/mhsboat_ctrl.py`. It continuously searches for the next available task and then executes it. During execution, each task receives a `Sensors` object (from `mhsboat_ctrl/sensors.py`) that contains both sensor data and a generated map of the course. This design facilitates easy modifications and expansions as needed.

## Sensors

The `Sensors` class in `mhsboat_ctrl/sensors.py` is responsible for subscribing to various sensor topics and processing their data. This processed data is then used by tasks to make informed decisions throughout the control loop.

## Enums

The `TaskCompletionStatus` and `TaskStatus` enums are defined in `mhsboat_ctrl/enums.py` and are used to represent the state and results of the tasks.

## Simulation

To run the control system in simulation mode, pass the `use_simulated_map` and `map_file` parameters to the `mhsboat_ctrl` node. For example:

```bash
ros2 run mhsboat_ctrl mhsboat_ctrl --ros-args -p use_simulated_map:=true -p map_file:=src/mhsboat_ctrl/maps/taskone.yaml
```
