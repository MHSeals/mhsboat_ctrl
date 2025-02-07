# mhsboat_ctrl

This is the control system for the MHSeals boat.

## Dependencies
- ROS 2 Humble
- Ubuntu 22.04
- Python >= 3.8
- numpy ~= 1.24.4
- [Custom messages](https://github.com/MHSeals/mhsboat_ctrl/tree/boat_interfaces)
    - (replace `ros_ws` with the path to your ROS 2 workspace)
    - `cd ~/ros_ws/src && git clone https://github.com/MHSeals/mhsboat_ctrl/ -b boat_interfaces boat_interfaces`

## Building
You should build this package using symlink install so that you can easily modify the code and see the changes without having to rebuild the package. To build the package, run the following commands, substituting with the path to your ROS 2 workspace:
```bash
cd ~/ros_ws
colcon build --symlink-install --packages-select mhsboat_ctrl
```

## Principles
1. **Boat-Centric Coordinates**: All coordinates and movements are relative to the boat. The boat is always considered the origin (0,0) of the coordinate system. This simplifies calculations and ensures that all sensor data and task decisions are made with respect to the boat's current position and orientation.

2. **Modularity**: The system is modular, allowing for easy addition and modification of tasks. Each task is a separate class that inherits from the Task class in mhsboat_ctrl/tasks/task.py. This modularity ensures that new tasks can be added without affecting the existing system.

3. **Simulation Support**: The system supports running in simulation mode, allowing for testing and development without the need for physical hardware. This is achieved by passing the use_simulated_map and map_file parameters to the mhsboat_ctrl node.

## Task System

Task files live in the `mhsboat_ctrl/tasks` directory. Each task is a separate class that inherits from the `Task` class in [`mhsboat_ctrl/tasks/task.py`](mhsboat_ctrl/tasks/task.py). The `Task` class has a `run` method that is called by the main loop of the control system. The `run` method should return a `TaskCompletionStatus` object indicating the status of the task. The `Task` class also has a `search` method which searches for the next task to run. The `search` method should return either `None` if it isn't found, or a tuple `(x, y)` where `x` and `y` are the coordinates of where the task was found.

Tasks are automatically detected and loaded by the control system. To add a new task, simply create a new file in the `mhsboat_ctrl/tasks` directory that defines a new class that inherits from the `Task` class. Then, add a main function at the bottom of the file that takes in a `BoatController` object and calls `BoatController.add_task` with an instance of the new task class.

## Main Loop
The main loop of the control system is in `mhsboat_ctrl/mhsboat_ctrl.py`. It is a simple loop that searches for the next task to run, runs it, and then repeats. The tasks take in a `Sensors` object which contains all the sensor data and our generated map of the course. The tasks can use this data to make decisions about what to do next. This architecture allows for easy addition of new tasks and easy modification of existing tasks compared to our approach in the 2023/2024 control system.

## Sensors
The `Sensors` class in mhsboat_ctrl/sensors.py is responsible for subscribing to various sensor topics and processing the incoming data. The sensor data is then used by the tasks to make decisions.

## Enums
The `TaskCompletionStatus` and `TaskStatus` enums in `mhsboat_ctrl/enums.py` are used to represent the status of tasks.

## Simulation
To run the control system in simulation mode, pass the `use_simulated_map` and `map_file` parameters to the mhsboat_ctrl node. For example:
```bash
ros2 run mhsboat_ctrl mhsboat_ctrl --ros-args -p use_simulated_map:=true -p map_file:=src/mhsboat_ctrl/maps/taskone.yaml
```