# Commands to Run the Boat

This document describes the steps needed to launch various components of the boat’s control system. Follow the instructions for your target platform (Jetson or Odroid) and refer to the troubleshooting section if you run into issues.

## For Jetson

Before running any commands, **ensure you have entered the docker container**:

```bash
sh ~/Scripts/run_container.sh
```

### 1. Velodyne LiDAR

Launch all nodes for the Velodyne sensor:
  
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```

*Tip:* Use this command to initialize and monitor your LiDAR sensor. Check that all sensor topics are active using `ros2 topic list`.

### 2. Odom Stack

Launch the odometry processing system which handles localization data:

```bash
ros2 launch mhsboat_ctrl odometry.launch.py
```

*Note:* The odometry stack configuration is defined in odometry.launch.py if you need to adjust parameters like sensor calibration.

### 3. Mapping

Initialize the mapping system that builds the course map:

```bash
ros2 launch mhsboat_ctrl mapping.launch.py
```

*Details:* The mapping launch file (launch/mapping.launch.py) sets up nodes to process sensor data and generate maps. This is especially useful for simulation and diagnostic purposes.

### 4. Controller

Start the main control system which processes tasks and sensor input:

```bash
ros2 run mhsboat_ctrl mhsboat_ctrl
```

This command calls the entry point defined in setup.py which in turn runs `mhsboat_ctrl.mhsboat_ctrl:main`.

## For Odroid

*Note:* There is no docker container setup for Odroid, so run the commands directly.

### 1. MAVROS

Launch the MAVROS node which handles communication with flight controllers:
  
```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

## Common Troubleshooting

- **Node/Package Not Found:**  
  If you encounter a `package not found` error, ensure that the package is correctly built and that you sourced the install/setup.bash file.

- **Sensor Data Missing:**
  - No Camera Data/Camera Data Lagging Behind: Check the camera connection and make sure it is plugged into the Jetson.
  - No LiDAR Data/LiDAR Data Lagging Behind: Ensure the Velodyne sensor is powered on and connected to the Jetson. Make sure the IP in the web interface matches the IP of the Jetson.
  - No Odometry Data/Odometry Data Lagging Behind: Check the connection between the Cube Orange and the Odroid. Ensure the USB port is correct and that the device is powered on. Make sure that MAVROS is running and connected to the flight controller.

- **No frames recieved in the last second:**
  - Check the connection between the realsense and the Jetson. Make sure the realsense-ros node (launched with odom stack/odometry.launch.py) is running and hasn't crashed.
