# Boat controller for Roboboat
## Installation

1. Clone into src directory
```shell
cd /root/ros_ws/src
git clone https://github.com/Wavefire5201/boat_ctrl
```
2. Build package
```shell
cd ..
colcon build --symlink-install --packages-select boat_ctrl
```
3. Source setup files
```shell
source /root/ros_ws/install/setup.bash
```
4. Run package
```shell
ros2 run boat_ctrl <entry_point>
```
> Note: to run buoy and taskone, you need to install ultralytics with pip
> `pip install ultralytics`

## Entry Points
### mavros
- boat_arm - arm the boat
    - Usage: `ros2 run boat_ctrl boat_arm <True | False>`
- boat_mode - set mode of boat to GUIDED, MANUAL, HOLD, or AUTO
- boat_camera - camera node, detects poles & publishes to /taskone/poles
- boat_mavros_controller - control the boat from terminal
- boat_taskone - taskone but vision only
- boat_taskone_waypoint - taskone but vision + lidar (waypoint)
### jonathan
- average_buoy_location - averages buoy locations across multiple readings to minimize error
- locate_buoys - combines camera and lidar data to get GPS coordinates of buoys
- center_of_clusters - Clusters buoys 
- buoy_recognition - Analyzes images to detect buoys
- jon_taskone - midpoint of red and green poles
- jon_tasktwo - in progress