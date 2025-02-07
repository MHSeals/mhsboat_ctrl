# jetson
## realsense
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1920x1080x30 enable_color:=true camera_namespace:=/camera camera_name:=camera

## velodyne
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

# odroid
## mavros
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 