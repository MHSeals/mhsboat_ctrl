# Package Descriptions

## mhsboat_ctrl

**Purpose:** This package contains the main control logic for the MHSeals autonomous boat. It is responsible for task management, decision-making, and interfacing with the boat's actuators.

**Key Components:**

* **`mhsboat_ctrl.py`:** The main control loop that orchestrates task execution. It subscribes to sensor data, manages the task queue, and publishes commands to the boat.
* **`task.py`:** Defines the base `Task` class, which all task implementations inherit from.
* **`tasks/`:** Directory containing individual task implementations (e.g., navigation, obstacle avoidance).

**ROS Topics Subscribed:**

* `/mhsboat_ctrl/map`: Receives the map of the environment from the `sensors` node.
* `/mavros/setpoint_velocity/cmd_vel`: Publishes velocity commands to the boat.

## sensors

**Purpose:** This package is responsible for collecting, processing, and fusing data from various sensors (camera, LiDAR, odometry) to create a unified representation of the boat's environment.

**Key Components:**

* **`sensors.py`:** The main sensor processing node that subscribes to sensor topics, performs data fusion, and publishes the processed map.
* **`utils/lidar.py`:** Utility functions for processing LiDAR point cloud data.

**ROS Topics Subscribed:**

* `/AiOutput`: Receives object detection data from the camera.
* `/center_of_clusters`: Receives processed LiDAR data.
* `/odometry/filtered`: Receives odometry data.

**ROS Topics Published:**

* `/mhsboat_ctrl/map`: Publishes the processed map of the environment.
* `/mhsboat_ctrl/buoy_clusters`: Publishes the buoy cluster data.

## simulated_map

**Purpose:** This package provides a simulated environment for testing the control system without physical hardware. It allows for defining a virtual course with buoys and obstacles.

**Key Components:**

* **`gui.py`:** Implements a graphical user interface (GUI) for visualizing the simulated environment and the boat's position.

## display_map

**Purpose:** This package is used for visualizing the map data received by the boat. It provides a graphical representation of the detected buoys and other course objects.

**Key Components:**

* **`testgui.py`:** Implements a GUI for displaying the map data.

## buoy_recognition

**Purpose:** This package is responsible for processing camera images to detect and classify buoys and other objects in the boat's environment.

**Parameters:**

* **`headless_mode` (default: `True`):** Controls whether the camera image processing displays images on the screen. Set to `False` to enable image display.

**Key Components:**

* **`buoy_recognition.py`:** The main buoy recognition node that subscribes to camera images, performs object detection using a YOLO model, and publishes the results.

**ROS Topics Subscribed:**

* `/image_raw`: Receives raw camera images.

**ROS Topics Published:**

* `/AiOutput`: Publishes the detected object data.

## center_of_clusters

**Purpose:** This package processes LiDAR point cloud data to identify clusters of points that likely correspond to buoys or other objects of interest.

**Key Components:**

* **`center_of_clusters.py`:** The main clustering node that subscribes to LiDAR point cloud data, performs clustering, and publishes the center points of the clusters.

**ROS Topics Subscribed:**

* `/velodyne_points`: Receives raw LiDAR point cloud data.

**ROS Topics Published:**

* `/center_of_clusters`: Publishes the processed LiDAR data.

## bag_recorder

**Purpose:** This package provides a utility for recording ROS 2 bag files, which can be used for data logging, analysis, and playback.

**Key Components:**

* **`bag_recorder.py`:** The main bag recording node that subscribes to specified topics and writes the data to a bag file.

**ROS Topics Subscribed:**

* `/mhsboat_ctrl/map`: Subscribes to the map topic to record the map data.
