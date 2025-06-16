# Lidar Detection and Tracking System

A ROS-based system for detecting and tracking obstacles using 2D lidar data.

## Overview

This package provides a simple yet effective system for:
1. Detecting obstacles from 2D lidar scans
2. Tracking detected obstacles over time 
3. Classifying obstacles as static or dynamic
4. Visualizing the results in RViz

The system uses clustering techniques to group lidar points into potential obstacles, tracks them between frames using a simple nearest-neighbor approach, and classifies them as static or dynamic based on their observed velocity.

## Requirements

- ROS (tested on ROS Noetic)
- Python 3
- NumPy
- tf

## Installation

1. Clone this repository into your catkin workspace:
```
cd ~/catkin_ws/src
git clone <repository-url>
```

2. Build your workspace:
```
cd ~/catkin_ws
catkin_make
```

3. Source your workspace:
```
source ~/catkin_ws/devel/setup.bash
```

## Usage

Launch the detection and tracking system:

```
roslaunch stack_master lidar_detection_tracking.launch
```

This will start:
1. The lidar detection and tracking node
2. RViz with the appropriate configuration (if `use_rviz:=true`)

### Parameters

You can adjust the following parameters in the launch file:

- `rate`: The update rate of the node in Hz (default: 10)
- `detection_threshold`: Minimum distance between points for clustering (default: 0.2 meters)
- `min_cluster_size`: Minimum number of points to consider a cluster as an obstacle (default: 3)
- `max_cluster_distance`: Maximum distance between points in a cluster (default: 0.3 meters)
- `tracking_distance_threshold`: Maximum distance for considering a detected obstacle as the same as a previously tracked one (default: 0.5 meters)
- `max_tracking_distance`: Maximum tracking distance (default: 10.0 meters)

### Visualization

The system publishes visualization markers to the following topics:

- `/detection/obstacles_markers`: Markers for detected obstacles (cylinders)
- `/detection/cluster_points`: Markers for the raw lidar points in each detected cluster
- `/detection/velocity_markers`: Arrow markers indicating obstacle velocity vectors

In RViz, static obstacles are shown in blue, while dynamic obstacles are shown in red. Trajectory trails are shown in green, and velocity vectors are shown in yellow.

## Files

- `lidar_detection_tracking.py`: The main detection and tracking node
- `lidar_detection_tracking.launch`: Launch file for the system
- `rviz/lidar_detection.rviz`: RViz configuration file for visualization

## How It Works

1. **Detection**: The system clusters lidar points based on proximity to detect potential obstacles.
2. **Tracking**: Detected obstacles are matched to previously tracked obstacles based on position proximity.
3. **Classification**: Obstacles are classified as static or dynamic based on their calculated velocity.
4. **Visualization**: Different marker types and colors are used to visualize the results in RViz.

## License

This project is licensed under the MIT License - see the LICENSE file for details. 