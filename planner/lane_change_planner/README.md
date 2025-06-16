# Lane Change based Planner
The Lane Change-based Planner is a geometry-driven overtaking planner for autonomous racing. It generates overtaking trajectories by constructing inner and outer lanes relative to the track centerline and selecting appropriate lane changes based on the geometric positions of opponent vehicles. This method allows for fast and interpretable overtaking decisions.

## Key Features
- **Centerline-Based Lane Generation:** Constructs inner and outer lanes relative to the track’s centerline for potential overtaking paths.
- **Geometry-Driven Decision Making:** Determines overtaking maneuvers based on the spatial configuration of opponent vehicles.
- **Efficient and Interpretable:** Provides a lightweight and intuitive decision-making process.

## Dynamic Reconfigurable Parameters
There are several parameters that can be dynamically reconfigured. To do this, launch `rqt` and select `dynamic_change_tuner_node`.
A short overview of the reconfigurable parameters:

### Lane Change based Planner
-  `evasion_dist`: Orthogonal distance of the apex to the obstacle.
-  `obs_traj_tresh`: Threshold of the obstacle towards raceline to be considered for evasion.
-  `spline_bound_mindist`: Splines may never be closer to the track bounds than this param in meters.

## Lane Change based Planner in simulation
![Image](https://github.com/user-attachments/assets/f6ac24be-ad7e-44e5-8938-95279ffbcb01)