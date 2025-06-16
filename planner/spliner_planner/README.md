# Piecewise Polynomial based Planner
The Piecewise Polynomial-based Planner is a lightweight overtaking planner for autonomous racing, generating trajectories purely based on piecewise polynomial representations. Although opponent predictions are not yet integrated, the piecewise polynomial framework allows for easy extension with externally trained Gaussian Process (GP) models in the future work.

## Key Features
- **Pure Piecewise Polynomial Trajectories:** Generates overtaking paths directly from piecewise polynomial fitting, ensuring smoothness.
- **Efficient and Lightweight:** Suitable for real-time planning due to its computational simplicity and direct trajectory construction.

## Dynamic Reconfigurable Parameters
There are several parameters that can be dynamically reconfigured. To do this, launch `rqt` and select `dynamic_avoidance_tuner_node`.
A short overview of the reconfigurable parameters:

### Piecewise Polynomial Planner
- `evasion_dist`: Orthogonal distance of the apex to the obstacle.
- `obs_traj_tresh`: Threshold of the obstacle towards raceline to be considered for evasion.
- `spline_bound_mindist`: Splines may never be closer to the track bounds than this param in meters.
- `lookahead_dist`: Lookahead distance in meters.
- `back_to_raceline_after`: Distance in meters after obstacle to go back on the raceline.

## Piecewise Polynomial based Planner in simulation
![Image](https://github.com/user-attachments/assets/2e2f7224-0986-46dc-851f-88f558883e00)