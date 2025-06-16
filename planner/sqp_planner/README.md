# Sequential Quadratic Programming based Planner
Sequential Quadratic Programming based Planner is a data-driven overtaking planner for autonomous racing, leveraging opponent trajectory prediction through Gaussian Process (GP) regression. This algorithm generates an overtaking trajectory by solving a sequential quadratic programming problem using the spatial prediction information of opponent vehicles.
The original reference algorithm is based on the [Predictive Spliner](https://github.com/ForzaETH/predictive-spliner) developed by the ForzaETH team.

## Key Features
- **Data-Driven Prediction Utilization:** Utilizes opponent trajectory predictions obtained from externally trained Gaussian Process (GP) models to ensure safe and strategic overtaking.
- **Experimental Validation:** Tested on a 1:10 scale autonomous racing platform with a robust comparison to state-of-the-art algorithms.
- **Optimization with Cost and Constraints:** Formulates and solves a constrained optimization problem using carefully designed cost functions and constraints for feasible and efficient overtaking paths.

## Dynamic Reconfigurable Parameters
There are several parameters that can be dynamically reconfigured. To do this, launch `rqt` and select `dynamic_sqp_tuner_node`.
A short overview of the reconfigurable parameters:

### SQP Planner
- `evasion_dist`: Orthogonal distance of the apex to the obstacle.
- `obs_traj_tresh`: Threshold of the obstacle towards raceline to be considered for evasion.
- `spline_bound_mindist`:  Splines may never be closer to the track bounds than this param in meters.
- `lookahead_dist`: Lookahead distance in meters.
- `avoidance_resolution`: Number of points used to generate avoidance path.
- `back_to_raceline_before`: Distance in meters before obstacle to stay on the raceline.
- `back_to_raceline_after`: Distance in meters after obstacle to go back on the raceline.
- `avoid_static_obs`: Avoid static obstacles.

## SQP Planner in simulation
![Image](https://github.com/user-attachments/assets/e6fe1826-0947-4bc1-94d8-2c6e114cb63a)