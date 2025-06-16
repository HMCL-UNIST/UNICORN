# Gaussian Process Regression Opponent Trajectory Prediction
This system extends the Planner & Sate Machnine for autonomous racing by integrating Gaussian Process Regression (GPR) to predict opponent vehicle trajectories. By collecting the opponent's state data over a full lap, the system builds a predictive model to anticipate future movements, enabling more strategic and collision-aware planning. The original reference algorithm is based on the [Predictive Spliner](https://github.com/ForzaETH/predictive-spliner) developed by the ForzaETH team.

## Key Features
- **Opponent Trajectory Prediction with GPR:** Collects opponent state data (position, velocity) throughout a full lap and uses GPR to predict the opponent's trajectory over a future time horizon.
- **Planner & State Machine Integration:** The predicted trajectory is incorporated into planner and state machnine to generate racing lines that strategically account for the opponent's future positions.
- **Uncertainty-Aware:** GPR provides both mean predictions and uncertainty (variance), allowing for risk-sensitive planning (e.g., expanding safety margins).

## Dynamic Reconfigurable Parameters
There are several parameters that can be dynamically reconfigured. To do this, launch `rqt` and select `dynamic_prediction_tuner_node`.
A short overview of the reconfigurable parameters:

### GPR Opponent Trajectory Prediction
- `n_time_steps`: Number of time steps for prediction.
- `dt`: Time step for prediction.
- `save_distance_front`:  Length of car in the front plus margin for enable prediction.
- `max_expire_counter`: Maximum n of iterations until prediction info gets deleted.
- `update_waypoints`: Update waypoints.
- `speed_offset`: Add speed offset.

## Opponent's Behavior
![Image](https://github.com/user-attachments/assets/ad62d8eb-64d0-4da2-a326-259f9e020f62)

## Opponent's Trajectory & Behavior
![Image](https://github.com/user-attachments/assets/243568a3-2f0b-4af4-bb4a-23e8d4c78732)