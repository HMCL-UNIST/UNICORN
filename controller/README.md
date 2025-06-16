# Controller
This controller package implements the following controllers, with more details in the respective READMEs:
- [Combined controller](./combined/README.md)
- [Follow the Gap controller](./ftg/README.md)

The `control_node` implemented in `controller_manager.py` initializes the needed controllers. It runs at a specified loop rate where in each cycle the next control inputs are calculated via the choosen controller.

## Input/Output Topic Signature
This nodes subscribes to:
- `/behavior_strategy`:  Receives local waypoints, opponent data, and current behavior state; updates trajectory, opponent info, and replanning flags.
- `/car_state/odom`: Reads the car's odom state
- `/car_state/pose`: Reads the car's position state
- `/car_state/odom_frenet`: Reads the car's state
- `/imu/data`: Reads the IMU measurements.
- `/dyn_controller/parameter_updates`: Dynamically updates controller parameters.
- `/scan`: Reads the LiDAR scans.
- `/save_start_traj`: Receives params for START state

The node publishes to:
- `/vesc/high_level/ackermann_cmd_mux/input/nav_1`: Publishes the control commands.
- `/lookahead_point`: Publishes lookahead point marker.
- `/trailing_opponent_marker`: Published trailing opponend marker.
- `/future_position`: Publishes future predicted position.
- `/l1_distance`: Publishes the L1 distance.
- `/trailing/gap_data`: Publishes the PID data for trailing.
- `/controller/latency`: Publishes the latency of the controller.

