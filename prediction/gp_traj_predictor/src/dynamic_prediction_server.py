#!/usr/bin/env python3
"""ROS 2 parameter node retained as a migration aid.

Prediction tuning parameters now belong to ``opp_prediction`` itself, where
they are applied immediately.  This node exposes the former defaults so older
launch files have a ROS 2 replacement, but it does not use ROS 1
``dynamic_reconfigure``.
"""

import rclpy
from rclpy.node import Node


class DynamicPredictionTuner(Node):
    def __init__(self):
        super().__init__('dynamic_prediction_tuner_node')
        for name, default in {
            'n_time_steps': 200,
            'dt': 0.02,
            'save_distance_front': 6.0,
            'max_expire_counter': 10,
            'update_waypoints': True,
                'speed_offset': 0.0}.items():
            self.declare_parameter(name, default)
        self.get_logger().info(
            'Tune /opponent_propagation_predictor parameters directly; this node preserves legacy launch compatibility.')


if __name__ == '__main__':
    rclpy.init()
    node = DynamicPredictionTuner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
