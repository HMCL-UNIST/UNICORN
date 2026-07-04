from __future__ import annotations

from typing import List, TYPE_CHECKING

from rcl_interfaces.msg import (
    SetParametersResult,
    ParameterDescriptor,
    ParameterType,
    FloatingPointRange,
    IntegerRange,
)
from rclpy.parameter import Parameter

if TYPE_CHECKING:
    from state_machine.state_machine_node import StateMachine

"""
Replaces the ROS1 `dynamic_reconfigure` server (`dyn_statemachine_tuner.cfg`) and the
plain `rospy.get_param` reads in the ROS1 state machine node.

All tunable parameters are declared here with descriptors (replacing the .cfg ranges) and
live-updated through a single `add_on_set_parameters_callback` registered on the node.
"""


class StateMachineParams:
    # dynamic params that the node also mirrors as a same-named attribute
    _NODE_MIRRORED_PARAMS = {
        "lateral_width_gb_m",
        "lateral_width_ot_m",
        "splini_hyst_timer_sec",
        "emergency_break_horizon",
        "ftg_speed_mps",
        "ftg_timer_sec",
        "gb_ego_width_m",
        "gb_horizon_m",
        "interest_horizon_m",
        "overtaking_horizon_m",
        "overtake_min_closing_mps",
    }

    def __init__(self, node: "StateMachine") -> None:
        self.node = node

        # ------------------------------------------------------------------ #
        # NON-DYNAMIC / STRUCTURAL PARAMETERS (read once)
        # ------------------------------------------------------------------ #
        self._declare(
            "rate", 80,
            ParameterDescriptor(
                description="Rate at which the state machine runs in Hz",
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=1, to_value=200, step=1)],
            ),
        )
        self.rate_hz: int = node.get_parameter("rate").value

        self._declare(
            "n_loc_wpnts", 80,
            ParameterDescriptor(
                description="Number of local waypoints published",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )
        self.n_loc_wpnts: int = node.get_parameter("n_loc_wpnts").value

        self._declare("timetrials_only", False)
        self.timetrials_only: bool = node.get_parameter("timetrials_only").value

        self._declare("sim", True)
        self.sim: bool = node.get_parameter("sim").value

        self._declare("measure", False)
        self.measuring: bool = node.get_parameter("measure").value

        self._declare("racecar_version", "NUCX")
        self.racecar_version: str = node.get_parameter("racecar_version").value

        self._declare(
            "ot_planner", "lane_change",
            ParameterDescriptor(
                description="Overtaking planner: lane_change, sqp, spliner or graph_based",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.ot_planner: str = node.get_parameter("ot_planner").value

        self._declare("track_length", 1.0)
        self.track_length: float = node.get_parameter("track_length").value

        self._declare(
            "volt_threshold", 11.0,
            ParameterDescriptor(
                description="Voltage threshold below which the car is considered low bat",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )
        self.volt_threshold: float = node.get_parameter("volt_threshold").value

        self._declare(
            "gb_ego_width_m", 0.4,
            ParameterDescriptor(
                description="Distance from gb path for rejoining in meters",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )
        self.gb_ego_width_m: float = node.get_parameter("gb_ego_width_m").value

        self._declare(
            "gb_horizon_m", 15.0,
            ParameterDescriptor(
                description="Horizon considered on the global waypoints to check for obstacles [m]",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )
        self.gb_horizon_m: float = node.get_parameter("gb_horizon_m").value

        self._declare(
            "interest_horizon_m", 20.0,
            ParameterDescriptor(
                description="Horizon in which obstacles are considered of interest [m]",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )
        self.interest_horizon_m: float = node.get_parameter("interest_horizon_m").value

        self._declare(
            "overtaking_horizon_m", 6.9,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )
        self.overtaking_horizon_m: float = node.get_parameter("overtaking_horizon_m").value

        self._declare("emergency_break_horizon", 0.5)
        self.emergency_break_horizon: float = node.get_parameter("emergency_break_horizon").value

        # Lower bound on the ego-vs-opponent closing speed used to turn gap into a
        # time-to-pass (ttc) when slicing the opponent prediction in _check_free_frenet.
        # While trailing, real relative speed ~0 makes ttc blow past the prediction
        # horizon; this floor assumes "if we overtake, we close at least this fast" so
        # the sliced prediction window lands within the predicted horizon. Tune per gap.
        self._declare(
            "overtake_min_closing_mps", 2.5,
            ParameterDescriptor(
                description="Min assumed closing speed (ego-opp) for ttc when checking OT free [m/s]",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.5, to_value=5.0, step=0.1)],
            ),
        )
        self.overtake_min_closing_mps: float = node.get_parameter("overtake_min_closing_mps").value

        # ------------------------------------------------------------------ #
        # DYNAMIC PARAMETERS (replaces dyn_statemachine_tuner.cfg)
        # ------------------------------------------------------------------ #
        self._declare(
            "lateral_width_gb_m", 0.3,
            ParameterDescriptor(
                description="Threshold to raceline for GB_FREE in meters",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.75, step=0.05)],
            ),
        )
        self.lateral_width_gb_m: float = node.get_parameter("lateral_width_gb_m").value

        self._declare(
            "lateral_width_ot_m", 0.3,
            ParameterDescriptor(
                description="Threshold to raceline for O_FREE in meters",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.75, step=0.05)],
            ),
        )
        self.lateral_width_ot_m: float = node.get_parameter("lateral_width_ot_m").value

        self._declare(
            "splini_hyst_timer_sec", 0.7,
            ParameterDescriptor(
                description="Time between switching overtaking sides [s]",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.5, step=0.05)],
            ),
        )
        self.splini_hyst_timer_sec: float = node.get_parameter("splini_hyst_timer_sec").value

        self._declare(
            "splini_ttl", 2.0,
            ParameterDescriptor(
                description="Spliner ttl caching in seconds",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )

        self._declare(
            "pred_splini_ttl", 2.0,
            ParameterDescriptor(
                description="Predictive spliner ttl caching in seconds",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )
        # splini_ttl actually used depends on the planner (matches ROS1 logic)
        if self.ot_planner == "spliner":
            self.splini_ttl: float = node.get_parameter("splini_ttl").value
        else:
            self.splini_ttl = node.get_parameter("pred_splini_ttl").value

        self._declare(
            "overtaking_ttl_sec", 3.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )
        self.overtaking_ttl_sec: float = node.get_parameter("overtaking_ttl_sec").value

        self._declare(
            "ftg_speed_mps", 0.1,
            ParameterDescriptor(
                description="Speed threshold below which ftg counter is incremented [mps]",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.05)],
            ),
        )
        self.ftg_speed_mps: float = node.get_parameter("ftg_speed_mps").value

        self._declare(
            "ftg_timer_sec", 3.0,
            ParameterDescriptor(
                description="Time threshold at which ftg is triggered [s]",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.5, to_value=7.0, step=0.05)],
            ),
        )
        self.ftg_timer_sec: float = node.get_parameter("ftg_timer_sec").value

        self._declare("ftg_active", False)
        self.ftg_active: bool = node.get_parameter("ftg_active").value

        self._declare("force_GBTRACK", False)
        self.force_GBTRACK: bool = node.get_parameter("force_GBTRACK").value

        self._declare("use_force_trailing", False)
        self.use_force_trailing: bool = node.get_parameter("use_force_trailing").value

        # Momentary rqt buttons (ROS1: served by dynamic_statemachine_server). When set
        # true they trigger an action and reset to false (done in the node timer, not
        # here, so set_parameters() isn't called inside the on-set callback).
        self._declare("save_start_traj", False)
        self._declare("save_params", False)

    def _declare(self, name, default, descriptor=None):
        """Declare a parameter unless it has already been auto-declared from a yaml
        override (the node is created with
        automatically_declare_parameters_from_overrides=True). When it already exists
        we just (re)apply the descriptor so the GUI ranges still show up."""
        if self.node.has_parameter(name):
            if descriptor is not None:
                try:
                    self.node.set_descriptor(name, descriptor)
                except Exception:
                    pass
            return
        if descriptor is not None:
            self.node.declare_parameter(name, default, descriptor)
        else:
            self.node.declare_parameter(name, default)

    def parameters_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """Single callback that mirrors the ROS1 dyn_param_cb live update behaviour."""
        for param in parameters:
            name = param.name
            value = param.value

            if name == "rate":
                self.rate_hz = value
                if self.node.main_loop is not None:
                    self.node.main_loop.timer_period_ns = int(1e9 / value)
            elif name == "splini_ttl":
                if self.node.ot_planner == "spliner":
                    self.splini_ttl = value
                    self.node.splini_ttl_counter = int(self.splini_ttl * self.rate_hz)
            elif name == "pred_splini_ttl":
                if self.node.ot_planner != "spliner":
                    self.splini_ttl = value
                    self.node.splini_ttl_counter = int(self.splini_ttl * self.rate_hz)
            elif name == "overtaking_ttl_sec":
                self.overtaking_ttl_sec = value
                self.node.overtaking_ttl_count_threshold = int(value * self.rate_hz)
            elif name == "ftg_active":
                self.ftg_active = value
                self.node.ftg_disabled = not value
            elif name == "save_start_traj":
                # momentary: act + reset in the node timer (not inside this on-set cb)
                if value:
                    self.node._save_start_traj_requested = True
            elif name == "save_params":
                if value:
                    self.node._save_params_requested = True
            else:
                # generic live update for the remaining tunables
                setattr(self, name, value)

            # The node mirrors several of these dynamic params as its own attributes
            # (the _check_* conditions read them directly off the node, matching the
            # ROS1 dyn_param_cb live-update behaviour). Keep them in sync.
            if name in self._NODE_MIRRORED_PARAMS and hasattr(self.node, name):
                setattr(self.node, name, value)
            if name == "force_GBTRACK":
                self.node.force_gbtrack_state = value
            if name == "use_force_trailing":
                self.node.use_force_trailing = value

            self.node.get_logger().info(f"Parameter '{name}' was set to {value}")

        return SetParametersResult(successful=True)
