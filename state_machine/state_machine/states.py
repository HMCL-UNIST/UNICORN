from __future__ import annotations

from typing import List, TYPE_CHECKING

from f110_msgs.msg import Wpnt

if TYPE_CHECKING:
    from state_machine.state_machine_node import StateMachine

"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""


def GlobalTracking(state_machine: "StateMachine") -> List[Wpnt]:
    s = int(state_machine.cur_s / state_machine.waypoints_dist + 0.5)
    return [state_machine.cur_gb_wpnts.list[(s + i) % state_machine.num_glb_wpnts]
            for i in range(state_machine.n_loc_wpnts)]


def Overtaking(state_machine: "StateMachine") -> List[Wpnt]:
    if state_machine.ot_planner == "spliner" or state_machine.ot_planner == "sqp" or state_machine.ot_planner == "lane_change":
        return state_machine.get_splini_wpts()
    else:
        s = state_machine.cur_id_ot
        return [
            state_machine.overtake_wpnts[(s + i) % state_machine.num_ot_points]
            for i in range(state_machine.n_loc_wpnts)
        ]


def RECOVERY(state_machine: "StateMachine"):
    return state_machine.get_recovery_wpts()


def START(state_machine: "StateMachine"):
    return state_machine.get_start_wpts()


def FTGOnly(state_machine: "StateMachine"):
    """No waypoints are generated in this follow the gap only state, all the
    control inputs are generated in the control node."""
    return []
