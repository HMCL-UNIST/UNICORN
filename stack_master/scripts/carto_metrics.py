#!/usr/bin/env python3
"""Compact viewer for cartographer's /read_metrics service.

Extracts only the numbers that matter for frozen-map matching health:
  * pose-graph constraint counts (different vs same trajectory)
  * local/global constraint search attempts vs successes
  * score histogram of accepted constraints (min_score margin)
  * submap states and back-end work queue pressure

Usage (no build needed, run by path):
    python3 carto_metrics.py            # single snapshot
    python3 carto_metrics.py --watch 2  # refresh every 2 s
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import ReadMetrics


def to_dict(labels):
    return {l.key: l.value for l in labels}


def find(families, name):
    for f in families:
        if f.name == name:
            return f
    return None


def metric_value(family, want):
    """Return value of the metric whose labels contain all of `want`."""
    if family is None:
        return None
    for m in family.metrics:
        lab = to_dict(m.labels)
        if all(lab.get(k) == v for k, v in want.items()):
            return m.value
    return None


def histogram_line(family, want, lo_edge=0.0):
    """Render non-empty buckets of a histogram metric as 'lo-hi:count'."""
    if family is None:
        return "n/a"
    for m in family.metrics:
        lab = to_dict(m.labels)
        if all(lab.get(k) == v for k, v in want.items()):
            parts, prev = [], lo_edge
            total = 0
            for b in m.counts_by_bucket:
                if b.count > 0:
                    hi = "inf" if b.bucket_boundary == float("inf") \
                        else f"{b.bucket_boundary:.2f}"
                    parts.append(f"{prev:.2f}-{hi}:{int(b.count)}")
                    total += int(b.count)
                prev = b.bucket_boundary
            return f"total {total} | " + ("  ".join(parts) if parts else "-")
    return "n/a"


def snapshot(node, client):
    req = ReadMetrics.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    res = future.result()
    if res is None:
        return "ERROR: /read_metrics call timed out"
    if res.status.code != 0:
        return f"ERROR: {res.status.message}"
    fam = res.metric_families

    pg_con = find(fam, "mapping_2d_pose_graph_constraints")
    cb_con = find(fam, "mapping_constraints_constraint_builder_2d_constraints")
    cb_scores = find(fam, "mapping_constraints_constraint_builder_2d_scores")
    cb_queue = find(fam, "mapping_constraints_constraint_builder_2d_queue_length")
    submaps = find(fam, "mapping_2d_pose_graph_submaps")
    wq_size = find(fam, "mapping_2d_pose_graph_work_queue_size")
    wq_delay = find(fam, "mapping_2d_pose_graph_work_queue_delay")
    rt_ratio = find(fam, "mapping_2d_local_trajectory_builder_real_time_ratio")
    latency = find(fam, "mapping_2d_local_trajectory_builder_latency")

    diff = metric_value(pg_con, {"trajectory": "different"})
    same = metric_value(pg_con, {"trajectory": "same"})
    l_s = metric_value(cb_con, {"search_region": "local", "matcher": "searched"}) or 0
    l_f = metric_value(cb_con, {"search_region": "local", "matcher": "found"}) or 0
    g_s = metric_value(cb_con, {"search_region": "global", "matcher": "searched"}) or 0
    g_f = metric_value(cb_con, {"search_region": "global", "matcher": "found"}) or 0
    l_pct = f"{100.0 * l_f / l_s:.1f}%" if l_s else "-"

    lines = [
        time.strftime("[%H:%M:%S] cartographer matching health"),
        f"  constraints in graph : frozen-map(diff traj) {int(diff) if diff is not None else '?'}"
        f"   loop-closure(same traj) {int(same) if same is not None else '?'}",
        f"  local search         : found {int(l_f)} / searched {int(l_s)}  ({l_pct})",
        f"  global search        : found {int(g_f)} / searched {int(g_s)}"
        f"{'   <-- should be 0 (ratio 0.0)' if g_s else ''}",
        f"  accepted score dist  : "
        f"{histogram_line(cb_scores, {'search_region': 'local'})}",
        f"  submaps              : active "
        f"{metric_value(submaps, {'state': 'active'})}, frozen "
        f"{metric_value(submaps, {'state': 'frozen'})}",
        f"  matcher queue        : {metric_value(cb_queue, {})} pending",
        f"  backend queue        : size "
        f"{metric_value(wq_size, {})}, delay "
        f"{metric_value(wq_delay, {})} s",
        f"  frontend rt ratio    : {metric_value(rt_ratio, {})}"
        f"   (batch wall time {metric_value(latency, {})} s)",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", type=float, metavar="SEC", default=None,
                        help="refresh every SEC seconds (default: run once)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("carto_metrics_viewer")
    client = node.create_client(ReadMetrics, "/read_metrics")
    if not client.wait_for_service(timeout_sec=5.0):
        print("ERROR: /read_metrics not available "
              "(is cartographer_node running with -collect_metrics?)")
        sys.exit(1)

    try:
        while True:
            print(snapshot(node, client))
            if args.watch is None:
                break
            print()
            time.sleep(args.watch)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
