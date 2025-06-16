-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "mapping.lua"
-- options.pose_publish_period_sec = 0.1

MAP_BUILDER.num_background_threads = 6 -- Multi-threading으로 성능 최적화

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 2 --매핑에 비해 낮춰줘야함

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5 --0.01
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1 --25
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 20.0 -- 10.0
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1. -- 선형 검색 범위 (m)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.) -- 각도 검색 범위 (deg)

POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.7
POSE_GRAPH.global_sampling_ratio = 0.000 -- 0.003 -- 전역 매칭 샘플링 비율
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80
POSE_GRAPH.global_constraint_search_after_n_seconds = 1000. -- 전역 제약 조건 검색 주기

POSE_GRAPH.constraint_builder.max_constraint_distance = 5.


return options