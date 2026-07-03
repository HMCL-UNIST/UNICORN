#include "detect.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <vector>
#include <limits>
#include <functional>

using std::placeholders::_1;

Obstacle::Obstacle(double x, double y, double size, double theta)
  : id(0), center_x(x), center_y(y), size(size), theta(theta)
{}

double Obstacle::squaredDist(const Obstacle &other)
{
  return (center_x - other.center_x) * (center_x - other.center_x) +
         (center_y - other.center_y) * (center_y - other.center_y);
}

Detect::Detect() : rclcpp::Node("detect"), car_s_(0),
  measuring_(false), from_bag_(false), path_needs_update_(false)
{
  // Load parameters from the ROS2 parameter server. Numeric params use
  // declareNumber(), which accepts either an int or a double from the yaml
  // (e.g. rate_detect: 40 and 40.0 both work) instead of hard-rejecting the
  // "wrong" scalar type.
  measuring_ = this->declare_parameter<bool>("measure", false);
  from_bag_ = this->declare_parameter<bool>("from_bag", false);
  rate_ = declareNumber("rate_detect", 10.0);
  min_size_n_ = static_cast<int>(declareNumber("min_size_n", 10));
  min_size_m_ = declareNumber("min_size_m", 0.2);
  max_size_m_ = declareNumber("max_size_m", 0.5);

  double lambda_deg = declareNumber("lambda_deg", 0.0);
  lambda_angle_ = lambda_deg * M_PI / 180.0;
  sigma_ = declareNumber("sigma", 0.0);
  min_2_points_dist_ = declareNumber("min_2_points_dist", 0.01);

  max_viewing_distance_ = declareNumber("max_viewing_distance", 9.0);
  boundaries_inflation_ = declareNumber("boundaries_inflation", 0.1);
  filter_kernel_size_ = static_cast<int>(declareNumber("filter_kernel_size", 1));
  new_cluster_threshold_m_ = declareNumber("new_cluster_threshold_m", 0.4);
  map_name_ = this->declare_parameter<std::string>("map", "default_map");

  // save-back: writes the detect: block to opponent_tracker_params.yaml when
  // save_params is set true (ROS1 dynamic_tracker_server role, detect side).
  this->declare_parameter<bool>("save_params", false);
  save_yaml_path_ = this->declare_parameter<std::string>(
      "save_yaml_path",
      ament_index_cpp::get_package_share_directory("stack_master") +
          "/config/opponent_tracker_params.yaml");

  // Load map for image filtering
  std::string packagePath = ament_index_cpp::get_package_share_directory("stack_master");
  std::string yamlPath = packagePath + "/maps/" + map_name_ + "/" + map_name_ + ".yaml";
  std::string image_path = packagePath + "/maps/" + map_name_ + "/" + map_name_ + ".png";

  GridFilter_.loadMapFromYAML(yamlPath, image_path);
  GridFilter_.setErosionKernelSize(filter_kernel_size_);

  // Publishers
  breakpoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detect/breakpoints_markers", 5);
  obstacles_msg_pub_ = this->create_publisher<f110_msgs::msg::ObstacleArray>("/detect/raw_obstacles", 5);
  obstacles_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detect/obstacles_markers_new", 5);

  if (measuring_) {
    latency_pub_ = this->create_publisher<std_msgs::msg::Float32>("/detect/latency", 5);
    on_track_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detect/on_track_points", 5);
  }

  // tf2 buffer + listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  global_wpnts_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
      "/global_waypoints_scaled", 10, std::bind(&Detect::pathCb, this, _1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&Detect::laserCb, this, _1));
  odom_frenet_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/car_state/odom_frenet", 10, std::bind(&Detect::carStateCb, this, _1));

  // Dynamic reconfigure replacement: live parameter callback (ROS2)
  if (!from_bag_) {
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Detect::dynParamCb, this, _1));
  }

  // Start the timer to run detection periodically
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&Detect::timerCallback, this));
}

Detect::~Detect() {}

void Detect::laserCb(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  scan_msgs = msg;
}

void Detect::pathCb(const f110_msgs::msg::WpntArray::ConstSharedPtr msg)
{
  waypoints_.clear();
  s_array_.clear();
  d_right_array_.clear();
  d_left_array_.clear();

  bool enable_wrapping = true;
  frenet_converter_.SetGlobalTrajectory(&(msg->wpnts), enable_wrapping);

  std::vector<geometry_msgs::msg::Point> points;

  for (const auto &wp : msg->wpnts) {
    std::vector<double> pt = {wp.x_m, wp.y_m};
    waypoints_.push_back(pt);
    s_array_.push_back(wp.s_m);
    d_right_array_.push_back(wp.d_right - boundaries_inflation_);
    d_left_array_.push_back(wp.d_left - boundaries_inflation_);

    geometry_msgs::msg::Point p;
    frenet_converter_.GetGlobalPoint(wp.s_m, -wp.d_right + boundaries_inflation_, &p.x, &p.y);
    p.z = 0;
    points.push_back(p);

    frenet_converter_.GetGlobalPoint(wp.s_m, wp.d_left - boundaries_inflation_, &p.x, &p.y);
    points.push_back(p);
  }

  if (!msg->wpnts.empty()) {
    track_length_ = msg->wpnts.back().s_m;
  }

  path_needs_update_ = false;
}

void Detect::carStateCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  (void)msg;
}

rcl_interfaces::msg::SetParametersResult Detect::dynParamCb(
    const std::vector<rclcpp::Parameter> &params)
{
  // read a numeric param as double whether rqt/param sent an int or a double
  auto asNum = [](const rclcpp::Parameter &p) -> double {
    return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
               ? static_cast<double>(p.as_int())
               : p.as_double();
  };
  for (const auto &param : params) {
    const std::string &name = param.get_name();
    if (name == "min_size_n") {
      min_size_n_ = static_cast<int>(asNum(param));
    } else if (name == "min_size_m") {
      min_size_m_ = asNum(param);
    } else if (name == "max_size_m") {
      max_size_m_ = asNum(param);
    } else if (name == "max_viewing_distance") {
      max_viewing_distance_ = asNum(param);
    } else if (name == "boundaries_inflation") {
      boundaries_inflation_ = asNum(param);
    } else if (name == "filter_kernel_size") {
      filter_kernel_size_ = static_cast<int>(asNum(param));
      GridFilter_.setErosionKernelSize(filter_kernel_size_);
    } else if (name == "new_cluster_threshold_m") {
      new_cluster_threshold_m_ = asNum(param);
    } else if (name == "lambda_deg") {
      lambda_angle_ = asNum(param) * M_PI / 180.0;
    } else if (name == "sigma") {
      sigma_ = asNum(param);
    }
  }

  // save-back on request: write the detect: block (keeps tracking: intact)
  for (const auto &param : params) {
    if (param.get_name() == "save_params" && param.as_bool()) {
      saveYaml();
    }
  }

  RCLCPP_INFO(this->get_logger(), "[Opponent Detection]: New dynamic reconfigure values received.");

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

double Detect::declareNumber(const std::string &name, double default_value)
{
  // dynamic typing lets the yaml provide an int (40) or a double (40.0)
  // without an InvalidParameterType abort; we coerce to double either way.
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  this->declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
  rclcpp::Parameter p = this->get_parameter(name);
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return p.as_double();
  }
  return default_value;
}

void Detect::saveYaml()
{
  if (save_yaml_path_.empty()) {
    RCLCPP_WARN(this->get_logger(), "[Opponent Detection]: no save_yaml_path; skipping save.");
    return;
  }
  try {
    YAML::Node root;
    std::ifstream fin(save_yaml_path_);
    if (fin.good()) {
      root = YAML::Load(fin);
    }
    fin.close();

    // Update only the detect: block; leave tracking: untouched. Numeric params
    // are loaded via declareNumber() (int-or-double tolerant), so a value that
    // happens to land on a whole number is fine to write out plainly.
    YAML::Node p;
    p["rate_detect"] = rate_;
    p["min_size_n"] = min_size_n_;
    p["min_size_m"] = min_size_m_;
    p["max_size_m"] = max_size_m_;
    p["lambda_deg"] = lambda_angle_ * 180.0 / M_PI;
    p["sigma"] = sigma_;
    p["min_2_points_dist"] = min_2_points_dist_;
    p["new_cluster_threshold_m"] = new_cluster_threshold_m_;
    p["max_viewing_distance"] = max_viewing_distance_;
    p["boundaries_inflation"] = boundaries_inflation_;
    p["filter_kernel_size"] = filter_kernel_size_;
    p["measure"] = measuring_;
    p["from_bag"] = from_bag_;
    p["save_params"] = false;
    root["detect"]["ros__parameters"] = p;

    std::ofstream fout(save_yaml_path_);
    fout << root;
    fout.close();
    RCLCPP_INFO(this->get_logger(), "[Opponent Detection]: detect params saved to: %s",
                save_yaml_path_.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[Opponent Detection]: failed to save yaml: %s", e.what());
  }
}

// --- Utility functions ---
double Detect::normalizeS(double x, double track_length)
{
  x = fmod(x, track_length);
  if (x > track_length / 2)
    x -= track_length;
  return x;
}

visualization_msgs::msg::MarkerArray Detect::clearmarkers()
{
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";  // set so the DELETEALL marker isn't dropped by RViz (empty frame)
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(marker);
  return ma;
}

std::vector<std::vector<std::pair<double, double>>> Detect::clustering(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {

  double l = lambda_angle_;
  double d_phi = msg->angle_increment;
  double sigma = sigma_;

  current_stamp_ = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("map", "laser", tf2::TimePointZero,
                                            tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "[Opponent Detection]: lookup Transform between map and laser not possible: %s", ex.what());
    std::vector<std::vector<std::pair<double, double>>> empty;
    return empty;
  }

  T_ = tf2::Vector3(transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z);
  quat_ = tf2::Quaternion(transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w);

  tf2::Transform tf_transform(quat_, T_);

  size_t n = msg->ranges.size();
  std::vector<Point2D> cloudPoints_list;
  cloudPoints_list.reserve(n);

  for (size_t i = 0; i < n; i++) {
    double angle = msg->angle_min + i * d_phi;
    double r = msg->ranges[i];
    // Coordinates in the laser frame (z is adjusted using T's z in the laser frame)
    double x_lf = r * cos(angle);
    double y_lf = r * sin(angle);
    double z_lf = -T_.z();
    tf2::Vector3 pt_lf(x_lf, y_lf, z_lf);
    tf2::Vector3 pt_map = tf_transform * pt_lf;
    cloudPoints_list.push_back(std::make_pair(pt_map.x(), pt_map.y()));
  }

  // --- Clustering: grouping points ---
  double div_const = sin(d_phi) / sin(l - d_phi);
  std::vector<std::vector<Point2D>> objects_pointcloud_list;
  std::vector<Point2D> on_track_pointcloud_list;

  // Lambda function to compute Euclidean distance
  auto euclidean_distance = [](const Point2D &a, const Point2D &b) -> double {
    double dx = a.first - b.first;
    double dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
  };

  for (size_t i = 0; i < n; i++) {
    Point2D curr_point = cloudPoints_list[i];
    if (GridFilter_.isPointInside(curr_point.first, curr_point.second)) {
      if (measuring_) on_track_pointcloud_list.push_back(curr_point);
      if (objects_pointcloud_list.empty()) {
        objects_pointcloud_list.push_back({curr_point});
        continue;
      }
      double curr_range = msg->ranges[i];
      double d_max = curr_range * div_const + 3 * sigma;
      double dist_to_next_point = euclidean_distance(cloudPoints_list[i],
                                                      objects_pointcloud_list.back().back());
      if (dist_to_next_point < d_max) {
        objects_pointcloud_list.back().push_back(curr_point);
      } else {
        if (objects_pointcloud_list.empty()) {
          objects_pointcloud_list.push_back({curr_point});
          continue;
        }
        double min_distance = std::numeric_limits<double>::max();
        size_t min_cluster_index = 0;
        for (size_t j = 0; j < objects_pointcloud_list.size(); j++) {
          double distance = euclidean_distance(curr_point, objects_pointcloud_list[j].back());
          if (distance < min_distance) {
            min_distance = distance;
            min_cluster_index = j;
          }
        }
        if (min_distance < new_cluster_threshold_m_) {
          // Move the cluster to the end of the list and then add the current point
          auto cluster_to_move = objects_pointcloud_list[min_cluster_index];
          objects_pointcloud_list.erase(objects_pointcloud_list.begin() + min_cluster_index);
          objects_pointcloud_list.push_back(cluster_to_move);
          objects_pointcloud_list.back().push_back(curr_point);
        } else {
          objects_pointcloud_list.push_back({curr_point});
        }
      }
    }
  }

  objects_pointcloud_list.erase(
      std::remove_if(objects_pointcloud_list.begin(), objects_pointcloud_list.end(),
                    [this](const std::vector<Point2D>& cluster) {
                        return cluster.size() < static_cast<size_t>(min_size_n_);
                    }),
      objects_pointcloud_list.end());

  if (measuring_) publishOnTrackPointCloud(on_track_pointcloud_list);

  return objects_pointcloud_list;

}

std::vector<Obstacle> Detect::fittingLShape(const std::vector<std::vector<std::pair<double, double>>> &objects_pointcloud_list) {
    std::vector<Obstacle> obstacles;
    const int numCandidates = 90;
    const double startAngle = 0.0;
    const double endAngle = M_PI/2 - M_PI/180;  // Final angle
    const double angleStep = (endAngle - startAngle) / (numCandidates - 1);

    // Precompute candidate angles and their corresponding cosine and sine values
    std::vector<double> candidateAngles(numCandidates);
    std::vector<double> candidateCos(numCandidates);
    std::vector<double> candidateSin(numCandidates);
    for (int j = 0; j < numCandidates; j++) {
        candidateAngles[j] = startAngle + j * angleStep;
        candidateCos[j] = std::cos(candidateAngles[j]);
        candidateSin[j] = std::sin(candidateAngles[j]);
    }

    // Minimum distance between two points (member variable)
    const double min_dist = min_2_points_dist_;

    // Perform L-shape fitting for each cluster (object)
    for (const auto &obstacle : objects_pointcloud_list) {
        if (obstacle.empty())
            continue;
        const int N = obstacle.size();

        // Store scores for each candidate angle (score: sum of 1/d for each point)
        std::vector<double> candidateScores(numCandidates, 0.0);

        // Iterate through each candidate angle
        for (int j = 0; j < numCandidates; j++) {
            const double cosVal = candidateCos[j];
            const double sinVal = candidateSin[j];

            // Store projections for each direction (length N)
            std::vector<double> proj1(N), proj2(N);
            for (int i = 0; i < N; i++) {
                double x = obstacle[i].first;
                double y = obstacle[i].second;
                // First direction: [cos, sin]
                proj1[i] = x * cosVal + y * sinVal;
                // Second direction: [-sin, cos]
                proj2[i] = -x * sinVal + y * cosVal;
            }
            // Find min and max for proj1
            double max1 = *std::max_element(proj1.begin(), proj1.end());
            double min1 = *std::min_element(proj1.begin(), proj1.end());
            // Find min and max for proj2
            double max2 = *std::max_element(proj2.begin(), proj2.end());
            double min2 = *std::min_element(proj2.begin(), proj2.end());

            // Compute D10 = -proj1 + max1, D11 = proj1 - min1
            std::vector<double> D10(N), D11(N), D1(N);
            for (int i = 0; i < N; i++) {
                D10[i] = -proj1[i] + max1;
                D11[i] = proj1[i] - min1;
            }
            // Compute norms of both vectors (Euclidean norm)
            double norm10 = 0.0, norm11 = 0.0;
            for (int i = 0; i < N; i++) {
                norm10 += D10[i] * D10[i];
                norm11 += D11[i] * D11[i];
            }
            norm10 = std::sqrt(norm10);
            norm11 = std::sqrt(norm11);
            // Select the direction with smaller norm
            for (int i = 0; i < N; i++) {
                D1[i] = (norm10 > norm11) ? D11[i] : D10[i];
            }

            // Same processing for proj2: D20 = -proj2 + max2, D21 = proj2 - min2
            std::vector<double> D20(N), D21(N), D2(N);
            for (int i = 0; i < N; i++) {
                D20[i] = -proj2[i] + max2;
                D21[i] = proj2[i] - min2;
            }
            double norm20 = 0.0, norm21 = 0.0;
            for (int i = 0; i < N; i++) {
                norm20 += D20[i] * D20[i];
                norm21 += D21[i] * D21[i];
            }
            norm20 = std::sqrt(norm20);
            norm21 = std::sqrt(norm21);
            for (int i = 0; i < N; i++) {
                D2[i] = (norm20 > norm21) ? D21[i] : D20[i];
            }

            // For each point, use D = min(D1, D2); clip small values and sum reciprocals as score
            double score = 0.0;
            for (int i = 0; i < N; i++) {
                double d_val = std::min(D1[i], D2[i]);
                if (d_val < min_dist)
                    d_val = min_dist;
                score += 1.0 / d_val;
            }
            candidateScores[j] = score;
        }  // end for each candidate angle

        // Find the index with the highest score and set the optimal angle theta_opt
        int bestIndex = std::distance(candidateScores.begin(), std::max_element(candidateScores.begin(), candidateScores.end()));
        double theta_opt = candidateAngles[bestIndex];

        // Recompute projections using the optimal angle
        std::vector<double> dist1(N), dist2(N);
        for (int i = 0; i < N; i++) {
            double x = obstacle[i].first;
            double y = obstacle[i].second;
            dist1[i] = x * std::cos(theta_opt) + y * std::sin(theta_opt);
            dist2[i] = -x * std::sin(theta_opt) + y * std::cos(theta_opt);
        }
        double max_dist1 = *std::max_element(dist1.begin(), dist1.end());
        double min_dist1 = *std::min_element(dist1.begin(), dist1.end());
        double max_dist2 = *std::max_element(dist2.begin(), dist2.end());
        double min_dist2 = *std::min_element(dist2.begin(), dist2.end());

        // Use vehicle position (T_ is a tf2::Vector3 member representing vehicle pose in the map frame)
        double cos_opt = std::cos(theta_opt);
        double sin_opt = std::sin(theta_opt);
        double x_rot = T_.x() * cos_opt + T_.y() * sin_opt;   // np.dot(self.T_[0:2], [cos, sin])
        double y_rot = -T_.x() * sin_opt + T_.y() * cos_opt;  // np.dot(self.T_[0:2], [-sin, cos])
        std::pair<double, double> my_pos(x_rot, y_rot);

        // Define four corners in the projected coordinate frame
        std::pair<double, double> corner_UR(max_dist1, max_dist2);
        std::pair<double, double> corner_LR(max_dist1, min_dist2);
        std::pair<double, double> corner_UL(min_dist1, max_dist2);
        std::pair<double, double> corner_LL(min_dist1, min_dist2);
        std::vector<std::pair<double, double>> corners = {corner_UR, corner_LR, corner_UL, corner_LL};

        // Choose the corner closest to the vehicle position
        int closest_index = 0;
        double minCornerDist = std::numeric_limits<double>::max();
        for (int k = 0; k < 4; k++) {
            double dx = corners[k].first - my_pos.first;
            double dy = corners[k].second - my_pos.second;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < minCornerDist) {
                minCornerDist = d;
                closest_index = k;
            }
        }
        std::pair<double, double> chosen_corner = corners[closest_index];

        // Determine obstacle size: use the larger of width or height, and clip to min_size_m_
        double width = max_dist1 - min_dist1;
        double height = max_dist2 - min_dist2;
        double rect_size = std::max(width, height);
        rect_size = std::max(rect_size, min_size_m_);

        // Estimate center coordinate based on selected corner (assume square cluster)
        std::pair<double, double> center;
        switch (closest_index) {
            case 0:
                center.first = chosen_corner.first - rect_size/2.0;
                center.second = chosen_corner.second - rect_size/2.0;
                break;
            case 1:
                center.first = chosen_corner.first - rect_size/2.0;
                center.second = chosen_corner.second + rect_size/2.0;
                break;
            case 2:
                center.first = chosen_corner.first + rect_size/2.0;
                center.second = chosen_corner.second - rect_size/2.0;
                break;
            case 3:
                center.first = chosen_corner.first + rect_size/2.0;
                center.second = chosen_corner.second + rect_size/2.0;
                break;
        }

        // Apply rotation correction to convert back from projected to original map coordinates
        double corrected_x = std::cos(theta_opt) * center.first - std::sin(theta_opt) * center.second;
        double corrected_y = std::sin(theta_opt) * center.first + std::cos(theta_opt) * center.second;

        obstacles.push_back(Obstacle(corrected_x, corrected_y, rect_size, theta_opt));
    }
    return obstacles;
}

void Detect::checkObstacles(std::vector<Obstacle> &current_obstacles)
{
  std::vector<Obstacle> filtered;
  int id = 0;
  for (size_t i = 0; i < current_obstacles.size(); i++) {
    if (current_obstacles[i].size <= max_size_m_) {
      current_obstacles[i].id = id;
      filtered.push_back(current_obstacles[i]);
      id++;
    }
  }
  tracked_obstacles_ = filtered;
}

void Detect::publishBreakpoints(const std::vector<std::vector<std::pair<double, double>>> &objects_pointcloud_list) {
  visualization_msgs::msg::MarkerArray markers_array;
  size_t num_objects = objects_pointcloud_list.size();

  for (size_t idx = 0; idx < num_objects; idx++) {
    const auto &obj = objects_pointcloud_list[idx];
    if (obj.empty()) {
      continue; // Skip empty clusters
    }

    // --- Marker for the first point ---
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_stamp_;  // Member variable of Detect class
    marker.id = idx * 10;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(3.0 / 40.0);  // auto-expire (~3 frames @ 40 Hz) so stale markers clear
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = static_cast<float>(idx) / num_objects;
    marker.pose.position.x = obj.front().first;
    marker.pose.position.y = obj.front().second;
    marker.pose.position.z = 0.0;  // On 2D plane
    marker.pose.orientation.w = 1.0;
    markers_array.markers.push_back(marker);

    // --- Marker for the last point ---
    visualization_msgs::msg::Marker marker2;
    marker2.header.frame_id = "map";
    marker2.header.stamp = current_stamp_;
    marker2.id = idx * 10 + 2;
    marker2.type = visualization_msgs::msg::Marker::SPHERE;
    marker2.action = visualization_msgs::msg::Marker::ADD;
    marker2.lifetime = rclcpp::Duration::from_seconds(3.0 / 40.0);  // auto-expire (~3 frames @ 40 Hz) so stale markers clear
    marker2.scale.x = 0.1;
    marker2.scale.y = 0.1;
    marker2.scale.z = 0.1;
    marker2.color.a = 0.5;
    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = static_cast<float>(idx) / num_objects;
    marker2.pose.position.x = obj.back().first;
    marker2.pose.position.y = obj.back().second;
    marker2.pose.position.z = 0.0;
    marker2.pose.orientation.w = 1.0;
    markers_array.markers.push_back(marker2);
  }

  // Publish a marker array to delete previous markers and then publish newly created markers
  breakpoints_markers_pub_->publish(clearmarkers());
  breakpoints_markers_pub_->publish(markers_array);
}

void Detect::publishObstaclesMessage()
{
  f110_msgs::msg::ObstacleArray obstacles_array_msg;
  obstacles_array_msg.header.stamp = current_stamp_;
  obstacles_array_msg.header.frame_id = "map";

  for (size_t i = 0; i < tracked_obstacles_.size(); i++) {
    double s,d;
    int idx_i;
    frenet_converter_.GetFrenetPoint(tracked_obstacles_[i].center_x, tracked_obstacles_[i].center_y, \
                                                                            &s, &d, &idx_i, true);

    f110_msgs::msg::Obstacle obsMsg;
    obsMsg.id = tracked_obstacles_[i].id;

    // wrap s_start/s_end into [0, track_length) across the seam; d is lateral
    double half = tracked_obstacles_[i].size / 2.0;
    obsMsg.s_start = std::fmod(std::fmod(s - half, track_length_) + track_length_, track_length_);
    obsMsg.s_end = std::fmod(std::fmod(s + half, track_length_) + track_length_, track_length_);
    obsMsg.d_left = d + half;
    obsMsg.d_right = d - half;
    obsMsg.s_center = s;
    obsMsg.d_center = d;
    obsMsg.size = tracked_obstacles_[i].size;
    obstacles_array_msg.obstacles.push_back(obsMsg);
  }
  obstacles_msg_pub_->publish(obstacles_array_msg);
}

void Detect::publishObstaclesMarkers()
{
  visualization_msgs::msg::MarkerArray markers_array;
  for (size_t i = 0; i < tracked_obstacles_.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_stamp_;
    marker.id = tracked_obstacles_[i].id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.lifetime = rclcpp::Duration::from_seconds(3.0 / 40.0);  // auto-expire (~3 frames @ 40 Hz) so stale markers clear
    marker.scale.x = tracked_obstacles_[i].size;
    marker.scale.y = tracked_obstacles_[i].size;
    marker.scale.z = tracked_obstacles_[i].size;
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.position.x = tracked_obstacles_[i].center_x;
    marker.pose.position.y = tracked_obstacles_[i].center_y;
    marker.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, tracked_obstacles_[i].theta);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    markers_array.markers.push_back(marker);
  }
  obstacles_marker_pub_->publish(clearmarkers());
  obstacles_marker_pub_->publish(markers_array);
}

void Detect::publishOnTrackPointCloud(const std::vector<Point2D> &on_track_points)
{
  sensor_msgs::msg::PointCloud2 pc_msg;
  pc_msg.header.stamp = this->get_clock()->now();
  pc_msg.header.frame_id = "map";

  // Set height to 1 and width to the number of points
  pc_msg.height = 1;
  pc_msg.width = on_track_points.size();

  // Set "xyz" fields and resize the message according to the number of points
  sensor_msgs::PointCloud2Modifier modifier(pc_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(on_track_points.size());

  // Use iterators to fill in the coordinates of each point (z is set to 0)
  sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

  for (const auto &pt : on_track_points) {
    *iter_x = pt.first;
    *iter_y = pt.second;
    *iter_z = 0.0f;
    ++iter_x; ++iter_y; ++iter_z;
  }

  on_track_points_pub_->publish(pc_msg);
}

// --- Timer callback ---
void Detect::timerCallback()
{
  // Guard: nothing to do until we have a scan and the global trajectory.
  // (ROS1 blocked on waypoints in the ctor; here we gate the periodic work.)
  if (!scan_msgs || waypoints_.empty()) {
    return;
  }

  double start_time = this->get_clock()->now().seconds();
  // Clustering
  std::vector<std::vector<std::pair<double, double>>> objects_pointcloud_list = clustering(scan_msgs);

  publishBreakpoints(objects_pointcloud_list);

  std::vector<Obstacle> current_obstacles = fittingLShape(objects_pointcloud_list);
  checkObstacles(current_obstacles);

  if (measuring_) {
    double end_time = this->get_clock()->now().seconds();
    std_msgs::msg::Float32 latency_msg;
    latency_msg.data = 1.0 / (end_time - start_time);
    latency_pub_->publish(latency_msg);
  }

  publishObstaclesMessage();
  publishObstaclesMarkers();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detect>());
  rclcpp::shutdown();
  return 0;
}
