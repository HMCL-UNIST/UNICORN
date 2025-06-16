#ifndef DETECT_H
#define DETECT_H

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <f110_msgs/WpntArray.h>
#include <f110_msgs/ObstacleArray.h>
#include <f110_msgs/Obstacle.h>
#include <dynamic_reconfigure/Config.h>


#include "frenet_conversion.h"
#include "grid_filter.h"

#include <vector>
#include <string>

class Obstacle
{
public:
  int id;
  double center_x;
  double center_y;
  double size;
  double theta;

  Obstacle(double x, double y, double size, double theta);
  double squaredDist(const Obstacle &other);
};

using Point2D = std::pair<double, double>;

class Detect
{
public:
  Detect();
  ~Detect();
  void run();

private:
  // Node handle and tf listener
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  tf::Vector3 T_;
  tf::Quaternion quat_;
  // Subscribers
  ros::Subscriber scan_sub_;
  ros::Subscriber global_wpnts_sub_;
  ros::Subscriber odom_frenet_sub_;
  ros::Subscriber dyn_param_sub_;

  // Publishers
  ros::Publisher breakpoints_markers_pub_;
  ros::Publisher boundaries_pub_;
  ros::Publisher obstacles_msg_pub_;
  ros::Publisher obstacles_marker_pub_;
  ros::Publisher latency_pub_;
  ros::Publisher on_track_points_pub_;

  sensor_msgs::LaserScan::ConstPtr scan_msgs;

  // Parameters (set via ROS params)
  double rate_;
  double lambda_angle_;  // in radians
  double sigma_;
  double new_cluster_threshold_m_;
  double min_size_m_;
  double min_2_points_dist_;
  int min_size_n_;
  int filter_kernel_size_;
  double max_size_m_;
  double max_viewing_distance_;
  double boundaries_inflation_;

  // Variables
  ros::Time current_stamp_;
  std::vector<Obstacle> tracked_obstacles_;
  std::vector<std::vector<double>> wpnts_data_; // For storing waypoint data (each row: id, s, d, x, y, d_right, d_left, psi, kappa, vx, ax)
  std::vector<std::vector<double>> waypoints_;   // Each element: {x, y}
  std::vector<double> s_array_;
  std::vector<double> d_right_array_;
  std::vector<double> d_left_array_;
  double track_length_;

  double car_s_;

  bool measuring_;
  bool from_bag_;
  bool path_needs_update_;
  std::string map_name_;
  frenet_conversion::FrenetConverter frenet_converter_;

  GridFilter GridFilter_;
  // Timer
  ros::Timer timer_;

  // Callbacks
  void laserCb(const sensor_msgs::LaserScan::ConstPtr &msg);
  void pathCb(const f110_msgs::WpntArray::ConstPtr &msg);
  void carStateCb(const nav_msgs::Odometry::ConstPtr &msg);
  void dynParamCb(const dynamic_reconfigure::Config::ConstPtr &msg);

  // Timer callback
  void timerCallback(const ros::TimerEvent &event);

  // Utility functions
  double normalizeS(double x, double track_length);
  bool laserPointOnTrack(double s, double d);
  void publishBreakpoints(const std::vector<std::vector<std::pair<double, double>>> &objects_pointcloud_list);

  visualization_msgs::MarkerArray clearmarkers();

  // Processing functions (clustering, obstacle fitting, etc.)
  std::vector<std::vector<std::pair<double, double>>> clustering(const sensor_msgs::LaserScan::ConstPtr &msg);
  std::vector<Obstacle> fittingLShape(const std::vector<std::vector<std::pair<double, double>>> &objects_pointcloud_list);
  void checkObstacles(std::vector<Obstacle> &current_obstacles);
  void publishObstaclesMessage();
  void publishObstaclesMarkers();
  void publishOnTrackPointCloud(const std::vector<Point2D> &on_track_points);

  // Converter initialization
  void initializeConverter();
};

#endif // DETECT_H
