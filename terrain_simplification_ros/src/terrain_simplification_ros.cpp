#include "terrain_simplification_ros/terrain_simplification_ros.hpp"

namespace terrain_simplification_ros {

TerrainSimplificationRos::TerrainSimplificationRos(
    bool& success,
    ros::NodeHandle& nh)
  : ros_nh_(nh),
    TerrainSimplification() {

  // Read ros parameters
  if (!readParameters()) {
    success = false;
    return;
  }
  // Setup filter chain
  filter_chain_ = std::make_shared<filters::FilterChain<grid_map::GridMap> >("grid_map::GridMap");
  if (!filter_chain_->configure(filter_chain_parameters_name_, ros_nh_)){
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }
  setFilterChain(filter_chain_);

  // ROS
  ros_sub_robot_pose_ = ros_nh_.subscribe(topic_robot_state_, 2, &TerrainSimplificationRos::subRobotPose, this);
  ros_sub_map_        = ros_nh_.subscribe(topic_elevation_map_, 2, &TerrainSimplificationRos::subGridMap, this);
  ros_pub_map_        = ros_nh_.advertise<grid_map_msgs::GridMap>(topic_map_simplified_, 1, false);
  ros_server_run_     = ros_nh_.advertiseService("/terrain_simplification/run", &TerrainSimplificationRos::run, this);
  ros_server_stop_    = ros_nh_.advertiseService("/terrain_simplification/stop", &TerrainSimplificationRos::stop, this);
  ros_server_pub_     = ros_nh_.advertiseService("/terrain_simplification/pub", &TerrainSimplificationRos::pub, this);
  ros_server_read_    = ros_nh_.advertiseService("/terrain_simplification/read", &TerrainSimplificationRos::read, this);
  ros_server_get_val_ = ros_nh_.advertiseService("/terrain_simplification/get_val", &TerrainSimplificationRos::getValueAtPosition, this);

  success = true;
}

bool TerrainSimplificationRos::run(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  setRun(true);
  return true;
}

bool TerrainSimplificationRos::stop(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  setRun(false);
  return true;
}

bool TerrainSimplificationRos::pub(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  if (isReady()) {
    pubSimplifiedGridMap();
  } else {
    if (!isReceived()) {
      ROS_WARN_STREAM("The elevation map was not received. "
                      "It either must be published on topic = " << topic_elevation_map_ <<
                      " or manually provided via setGridMap().");
    } else {
      ROS_WARN_STREAM("TerrainSimplification is not running. "
                      "It must be started via the /run service call. ");
    }
  }
  return true;
}

bool TerrainSimplificationRos::read(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  bool success = readParameters();
  return success;
}

bool TerrainSimplificationRos::getValueAtPosition(
    terrain_simplification_ros::GetValueAtPosition::Request &request,
    terrain_simplification_ros::GetValueAtPosition::Response &response) {
  Eigen::Vector2d p(request.x, request.y);
  bool is_inside = false;
  double val = TerrainSimplification::getValueAtPosition(request.layer, p, is_inside);

  if (is_inside) {
    if (request.layer == "simplified") {
      response.val = val + h_nominal_;
    } else {
      response.val = val;
    }
  } else {
    ROS_ERROR_STREAM("The provided position (" << p.transpose() << ") "
                     "is outside the map; "
                     "hence, the corresponding value cannot be obtained.");
  }
  return is_inside;
}

double TerrainSimplificationRos::getHeight(
    const Eigen::Vector2d& location,
    bool& is_inside,
    const bool& apply_h_offset) {
  double h = TerrainSimplification::getValueAtPosition("simplified", location, is_inside);
  if (apply_h_offset) h += h_nominal_;
  return  h;
}

double TerrainSimplificationRos::getTraversability(
    const Eigen::Vector2d& location,
    bool& is_inside) {
  TerrainSimplification::getValueAtPosition("traversability", location, is_inside);
}

bool
TerrainSimplificationRos::readParameters() {
  if (!ros_nh_.getParam("/terrain_simplification_ros_node/topic_elevation_map", topic_elevation_map_)){
    ROS_ERROR("Could not read parameter `terrain_simplification_ros_node/topic_elevation_map`.");
    return false;
  }
  if (!ros_nh_.getParam("/terrain_simplification_ros_node/topic_robot_state", topic_robot_state_)){
    ROS_ERROR("Could not read parameter `terrain_simplification_ros_node/topic_robot_state`.");
    return false;
  }
  if (!ros_nh_.getParam("/terrain_simplification_ros_node/topic_map_simplified", topic_map_simplified_)){
    ROS_ERROR("Could not read parameter `terrain_simplification_ros_node/topic_robot_state`.");
    return false;
  }
  if (!ros_nh_.getParam("/terrain_simplification_ros_node/filter_chain_parameter_name", filter_chain_parameters_name_)){
    ROS_ERROR("Could not read parameter `terrain_simplification_ros_node/filter_chain_parameter_name`.");
    return false;
  }

  ros_nh_.param("/terrain_simplification_ros_node/gridmap_size_x",   map_size_.x(),    2.5);
  ros_nh_.param("/terrain_simplification_ros_node/gridmap_size_y",   map_size_.y(),    2.5);
  ros_nh_.param("/terrain_simplification_ros_node/h_nominal",        h_nominal_,       0.53);
  ros_nh_.param("/terrain_simplification_ros_node/map_res_scaling",  map_res_scaling_, 0.4);
  setMapSize(map_size_);

  return true;
}

void
TerrainSimplificationRos::subRobotPose(
    const geometry_msgs::PoseWithCovarianceStamped& msg) {
  msg_robot_pose_ = msg;

  Eigen::Vector3d p (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z);
  Eigen::Quaterniond o (
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z);
  setRobotPose(p, o);
}

void
TerrainSimplificationRos::subGridMap(
    const grid_map_msgs::GridMap& msg) {
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  TerrainSimplification::setGridMap(map);
}

void
TerrainSimplificationRos::pubSimplifiedGridMap() {
  grid_map::GridMap map, map_h_offset;
  getSimplifiedGridMap(map, map_res_scaling_);
  Eigen::Isometry3d transf = Eigen::Isometry3d::Identity();
  Eigen::Vector3d translation(0.0, 0.0, h_nominal_);
  transf.translate(translation);
  map_h_offset = map.getTransformedMap(transf, "simplified", "point_cloud_odom", 1.0);

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map_h_offset, msg);
  ros_pub_map_.publish(msg);
}
}
