/**
 * @file terrain_simplification_ros.hpp
 * @brief A ros interface to the TerrainSimplification class,
 * @author Oliwier Melon (omelon@robots.ox.ac.uk)
 * @bug No known bugs.
 * @date 07/01/2021
 * @version 1.1
 * @copyright 2020, Oliwier Melon. BSD-3-Clause
 */

#ifndef TERRAIN_SIMPLIFICATION_ROS_HPP
#define TERRAIN_SIMPLIFICATION_ROS_HPP

#include <terrain_simplification/terrain_simplification.hpp>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <filters/filter_chain.h>

#include <Eigen/Eigen>

namespace terrain_simplification_ros {

class TerrainSimplificationRos
    : public terrain_simplification::TerrainSimplification {

public:
  TerrainSimplificationRos();

  /**
   * @brief Construct the ROS-based interface to the terrain simplification module.
   * @param node_handle
   */
  TerrainSimplificationRos(
      bool& success,
      ros::NodeHandle& nh);
  ~TerrainSimplificationRos() = default;

protected:
  void subGridMap(const grid_map_msgs::GridMap& msg);
  void subRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg);

  void pubSimplifiedGridMap();

  bool readParameters();

  /**
   * @brief The callback function used to start the run function in TerrainSimplification
   */
  bool run(
      std_srvs::Empty::Request& request,
      std_srvs::Empty::Response& response);

  /**
   * @brief The callback function used to stop the run function in TerrainSimplification
   */
  bool stop(
      std_srvs::Empty::Request& request,
      std_srvs::Empty::Response& response);

  /**
   * @brief The callback function used to publish the simplified map
   */
  bool pub(
      std_srvs::Empty::Request& request,
      std_srvs::Empty::Response& response);

  /**
   * @brief The callback function used to read ros parameters
   */
  bool read(
      std_srvs::Empty::Request& request,
      std_srvs::Empty::Response& response);

  /**
   * @brief Obtains the height of the simplified map with the nominal height offset (h_nominal) applied by default
   * @param location xy-coordinates of a location
   * @param apply_h_offset flag to apply the nominal height offset
   */
  double getHeight(
      const Eigen::Vector2d& location,
      const bool& apply_h_offset = true);

  /**
   * @brief Obtains the traversability of the traversability
   * @param location xy-coordinates of a location
   */
  double getTraversability(
      const Eigen::Vector2d& location);

private:
  // ROS
  ros::NodeHandle     ros_nh_;              ///< node handle
  ros::Subscriber     ros_sub_robot_pose_;  ///< subscriber robot pose
  ros::Subscriber     ros_sub_map_;         ///< subscriber elevation map
  ros::Publisher      ros_pub_map_;         ///< publisher simplified map
  ros::ServiceServer  ros_server_run_;      ///< service server to start terrain simplification
  ros::ServiceServer  ros_server_stop_;     ///< service server to stop terrain simplification
  ros::ServiceServer  ros_server_pub_;      ///< service server to publish simplified map
  ros::ServiceServer  ros_server_read_;     ///< service server to read ros parameters

  std::string topic_elevation_map_;         ///< topic name of the elevation map
  std::string topic_robot_state_;           ///< topic name of the robot state
  std::string topic_map_simplified_;        ///< topic name of the simplified map

  std::shared_ptr<filters::FilterChain<grid_map::GridMap> > filter_chain_; ///< filter chain for traversability layer
  std::string filter_chain_parameters_name_; ///< parameters for the filter chain

  // Robot pose
  geometry_msgs::PoseWithCovarianceStamped msg_robot_pose_; ///< message robot pose

  // Parameters
  Eigen::Vector2d map_size_;                ///< size of simplified map
  double h_nominal_ = 0.53;                 ///< nominal robot height
  double map_res_scaling_ = 0.4;            ///< map resolution scaling

  // GridMap
  grid_map::GridMap map_sub_;               ///< submap (of size map_size_) of the elevation map
  grid_map::GridMap map_simplified_;        ///< the simplified map

};
}

#endif //TERRAIN_SIMPLIFICATION_ROS_HPP