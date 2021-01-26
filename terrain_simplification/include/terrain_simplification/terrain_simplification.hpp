/**
 * @file terrain_simplification.hpp
 * @brief A module that simplifies the observed terrain, represented as a gridmap,
 * by applying an OpenCV-based low-pass filter to extract the high-level geometry
 * and qualify its irregularity by computing traversability using a filter chain.
 * @author Oliwier Melon (omelon@robots.ox.ac.uk)
 * @bug No known bugs.
 * @date 07/01/2021
 * @version 1.1
 * @note The computation can be accelerated if the functionality of the Filter Chain
 * were manually implemented.
 * @copyright 2020, Oliwier Melon. BSD-3-Clause
 */

#ifndef TERRAIN_SIMPLIFICATION_HPP
#define TERRAIN_SIMPLIFICATION_HPP

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.h>

#include <thread>
#include <atomic>
#include <mutex>

namespace terrain_simplification {

class TerrainSimplification {

public:
  /**
   * @brief Constructs a multi-threaded (2) object
   */
  TerrainSimplification();

  /**
   * @brief Destructs the multi-threaded (2) object
   */
  ~TerrainSimplification();

protected:
  /**
   * @brief Assigns a shared pointer to the filter_chain_ object
   * @param[in] filter_chain  shared pointer to a filter chain object in the derived class (ros).
   */
  void setFilterChain(
      std::shared_ptr<filters::FilterChain<grid_map::GridMap> > filter_chain) {
    filter_chain_ = filter_chain;
  }

  /**
   * @brief Set the position and orientation of the robot
   * @param[in] position     the position of the robot
   * @param[in] orientation  the orientation of the robot
   */
  virtual void setRobotPose (
      const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation) {
    robot_position_ = position;
    robot_orientation_ = orientation;
  }

  /**
   * @brief Computes the yaw angle from the local copy of the robot's orientation.
   * Stores the result in a member variable;
   * @return Yaw angle
   */
  double getRobotYaw ();

  /**
   * @brief Sets a member variable of the full map and sets received_ flag to true.
   * @param[in] map  the full elevation map
   */
  virtual void setGridMap(
      const grid_map::GridMap& map);

  /**
   * @brief Simplifies the acquired elevation map and stores the result in a member variable.
   */
  void simplifyGridMap();

  /**
   * @brief Converts a layer of a gridmap to an image
   * @param[in]  layer_name  the name of the layer from which to extract the gridmap
   * @param[in]  map         the input gridmap
   * @param[out] image       the output image
   */
  void convertGridMapToCvImage(
      const std::string& layer_name,
      const grid_map::GridMap& map,
      cv::Mat& image);

  /**
   * @brief Converts an image to a layer of a gridmap
   * @param[in]  layer_name  the name of the layer
   * @param[in]  image       the input image
   * @param[out] map         the output gridmap
   */
  void convertCvImageToGridMap(
      const std::string& layer_name,
      const cv::Mat& image,
      grid_map::GridMap& map);

  /**
   * @brief Filters an image using OpenCV filters by applying directional blur
   * @param[in]     image           the input image
   * @param[in/out] image_filtered  the output image
   */
  void filterCvImage(
      const cv::Mat& image,
      cv::Mat& image_filtered);

  /**
   * @brief Scales an image
   * @param[in]     scale         the scaling factor
   * @param[in]     image         the input image
   * @param[in/out] image_scaled  the output image
   */
  void scaleCvImage(
      const double& scale,
      const cv::Mat& image,
      cv::Mat& image_scaled);

  /**
   * @brief Gets the Kernel dependent on robot's orientation (yaw)
   * for the 2D filter (directional Gaussian blur)
   */
  cv::Mat getDirectionalBlurKernel();

  /**
   * @brief Gets the simplified gridmap of the terrain
   * @param[in/out]  simplified_map  the simplified map
   * @param[in]      scale           an optional scaling factor
   */
  void getSimplifiedGridMap(
      grid_map::GridMap& simplified_map,
      const double& scale = 1.0);

  /**
   * @brief Applies a sequence of filters defined via the Filter Chain to a gridmap
   * @param[in]      map_in   the input map without the filters applied
   * @param[in/out]  map_out  the output map with the filters applied
   */
  void applyFilterChain(
      const grid_map::GridMap& map_in,
      grid_map::GridMap& map_out);

  /**
   * @brief  Gets a value from a layer of the simplified map at a specific position
   * @param  layer name of the accessed layer
   * @param  position xy-coordinates
   * @param  is_inside flag that states whether the provided position is inside the map
   * @return value at a position
   */
  double getValueAtPosition(
      const std::string& layer,
      const Eigen::Vector2d& position,
      bool& is_inside);

  /**
   * @brief The run function for the TerrainSimplification
   * which calls advance() in a while loop.
   * It is started in a separate thread by a rosservice callback function.
   */
  void run();

  /**
   * @brief The run function for the TerrainSimplification
   * which calls advance() in a while loop.
   * It is started in a separate thread by a rosservice callback function.
   */
  void setRun(const bool& run) { run_ = run; }

  /**
   * @brief Set the size of the simplified map
   */
  void setMapSize(const Eigen::Vector2d& map_size) { map_size_ = map_size; }

  /**
   * @brief The advance function for the TerrainSimplification.
   * It can be stopped by a rosservice call which sets the stop_ flag.
   * @return true if successful
   */
  bool advance();

  /**
   * @brief Returns the received_ flag indicating that the raw elevation map has been received
   * @return true if received_
   */
  bool isReceived() { return received_; }

  /**
   * @brief Returns the ready_ flag indicating that the simplified map has been created
   * @return true if ready_
   */
  bool isReady() { return ready_; }

private:
  // Flags
  bool run_ = true;               ///< flag allowing run() to execute
  bool received_ = false;         ///< flag indicating that the raw elevation map has been received
  bool ready_ = false;            ///< flag indicating that the simplified map has been created
  std::thread thread_run_;        ///< thread for run()
  std::atomic_bool thread_loop_;  ///< flag for the thread

  // Gridmaps
  grid_map::GridMap map_full_;              ///< full elevation map
  grid_map::GridMap map_sub_;               ///< submap of the full elevation map
  grid_map::GridMap map_simplified_wo_traversability_;  ///< filtered submap, without the Filter Chain applied (because the class is multi-threaded and applying the filter takes significant time)
  grid_map::GridMap map_simplified_;        ///< filtered submap, with the Filter Chain applied
  grid_map::GridMap map_simplified_scaled_; ///< scaled filtered submap, with the Filter Chain applied

  // Gridmaps' parameters
  Eigen::Vector2d map_size_;                ///< size of the submap
  int k_size_ = 51;                         ///< size of the kernel (for the directional Gaussian blur filter)

  // Filter Chain
  std::shared_ptr<filters::FilterChain<grid_map::GridMap> > filter_chain_; ///< filter chain for traversability layer

  // Images
  cv::Mat img_raw_;                         ///< image of the raw elevation map
  cv::Mat img_simplified_;                  ///< image of the simplified map
  cv::Mat img_simplified_scaled_;           ///< scaled image of the simplified map
  cv::Mat img_traversability_;              ///< image of the traversability map
  cv::Mat img_traversability_scaled_;       ///< scaled image of the traversability map

  // Robot's state
  Eigen::Vector3d robot_position_;          ///< robot's xyz position in the world (odom) frame
  Eigen::Quaterniond robot_orientation_;    ///< robot's base orientation (quaternion) in the base frame, aligned with the world frame
  double yaw_prev_ = 0.0;                   ///< robot's base orientation (yaw) in the base frame, aligned with the world frame

  // Mutex
  std::mutex mutex_;               ///< mutex for map_ and img_ variables
  std::mutex mutex_state_;         ///< mutex for robot's position and orientation variables

};

} //end namespace

#endif //TERRAIN_SIMPLIFICATION_HPP
