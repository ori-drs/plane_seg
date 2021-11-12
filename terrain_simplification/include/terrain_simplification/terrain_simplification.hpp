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

#if defined(__GNUG__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <filters/filter_chain.hpp>

#if defined(__GNUG__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <thread>
#include <atomic>
#include <mutex>

namespace terrain_simplification {

enum Dim { X=0, Y, XY };

struct D {
  cv::Mat d;
  cv::Mat dx;
  cv::Mat dy;
};

struct DD {
  cv::Mat dd;
  cv::Mat dxdx;
  cv::Mat dxdy;
  cv::Mat dydx;
  cv::Mat dydy;
};

struct MD {
  cv::Mat m;
  D f;
};

struct MDD {
  cv::Mat m;
  D f;
  DD s;
};

class TerrainSimplification {

public:
  /**
   * @brief Constructs a multi-threaded (2) object
   */
  TerrainSimplification();

  /**
   * @brief Destructs the multi-threaded (2) object
   */
  virtual ~TerrainSimplification();

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
   * @brief Computes the unwrapped yaw angle from the local copy of the robot's orientation.
   * Stores the result in a member variable;
   * @return Yaw angle
   */
  double getRobotYaw ();

  /**
   * @brief Sets a member variable of the full map and sets received_ flag to true.
   * @param[in] map      the full elevation map
   * @param[in] o_T_pco  the transformation from point_cloud_odom to odom frame
   */
  virtual void setGridMap(
      const grid_map::GridMap& map,
      const Eigen::Isometry3d& o_T_pco);

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
      cv::Mat& image,
      const float lower_value = 0.0,
      const float upper_value = 1.0);

  /**
   * @brief Converts images of first- and second-order derivatives to layers of a gridmap
   * @param[in]  layer_name  the common name of the layers
   * @param[in]  images      the input images
   * @param[out] map         the output gridmap
   */
  void convertCvImagesOfFirstAndSecondOrderDerivativesToGridMap(
      const std::string& layer_name,
      const MDD& images,
      grid_map::GridMap& map);

  /**
   * @brief Converts images of first-order derivatives to layers of a gridmap
   * @param[in]  layer_name  the common name of the layers
   * @param[in]  images       the input images
   * @param[out] map         the output gridmap
   */
  void convertCvImagesOfFirstOrderDerivativesToGridMap(
      const std::string& layer_name,
      const D& images,
      grid_map::GridMap& map);

  /**
   * @brief Converts images of second-order derivatives to layers of a gridmap
   * @param[in]  layer_name  the common name of the layers
   * @param[in]  images      the input images
   * @param[out] map         the output gridmap
   */
  void convertCvImagesOfSecondOrderDerivativesToGridMap(
      const std::string& layer_name,
      const DD& images,
      grid_map::GridMap& map);

  /**
   * @brief Converts an image to a layer of a gridmap
   * @param[in]  layer_name  the name of the layer
   * @param[in]  image       the input image
   * @param[out] map         the output gridmap
   */
  void convertCvImageToGridMap(
      const std::string& layer_name,
      const cv::Mat& image,
      grid_map::GridMap& map,
      const float lower_value = 0.0,
      const float upper_value = 1.0);

  /**
   * @brief Filters an image using OpenCV filters by applying directional blur
   * @param[in]     image           the input image
   * @param[in/out] image_filtered  the output image
   */
  void applyDirectionalGaussianBlurToCvImage(
      const cv::Mat& image,
      cv::Mat& image_filtered);

  /**
   * @brief Filters an image using OpenCV filters by applying first- and second-order (Sobel) derivatives
   * @param[in]     image           the input image
   * @param[in/out] images_filtered the output images (d, dx, dy, dd, dxdx, dxdy, dydx, dydy)
   * @param[in]     size            the size of the kernel
   * @param[in]     denoise         flag to apply Gaussian blur as a method of removing noise
   */
  void applyFirstAndSecondOrderDerivativesToCvImage(
      const cv::Mat& image,
      MDD& images_filtered,
      const int& size,
      const bool& denoise);

  /**
   * @brief Filters an image using OpenCV filters by applying second-order (Sobel) derivatives
   * @param[in]     image           the input images (dx, dy)
   * @param[in/out] images_filtered the output images (dd, dxdx, dxdy, dydx, dydy)
   * @param[in]     size            the size of the kernel
   * @param[in]     denoise         flag to apply Gaussian blur as a method of removing noise
   */
  void applySecondOrderDerivativesToCvImage(
      const D& image,
      DD& images_filtered,
      const int& size,
      const bool& denoise);

  /**
   * @brief Filters an image using OpenCV filters by applying first-order (Sobel) derivatives
   * @param[in]     image           the input image
   * @param[in/out] images_filtered the output images (d, dx, dy)
   * @param[in]     size            the size of the kernel
   * @param[in]     denoise         flag to apply Gaussian blur as a method of removing noise
   */
  void applyFirstOrderDerivativesToCvImage(
      const cv::Mat& image,
      D& images_filtered,
      const int& size,
      const bool& denoise);

  /**
   * @brief Filters an image using OpenCV filters by applying first-order (Sobel) derivative
   * @param[in]     image           the input image
   * @param[in/out] image_filtered  the output image
   * @param[in]     size            the size of the kernel
   * @param[in]     dim             dimension {x, Y} with respect to which to take the derivative
   * @param[in]     denoise         flag to apply Gaussian blur as a method of removing noise
   */
  void applyDerivativeToCvImage(
      const cv::Mat& image,
      cv::Mat& image_filtered,
      const int& size,
      const Dim& dim,
      const bool& denoise);

  /**
   * @brief Filters an image using OpenCV filters by applying Gaussian blur
   * @param[in]     image           the input image
   * @param[in/out] image_filtered  the output image
    @param[in]      size            the size of the kernel
   */
  void applyGaussianBlurToCvImage(
      const cv::Mat& image,
      cv::Mat& image_filtered,
      const int& size);

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
   * @brief Set the resolution of the simplified map
   */
  void setMapResolution(double resolution) { grid_map_resolution_ = resolution; }

  /**
   * @brief Set the size of the simplified map
   */
  void setApplyFilterChain(bool apply_filter_chain) { apply_filter_chain_ = apply_filter_chain; }

  /**
   * @brief Set the name of the input layer
   */
  void setNameOfInputLayer(const std::string& layer) {
    layer_ = layer;
  }

  /**
   * @brief Set the name of the input layer, if filter chain were applied
   */
  void setNameOfInputLayerFiltered(const std::string& layer_filtered) {
    layer_filtered_ = layer_filtered;
  }

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
  bool apply_filter_chain_ = true;  ///< flag indicating that filter chain should be applied
  double grid_map_resolution_ = 0.02;  ///< grid map resolution
  std::thread thread_run_;        ///< thread for run()
  std::atomic_bool thread_loop_;  ///< flag for the thread

  // Transformations
  Eigen::Isometry3d o_T_pco_;     ///< a transformation from point_cloud_odom to odom frame

  // Layers
  std::string layer_ = "elevation";  ///< name of the elevation layer in the
                                        ///< incoming elevation map
  std::string layer_filtered_ = "elevation";  ///< name of the filtered elevation layer
                                              ///< in the incoming elevation map

  // Gridmaps
  grid_map::GridMap map_full_pco_;          ///< submap of the full elevation map in point_cloud_odom frame
  grid_map::GridMap map_full_;              ///< full elevation map
  grid_map::GridMap map_sub_;               ///< submap of the full elevation map
  grid_map::GridMap map_sub_inpainted_;     ///< inpainted submap of the full elevation map
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
  cv::Mat img_inpainted_;                   ///< image of the inpainted elevation map
  MD img_simplified_;                       ///< images of the simplified map and its first-order derivatives
  MD img_simplified_scaled_;                ///< scaled images of the simplified map and its first-order derivatives
  MDD img_elevation_;                       ///< images of the elevation map and its second-order derivatives
  MDD img_elevation_scaled_;                ///< scaled images of the elevation map and its second-order derivatives
  cv::Mat img_traversability_;              ///< image of the traversability map
  cv::Mat img_traversability_scaled_;       ///< scaled image of the traversability map
  cv::Mat img_slope_;                       ///< image of the slope map
  cv::Mat img_slope_scaled_;                ///< scaled image of the slope map

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
