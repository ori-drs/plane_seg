#include <terrain_simplification/terrain_simplification.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <ros/ros.h>
#include <iostream>

using Clock = std::chrono::high_resolution_clock;

namespace terrain_simplification {

TerrainSimplification::TerrainSimplification ()
  : thread_loop_(true),
    thread_run_(&TerrainSimplification::run, this) {
}

TerrainSimplification::~TerrainSimplification() {
  if (thread_run_.joinable()) {
    thread_loop_ = false;
    // Attempting to join the thread running the simplification with the main thread of TerrainSimplification...
    thread_run_.join();
  } else {
    std::cerr << "[TerrainSimplification::~TerrainSimplification] "
                 "The thread running the MPC could NOT be joined with "
                 "the main thread of TerrainSimplification: "
                 "thread_run_.joinable() = " << std::endl;
  }
}

void
TerrainSimplification::run() {
  // continously run the thread, until stopped
  while (thread_loop_.load()) {
    // IF flags true, advance the simplifying algorithm to produce to create simplified map
    // ELSE wait for the flags
    if (run_ && received_) {
      advance();
    } else {
      // wait 0.2 seconds
      for (int i = 0; i < 100; i++) {
        usleep(2000);
      }
    }
  }
}

bool
TerrainSimplification::advance() {
  simplifyGridMap();
  return true;
}

void
TerrainSimplification::setGridMap(
    const grid_map::GridMap& map) {
  std::lock_guard<std::mutex> lock(mutex_); // to write map_full_ and received_
  map_full_ = map;
  received_ = true;
}

double
TerrainSimplification::getRobotYaw() {
  std::lock_guard<std::mutex> lock(mutex_state_); // to read and write
  double yaw_prev = yaw_prev_;
  const double q0 = robot_orientation_.w();
  const double q1 = robot_orientation_.x();
  const double q2 = robot_orientation_.y();
  const double q3 = robot_orientation_.z();
  double yaw   = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));

  // unwrap angle (see MATLAB's unwrap() for more info)
  double d = yaw - yaw_prev;
  d = d > M_PI ? d - 2.0*M_PI : (d < -M_PI ? d + 2.0*M_PI : d);
  yaw_prev += d;
  yaw_prev_ = yaw_prev;
  return yaw_prev;
}

void
TerrainSimplification::simplifyGridMap () {
  bool success = false;

  // Extract the submap from the elevation map
  mutex_state_.lock();
  grid_map::Position robot_position(robot_position_[0], robot_position_[1]);
  mutex_state_.unlock();

  mutex_.lock(); // to read map_full_
  map_sub_ = map_full_.getSubmap(robot_position, grid_map::Length(map_size_.x(), map_size_.y()), success);
  mutex_.unlock();

  // Create empty map
  grid_map::GridMap map_simplified_wo_traversability({"simplified"});
  map_simplified_wo_traversability.setFrameId("point_cloud_odom");
  map_simplified_wo_traversability.setGeometry(map_sub_.getLength(), 0.02);
  map_simplified_wo_traversability.setPosition(robot_position);

  // Copy orignal layers
  std::vector<std::string> layers = {"elevation"};
  if (!map_simplified_wo_traversability.addDataFrom(map_sub_, true, true, true, layers)) {
    ROS_ERROR("Could not add data from map_sub_ to map_filtered_.");
    return;
  }

  // Simplify the terrain
  convertGridMapToCvImage("elevation", map_sub_, img_raw_);             // takes 0.5 ms
  mutex_.lock(); // to write and img_simplified_
  filterCvImage(img_raw_, img_simplified_);                             // takes 1-10 ms
  convertCvImageToGridMap("simplified", img_simplified_, map_simplified_wo_traversability); // takes 1-2 ms
  mutex_.unlock();

  // Apply the filter chain to create a separate map with a traversability layer
  grid_map::GridMap map_simplified({"simplified"});
  applyFilterChain(map_simplified_wo_traversability, map_simplified);                       // takes ~100-200 ms

  mutex_.lock(); // to write map_filtered_ and map_simplified_
  map_simplified_wo_traversability_ = map_simplified_wo_traversability;
  map_simplified_ = map_simplified;
  ready_ = true;
  mutex_.unlock();
}

void
TerrainSimplification::convertGridMapToCvImage (
    const std::string& layer_name,
    const grid_map::GridMap& map,
    cv::Mat& image) {
  grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
        map, layer_name, CV_16UC1, 0.0, 1.0, image);

}

void
TerrainSimplification::convertCvImageToGridMap (
    const std::string& layer_name,
    const cv::Mat& image,
    grid_map::GridMap& map) {
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(
        image, layer_name, map);
}

void
TerrainSimplification::filterCvImage (
    const cv::Mat& image,
    cv::Mat& image_filtered) {

  cv::Mat image_median; cv::medianBlur(image, image_median, 5);

  // not used
  //  cv::Mat image_gaussian;
  //  int size_x = abs(2*floor((sin(getRobotYaw())*51.)/2.)+1);
  //  int size_y = abs(2*floor((cos(getRobotYaw())*51.)/2.)+1);
  //  cv::GaussianBlur(image_median, image_gaussian, cv::Size(size_x, size_y), 0.0, 0.0);

  cv::Mat image_directional;
  cv::filter2D(image_median, image_directional, -1, getDirectionalBlurKernel());

  image_filtered = image_directional;
}

cv::Mat
TerrainSimplification::getDirectionalBlurKernel() {
  cv::Mat k;
  k = cv::Mat::zeros(k_size_, k_size_, CV_64F);
  k.row((int)(k_size_-1)/2) = cv::getGaussianKernel(k_size_, -1, CV_64F).t();
  double yaw = getRobotYaw();
  cv::warpAffine(
        k,
        k,
        cv::getRotationMatrix2D(
          cv::Point2f((k_size_-1)/2,(k_size_-1)/2),
          (double)yaw*180./M_PI-90,
          1.0),
        cv::Size(k_size_, k_size_)
        );
  k = k * (double)(1./ cv::sum(k)[0]);
  return k;
}

void
TerrainSimplification::applyFilterChain(
    const grid_map::GridMap &map_in,
    grid_map::GridMap &map_out) {
  // Apply filter chain.
  if (!filter_chain_->update(map_in, map_out)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }
}

void
TerrainSimplification::getSimplifiedGridMap(
    grid_map::GridMap& simplified_map,
    const double& scale) {

  // IF the returned map is to be scaled ELSE not scaled
  if (scale != 1.0) {

    // Set up the map object
    mutex_.lock(); // to read map_filtered_
    map_simplified_scaled_.setFrameId("point_cloud_odom");
    map_simplified_scaled_.setGeometry(map_simplified_wo_traversability_.getLength(), 0.02/scale);
    map_simplified_scaled_.setPosition(map_simplified_wo_traversability_.getPosition());
    mutex_.unlock();

    // Scale
    mutex_.lock(); // to read img_simplified_
    scaleCvImage(scale, img_simplified_, img_simplified_scaled_);
    mutex_.unlock();

    convertCvImageToGridMap("simplified", img_simplified_scaled_, map_simplified_scaled_);

    mutex_.lock(); // to read map_simplified_
    convertGridMapToCvImage("traversability", map_simplified_, img_traversability_);
    mutex_.unlock();

    scaleCvImage(scale, img_traversability_, img_traversability_scaled_);
    convertCvImageToGridMap("traversability", img_traversability_scaled_, map_simplified_scaled_);

    simplified_map = map_simplified_scaled_;
  } else {
    mutex_.lock(); // to read map_simplified_
    simplified_map = map_simplified_;
    mutex_.unlock();
  }
}

void
TerrainSimplification::scaleCvImage(
    const double &scale,
    const cv::Mat &image,
    cv::Mat &image_scaled) {
  int size_x = (int) (scale*image.size().width);
  int size_y = (int) (scale*image.size().height);
  image_scaled = cv::Mat::zeros(size_x, size_y, CV_16UC1);
  cv::resize(image, image_scaled, cv::Size(size_x, size_y));
}

double
TerrainSimplification::getValueAtPosition(
    const std::string& layer,
    const Eigen::Vector2d& position,
    bool& is_inside) {
  grid_map::Position p(position);
  std::lock_guard<std::mutex> lock(mutex_);
  double value = 0.0;
  if (map_simplified_.isInside(p)) {
    is_inside = true;
    value = map_simplified_.atPosition(layer, p);
  } else {
    is_inside = false;
  }
  return value;
}
}
