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
  // check that robot position is inside the map
  if (std::pow(std::pow(robot_position[0] - map_full_.getPosition()[0], 2.), 0.5) > map_full_.getLength()[0]/2. ||
      std::pow(std::pow(robot_position[1] - map_full_.getPosition()[1], 2.), 0.5) > map_full_.getLength()[1]/2.) {
    std::cout << "[TerrainSimplification::simplifyGridMap] The position of the robot (" << robot_position[0] << ", " << robot_position[1]
              << ") lays outside of the map centered at (" << map_full_.getPosition()[0] << ", " << map_full_.getPosition()[1]
              << "), of size (" << map_full_.getLength()[0] << ", " << map_full_.getLength()[1] << "). Will reattempt." << std::endl;
    mutex_.unlock();
    usleep(1000000);
    return;
  }
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
  MD img_simplified;
  MDD img_elevation;
  img_elevation.m = img_raw_;
  applyDirectionalGaussianBlurToCvImage(img_elevation.m, img_simplified.m);                             // takes 1-10 ms
  applyFirstOrderDerivativesToCvImage(img_simplified.m, img_simplified.f, 3, false);
  applyFirstAndSecondOrderDerivativesToCvImage(img_elevation.m, img_elevation, 3, true);

  convertCvImagesOfFirstOrderDerivativesToGridMap("simplified", img_simplified.f, map_simplified_wo_traversability);
  convertCvImagesOfFirstAndSecondOrderDerivativesToGridMap("elevation", img_elevation, map_simplified_wo_traversability);
  convertCvImageToGridMap("simplified", img_simplified.m, map_simplified_wo_traversability); // takes 1-2 ms
  convertCvImageToGridMap("elevation", img_elevation.m, map_simplified_wo_traversability); // takes 1-2 ms
  mutex_.lock(); // to write and img_simplified_
  img_simplified_ = img_simplified;
  img_elevation_ = img_elevation;
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
TerrainSimplification::convertCvImagesOfFirstAndSecondOrderDerivativesToGridMap(
    const std::string &layer_name,
    const MDD &images,
    grid_map::GridMap &map) {
  convertCvImagesOfFirstOrderDerivativesToGridMap(layer_name, images.f, map);
  convertCvImagesOfSecondOrderDerivativesToGridMap(layer_name, images.s, map);
}

void
TerrainSimplification::convertCvImagesOfFirstOrderDerivativesToGridMap(
    const std::string &layer_name,
    const D &images,
    grid_map::GridMap &map) {
  convertCvImageToGridMap(layer_name + "_d",  images.d,  map);
  convertCvImageToGridMap(layer_name + "_dx", images.dx, map);
  convertCvImageToGridMap(layer_name + "_dy", images.dy, map);
}

void
TerrainSimplification::convertCvImagesOfSecondOrderDerivativesToGridMap(
    const std::string &layer_name,
    const DD &images,
    grid_map::GridMap &map) {
//  convertCvImageToGridMap(layer_name + "_dd",   images.dd,   map); // note: currently .dd is not computed
  convertCvImageToGridMap(layer_name + "_dxdx", images.dxdx, map);
  convertCvImageToGridMap(layer_name + "_dxdy", images.dxdy, map);
  convertCvImageToGridMap(layer_name + "_dydx", images.dydx, map);
  convertCvImageToGridMap(layer_name + "_dydy", images.dydy, map);
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
TerrainSimplification::applyDirectionalGaussianBlurToCvImage (
    const cv::Mat& image,
    cv::Mat& image_filtered) {
  cv::Mat image_median;
  cv::medianBlur(image, image_median, 5);
  cv::Mat image_directional;
  cv::filter2D(image_median, image_directional, -1, getDirectionalBlurKernel());

  image_filtered = image_directional;
}

void
TerrainSimplification::applyGaussianBlurToCvImage (
    const cv::Mat& image,
    cv::Mat& image_filtered,
    const int& size) {
  cv::Mat image_gaussian;
  cv::GaussianBlur(image, image_gaussian, cv::Size(size, size), 0.0, 0.0);

  image_filtered = image_gaussian;
}

void
TerrainSimplification::applyDerivativeToCvImage(
    const cv::Mat& image,
    cv::Mat& image_filtered,
    const int& size,
    const Dim& dim,
    const bool& denoise) {
  // IF denoise, apply Gaussian Blur, ELSE copy image
  cv::Mat image_denoised, image_denoised_signed;
  if (denoise) {
    cv::GaussianBlur(image, image_denoised, cv::Size(3,3), 0, 0);
  } else {
    image_denoised = image;
  }
  // Compute the gradient with respect to the specified dimension
  cv::Mat image_grad, image_grad_unsigned;
  // Convert from insigned int [0, 65,535] to signed int [−32,767, +32,767]
  image_denoised.convertTo(image_denoised_signed, CV_16S, 0.5);       // note: multiplying by 0.5
  if (dim == X) {
    cv::Sobel(image_denoised_signed, image_grad, CV_16S, 0, 1, size); // note: 0, 1
  } else if (dim == Y) {
    cv::Sobel(image_denoised_signed, image_grad, CV_16S, 1, 0, size); // note: 1, 0
  } else {
    image_grad = image_denoised_signed;
  }

  image_grad.convertTo(image_grad_unsigned, CV_16UC1, 2.0, 32767.);    // note: multiplying by 2.0
  image_filtered = image_grad_unsigned;
}

void
TerrainSimplification::applyFirstOrderDerivativesToCvImage(
    const cv::Mat& image,
    D& images_filtered,
    const int& size,
    const bool& denoise) {
   applyDerivativeToCvImage(image, images_filtered.dx, size, X,  denoise);
   applyDerivativeToCvImage(image, images_filtered.dy, size, Y,  denoise);

   images_filtered.d = cv::Mat::zeros(images_filtered.dx.size().height, images_filtered.dx.size().width, CV_16UC1);
   int n_rows = images_filtered.dx.size().height;
   int n_cols = images_filtered.dx.size().width;
   mutex_state_.lock();
   double yaw = yaw_prev_;
   mutex_state_.unlock();
   double cos_yaw = cos(yaw);
   double sin_yaw = sin(yaw);
   for (int i = 0; i < n_rows; i++) {
     unsigned short* d_row        = images_filtered.d.ptr<unsigned short>(i);
     const unsigned short* dx_row = images_filtered.dx.ptr<unsigned short>(i);
     const unsigned short* dy_row = images_filtered.dy.ptr<unsigned short>(i);
     for (int j = 0; j < n_cols; j++) {
       d_row[j] = static_cast<unsigned short>(cos_yaw*(dx_row[j]-32767.) + sin_yaw*(dy_row[j]-32767.) + 32767.);
     }
   }
}

void
TerrainSimplification::applySecondOrderDerivativesToCvImage(
    const D& image,
    DD& images_filtered,
    const int& size,
    const bool& denoise) {
  applyDerivativeToCvImage(image.dx, images_filtered.dxdx, size, X,  denoise);
  applyDerivativeToCvImage(image.dx, images_filtered.dxdy, size, Y,  denoise);
  applyDerivativeToCvImage(image.dy, images_filtered.dydx, size, X,  denoise);
  applyDerivativeToCvImage(image.dy, images_filtered.dydy, size, Y,  denoise);
}

void
TerrainSimplification::applyFirstAndSecondOrderDerivativesToCvImage(
    const cv::Mat& image,
    MDD& images_filtered,
    const int& size,
    const bool& denoise) {
  applyFirstOrderDerivativesToCvImage(image, images_filtered.f, size, denoise);
  applySecondOrderDerivativesToCvImage(images_filtered.f, images_filtered.s, size, false);
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

    // Convert GridMap layers to images
    mutex_.lock(); // to read map_simplified_
    convertGridMapToCvImage("traversability", map_simplified_, img_traversability_);
    convertGridMapToCvImage("slope", map_simplified_, img_slope_);
    mutex_.unlock();

    // Scale images
    mutex_.lock(); // to read img_simplified_
    MD img_simplified = img_simplified_;
    MDD img_elevation = img_elevation_;
    mutex_.unlock();

    scaleCvImage(scale, img_simplified.m,    img_simplified_scaled_.m);
    scaleCvImage(scale, img_simplified.f.d,  img_simplified_scaled_.f.d);
    scaleCvImage(scale, img_simplified.f.dx, img_simplified_scaled_.f.dx);
    scaleCvImage(scale, img_simplified.f.dy, img_simplified_scaled_.f.dy);

    scaleCvImage(scale, img_elevation.m,      img_elevation_scaled_.m);
    scaleCvImage(scale, img_elevation.f.d,    img_elevation_scaled_.f.d);
    scaleCvImage(scale, img_elevation.f.dx,   img_elevation_scaled_.f.dx);
    scaleCvImage(scale, img_elevation.f.dy,   img_elevation_scaled_.f.dy);
    scaleCvImage(scale, img_elevation.s.dxdx, img_elevation_scaled_.s.dxdx);
    scaleCvImage(scale, img_elevation.s.dxdy, img_elevation_scaled_.s.dxdy);
    scaleCvImage(scale, img_elevation.s.dydx, img_elevation_scaled_.s.dydx);
    scaleCvImage(scale, img_elevation.s.dydy, img_elevation_scaled_.s.dydy);

    scaleCvImage(scale, img_traversability_, img_traversability_scaled_);
    scaleCvImage(scale, img_slope_, img_slope_scaled_);

    // Convert scaled images to GridMap layers
    convertCvImageToGridMap("slope", img_slope_scaled_, map_simplified_scaled_);
    convertCvImageToGridMap("traversability", img_traversability_scaled_, map_simplified_scaled_);
    convertCvImageToGridMap("simplified", img_simplified_scaled_.m, map_simplified_scaled_);
    convertCvImagesOfFirstOrderDerivativesToGridMap("simplified", img_simplified_scaled_.f, map_simplified_scaled_);
    convertCvImagesOfFirstAndSecondOrderDerivativesToGridMap("elevation", img_elevation_scaled_, map_simplified_scaled_);
    convertCvImageToGridMap("elevation", img_elevation_scaled_.m, map_simplified_scaled_); // takes 1-2 ms

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
    std::cerr << "[TerrainSimplification::getValueAtPosition] Error: the queried position (" << position
              << ") lays outside of the map centered at (" << map_simplified_.getPosition()[0] << ", " << map_simplified_.getPosition()[1]
              << "), of size (" << map_simplified_.getLength()[0] << ", " << map_simplified_.getLength()[1] << ")." << std::endl;
    is_inside = false;
  }
  return value;
}
}
