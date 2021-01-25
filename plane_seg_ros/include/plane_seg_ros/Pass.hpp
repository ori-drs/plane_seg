#pragma once
#include <Eigen/Dense>
#include <vector>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <plane_seg/BlockFitter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <plane_seg/Tracker.hpp>
#include <plane_seg_ros/Visualizer.hpp>
#include <plane_seg/ImageProcessor.hpp>
#include <chrono>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <filters/filter_chain.h>
#include <grid_map_core/GridMap.hpp>

namespace planeseg {

class Pass{
  public:
    Pass(ros::NodeHandle node_);

    ~Pass(){
    }

    bool loadParameters();
    void elevationMapCallback(const grid_map_msgs::GridMap& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void imageProcessingCallback(const grid_map_msgs::GridMap &msg);

    void processCloud(planeseg::LabeledCloud::Ptr& inCloud, Eigen::Vector3f origin, Eigen::Vector3f lookDir);
    void processFromFile(int test_example);
    void stepThroughFile(std::string filename);
    void publishHullsAsCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs,
                                 int secs, int nsecs);

    void publishHullsAsMarkersOLD(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs);
    void printResultAsJson();
    void publishResult();
    void publishIdsAsStrings();
    void publishCentroidsAsSpheres();
    void publishHullsAsMarkers();
    void publishLineStrips();
    void extractNthCloud(std::string filename, int n);
    void tic();
    std::chrono::duration<double> toc();
    void gridMapCallback(const grid_map_msgs::GridMap& msg);

  private:
    ros::NodeHandle node_;
    std::vector<double> colors_;
    std::vector<double> colors_2;
    std::vector<double> colors_3;

    ros::Subscriber point_cloud_sub_, grid_map_sub_, pose_sub_;
    ros::Publisher received_cloud_pub_, hull_cloud_pub_, hull_markers_pub_, look_pose_pub_, elev_map_pub_, pose_pub_, id_strings_pub_, centroids_pub_, hulls_pub_, linestrips_pub_, filtered_map_pub_;

    Eigen::Isometry3d last_robot_pose_;
    planeseg::BlockFitter::Result result_;
    planeseg::Tracker tracking_;
    planeseg::Visualizer visualizer_;
    planeseg::ImageProcessor imgprocessor_;
    filters::FilterChain<grid_map::GridMap> filter_chain_;
    tf::TransformListener* listener_;
    std::string elevation_map_topic_;
    std::string filter_chain_parameters_name_;
    std::string filtered_map_topic_;
    double erode_radius_;
    double traversability_threshold_;
    bool verbose_timer_;
    std::chrono::high_resolution_clock::time_point last_time_;
};
}
