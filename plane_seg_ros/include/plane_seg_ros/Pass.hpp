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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <plane_seg/Tracker.hpp>
#include <plane_seg_ros/Visualizer.hpp>

namespace planeseg {

class Pass{
  public:
    Pass(ros::NodeHandle node_);

    ~Pass(){
    }

    void elevationMapCallback(const grid_map_msgs::GridMap& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    void processCloud(planeseg::LabeledCloud::Ptr& inCloud, Eigen::Vector3f origin, Eigen::Vector3f lookDir);
    void processFromFile(int test_example);
    void stepThroughFile(std::string filename);
    void publishHullsAsCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs,
                                 int secs, int nsecs);

    void publishHullsAsMarkers(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs);
    void printResultAsJson();
    void publishResult();
    void publishIdsAsStrings();
    void publishCentroidsAsSpheres();

  private:
    ros::NodeHandle node_;
    std::vector<double> colors_;

    ros::Subscriber point_cloud_sub_, grid_map_sub_, pose_sub_;
    ros::Publisher received_cloud_pub_, hull_cloud_pub_, hull_markers_pub_, look_pose_pub_, elev_map_pub_, pose_pub_, id_strings_pub_, centroids_pub_;

    Eigen::Isometry3d last_robot_pose_;
    planeseg::BlockFitter::Result result_;
    planeseg::Tracker tracking_;
    planeseg::Visualizer visualizer_;
};
}
