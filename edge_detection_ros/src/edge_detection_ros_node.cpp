#include "edge_detection_ros/edge_detection_ros.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "edge_detection_ros");
  ros::NodeHandle node_handle;
  std::string frame_name = "odom";
  edge_detection::EdgeDetectionRos edge_detection_ros(node_handle, frame_name, 0.4, 0.02);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}
