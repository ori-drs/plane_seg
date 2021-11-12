#include "terrain_simplification_ros/terrain_simplification_ros.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "terrain_simplification");
  ros::NodeHandle node_handle("~");
  terrain_simplification_ros::TerrainSimplificationRos terrain_simplification_ros(node_handle);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
