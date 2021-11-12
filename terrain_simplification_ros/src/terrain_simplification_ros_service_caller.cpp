#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "terrain_simplification_service_caller");
  ros::NodeHandle nh;
  ros::ServiceClient ros_client_pub
      = nh.serviceClient<std_srvs::Empty>("/terrain_simplification/pub");
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;

  double rate = 10.;
  while (ros::ok())
  {
    nh.param("/terrain_simplification_ros_service_caller/rate", rate, 10.);
    ros_client_pub.call(req,res);
    ros::Rate r(rate);
    r.sleep();
  }
  return 0;
}
