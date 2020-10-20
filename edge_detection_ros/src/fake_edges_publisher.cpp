#include "edge_detection/edge_detection.h"
#include <edge_detection/Edge.h>
#include <edge_detection/EdgeArray.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/RobotStateCartesian.h>

Eigen::Vector3d robot_pose_ = Eigen::Vector3d::Zero();

void ReadXppState(const xpp_msgs::RobotStateCartesian & xpp_state_msg) {
  robot_pose_[0] = xpp_state_msg.base.pose.position.x;
  robot_pose_[1]= xpp_state_msg.base.pose.position.y;
  
  Eigen::Quaterniond state_world_q(xpp_state_msg.base.pose.orientation.w, xpp_state_msg.base.pose.orientation.x, xpp_state_msg.base.pose.orientation.y, xpp_state_msg.base.pose.orientation.z);
  Eigen::Vector3d euler = state_world_q.toRotationMatrix().eulerAngles(2, 1, 0);
  double yaw = euler[0]; //pitch = euler[1]; roll = euler[2];
  robot_pose_[2] = yaw;

  std::cout<<"[ReadXppState] robot_pose_: "<<robot_pose_.transpose()<<std::endl;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fake_edges_pub");
  ros::NodeHandle node_handle;
  double min_lenght = 0.8;
  double min_height = 0.3;
  std::string frame_name = "world";
  edge_detection::EdgeDetection edge_det(node_handle, frame_name, min_lenght, min_height);

  double number_of_published_edges = 4;
  ros::Publisher fake_edges_pub = node_handle.advertise<edge_detection::EdgeArray>("/edge_detection/edge_array", 1000);
  ros::Subscriber xpp_state_sub_ = node_handle.subscribe("/xpp/state_des", 1, &ReadXppState);

  ros::Rate loop_rate(2.0);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

    edge_det.setFakeEdges(robot_pose_);

    edge_detection::edge_idx next_edge_id = edge_det.findNextEdge();
    edge_detection::EdgeArray edges_array;
    int last_edge_id = edge_det.numberOfDetectedEdges() - next_edge_id - number_of_published_edges >= 0? next_edge_id + number_of_published_edges: edge_det.numberOfDetectedEdges();
    for(int j = next_edge_id; j<edge_det.numberOfDetectedEdges(); j++){
      if(j<last_edge_id){
        Eigen::Vector2d middle_point_wf = edge_det.getPointAlongEdgeInWorldFrame(j);
        edge_detection::Edge new_edge;
        new_edge.index = j;
        geometry_msgs::Pose2D edge_pose;
        new_edge.step_height = edge_det.getStepHeight(j);
        edge_pose.x = middle_point_wf[0];
        edge_pose.y = middle_point_wf[1];
        edge_pose.theta = edge_det.getEdgeYawAngleInWorldFrame(j);
        new_edge.pose = edge_pose;
        edges_array.edges.push_back(new_edge);
      }
    }
    fake_edges_pub.publish(edges_array);

    edge_det.plotEdges();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
