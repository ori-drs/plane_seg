#include "edge_detection_ros/edge_detection_ros.h"

namespace edge_detection {

    EdgeDetectionRos::EdgeDetectionRos(ros::NodeHandle &node_handle, std::string & frame_name, double min_length, double min_height) :
            node_handle_(node_handle), EdgeDetection(node_handle, frame_name, min_length, min_height) {

      elevation_map_sub_ = node_handle_.subscribe("elevation_map_processing/sub_map", 1, &edge_detection::EdgeDetectionRos::UpdateEdges, this);
      anymal_state_sub_ = node_handle_.subscribe("/state_estimator/anymal_state", 1, &edge_detection::EdgeDetectionRos::ReadAnymalState, this);
      edge_pub_ = node_handle_.advertise<edge_detection::EdgeArray>("/edge_detection/edge_array", 1000);

      number_of_published_edges_ = 4;
      std::cout<<"[EdgeDetection::detectEdges] number of published edges: "<<number_of_published_edges_<<std::endl;
      //state_world_yaw_prev_ = 0.0;

    }

    EdgeDetectionRos::~EdgeDetectionRos() {
    }

    void EdgeDetectionRos::ReadAnymalState(const anymal_msgs::AnymalState & anymal_state_msg) {
      robot_state_[0] = anymal_state_msg.pose.pose.position.x;
      robot_state_[1]= anymal_state_msg.pose.pose.position.y;

      Eigen::Quaterniond state_world_q(anymal_state_msg.pose.pose.orientation.w, anymal_state_msg.pose.pose.orientation.x, anymal_state_msg.pose.pose.orientation.y, anymal_state_msg.pose.pose.orientation.z);
      Eigen::Vector3d euler = state_world_q.toRotationMatrix().eulerAngles(2, 1, 0);
      double yaw = euler[0]; //pitch = euler[1]; roll = euler[2];
      robot_state_[2] = yaw;
    }

    edge_detection::EdgeArray EdgeDetectionRos::createMessage(){
      edge_detection::EdgeArray edges_array;
      for(int j = 0; j<numberOfDetectedEdges(); j++){
        if(j<number_of_published_edges_){
          Eigen::Vector2d middle_point_wf = getPointAlongEdgeInWorldFrame(j);
          edge_detection::Edge new_edge;
          new_edge.index = j;
          geometry_msgs::Pose2D edge_pose;
          new_edge.step_height = getStepHeight(j);
          edge_pose.x = middle_point_wf[0];
          edge_pose.y = middle_point_wf[1];
          edge_pose.theta = getEdgeYawAngleInWorldFrame(j);
          new_edge.pose = edge_pose;
          edges_array.edges.push_back(new_edge);
        }
      }

      return edges_array;
    }

    void EdgeDetectionRos::UpdateEdges(const grid_map_msgs::GridMap&  grid_map_in) {
      advance(robot_state_, grid_map_in);

      edge_detection::EdgeArray edges_array = createMessage();

      edge_pub_.publish(edges_array);

      plotEdges();
    }
}

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
