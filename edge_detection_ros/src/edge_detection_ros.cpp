#include "edge_detection_ros/edge_detection_ros.h"

namespace edge_detection {

    EdgeDetectionRos::EdgeDetectionRos(ros::NodeHandle &node_handle, std::string & frame_name, double min_length, double min_height) :
            node_handle_(node_handle), EdgeDetection(node_handle, frame_name, min_length, min_height) {

      elevation_map_sub_ = node_handle_.subscribe("elevation_map_processing/sub_map", 1, &edge_detection::EdgeDetectionRos::UpdateEdges, this);
      anymal_state_sub_ = node_handle_.subscribe("/state_estimator/anymal_state", 1, &edge_detection::EdgeDetectionRos::ReadAnymalState, this);
      edge_pub_ = node_handle_.advertise<edge_detection::EdgeArray>("/edge_detection/edge_array", 1000);
      edges_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>( "/edge_detection/detected_edges", 0 );

      number_of_published_edges_ = 6;
      std::cout<<"[EdgeDetection::detectEdges] number of published edges: "<<number_of_published_edges_<<std::endl;
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

    void EdgeDetectionRos::plotEdges(){

      visualization_msgs::MarkerArray marker_array;
      for( size_t i = 0; i < edges_.size(); i++ )
      {
        visualization_msgs::Marker marker;
        Eigen::Vector3d edge_pose;
        edge_pose(0) = (edges_.at(i).point1_wf[0]+edges_.at(i).point2_wf[0])/2.0;
        edge_pose(1) = (edges_.at(i).point1_wf[1]+edges_.at(i).point2_wf[1])/2.0;
        edge_pose(2) = edges_.at(i).z;

        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(edges_.at(i).yaw, Eigen::Vector3d::UnitZ());

        if(closest_orthogonal_edge_index_==i){
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
        }else{
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
        }

        marker.header.frame_id = frame_name_;
        marker.header.stamp = ros::Time();
        marker.ns = "edge_detection";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = edge_pose(0);
        marker.pose.position.y = edge_pose(1);
        marker.pose.position.z = edge_pose(2);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker_array.markers.push_back(marker);
      }

      edges_publisher_.publish(marker_array);

    }
}