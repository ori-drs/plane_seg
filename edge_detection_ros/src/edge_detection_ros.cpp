#include "edge_detection_ros/edge_detection_ros.h"

namespace edge_detection {

    EdgeDetectionRos::EdgeDetectionRos(ros::NodeHandle &node_handle, std::string & frame_name, double min_length, double min_height) :
            node_handle_(node_handle), EdgeDetection(node_handle, frame_name, min_length, min_height) {

      elevation_map_sub_ = node_handle_.subscribe("elevation_mapping/elevation_map", 1, &edge_detection::EdgeDetectionRos::UpdateEdges, this);
      edge_pub_ = node_handle_.advertise<edge_detection::EdgeArray>("/edge_detection/edge_array", 1000);
      edges_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>( "/edge_detection/detected_edges", 0 );

      number_of_published_edges_ = 6;
      std::cout<<"[EdgeDetection::detectEdges] size of edge array: "<<number_of_published_edges_<<std::endl;
    }

    EdgeDetectionRos::~EdgeDetectionRos() {
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
        q = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(edges_.at(i).yaw + M_PI/2.0, Eigen::Vector3d::UnitY());

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
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = edges_.at(i).length;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker_array.markers.push_back(marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_name_;
        text_marker.header.stamp = ros::Time();
        text_marker.ns = "edge_detection";
        text_marker.id = edges_.size() + i;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = edges_.at(i).point1_wf[0];
        text_marker.pose.position.y = edges_.at(i).point1_wf[1];
        text_marker.pose.position.z = edges_.at(i).z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        text_marker.text = std::to_string(i);
        text_marker.scale.x = 0.1;
        text_marker.scale.y = 0.1;
        text_marker.scale.z = 0.1;
        text_marker.color.a = 1.0; // Don't forget to set the alpha!
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        marker_array.markers.push_back(text_marker);

      }

      edges_publisher_.publish(marker_array);

    }
}