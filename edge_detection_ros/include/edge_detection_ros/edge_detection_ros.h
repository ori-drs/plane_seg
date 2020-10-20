#ifndef TOWR_EDGE_DETECTION_ROS_H
#define TOWR_EDGE_DETECTION_ROS_H

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <anymal_msgs/AnymalState.h>

#include <locomotion_viewer/LocomotionViewer.hpp>
#include <Eigen/Eigen>

#include <edge_detection/edge_detection.h>
#include <edge_detection/Edge.h>
#include <edge_detection/EdgeArray.h>

namespace towr {
    class EdgeDetectionRos: public EdgeDetection
    {
    public:
        EdgeDetectionRos(ros::NodeHandle & node_handle, std::string & frame_name, double min_lenght, double height);
        ~EdgeDetectionRos();

        edge_detection::EdgeArray createMessage();

    private:

        void UpdateEdges(const grid_map_msgs::GridMap& grid_map_in);
        void ReadAnymalState(const anymal_msgs::AnymalState & anymal_state_msg);

        //ROS
        ros::NodeHandle node_handle_;
        ros::Subscriber elevation_map_sub_;
        ros::Subscriber anymal_state_sub_;
        ros::Publisher edge_pub_;
        Eigen::Vector3d robot_state_;
        int number_of_published_edges_;

    };
}

#endif //TOWR_EDGE_DETECTION_ROS_H
