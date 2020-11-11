#ifndef EDGE_DETECTION_EDGE_DETECTION_ROS_H
#define EDGE_DETECTION_EDGE_DETECTION_ROS_H

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include <Eigen/Eigen>

#include <edge_detection/edge_detection.h>
#include <edge_detection/Edge.h>
#include <edge_detection/EdgeArray.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace edge_detection {
    class EdgeDetectionRos: public EdgeDetection
    {
    public:
        EdgeDetectionRos(ros::NodeHandle & node_handle, std::string & frame_name, double min_lenght, double height);
        ~EdgeDetectionRos();

        edge_detection::EdgeArray createMessage();

        /**
        * @brief plot edges to Rviz.
        */
        void plotEdges();

    private:

        void UpdateEdges(const grid_map_msgs::GridMap& grid_map_in);

        //ROS
        ros::NodeHandle node_handle_;
        ros::Subscriber elevation_map_sub_;
        ros::Publisher edge_pub_;
        ros::Publisher edges_publisher_;
        Eigen::Vector3d robot_state_;
        int number_of_published_edges_;

    };
}

#endif //EDGE_DETECTION_EDGE_DETECTION_ROS_H
