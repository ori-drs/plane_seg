/**
 * @file edge_detection.hpp
 * @brief The detector of edges in the robot's environment
 * @author Romeo Orsolino (rorsolino@robots.ox.ac.uk)
 * @bug No known bugs.
 * @date 22/09/2020
 * @version 1.0
 * @copyright 2020, Romeo Orsolino. BSD-3-Clause
 */
#ifndef EDGE_DETECTION_EDGE_DETECTION_H
#define EDGE_DETECTION_EDGE_DETECTION_H

#include <random>

#include <grid_map_core/grid_map_core.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <edge_detection/edge_container.h>
#include <locomotion_viewer/LocomotionViewer.hpp>


namespace edge_detection {

    class EdgeDetection {
    public:

        EdgeDetection(ros::NodeHandle & node_handle, std::string & frame_name, double & min_length, double & min_height);
        virtual ~EdgeDetection();

        /**
        * @brief update list of detected edges
        */
        bool advance(const Eigen::Vector3d & base_pose_so2, const grid_map_msgs::GridMap & message);

        /**
        * @brief plot edges to Rviz.
        */
        void plotEdges();

        /**
        * @brief get perpendicular distance between robot's odometry frame and the considered edge.
        */
        double getEdgeDistanceFromBase(const Eigen::Vector3d & base_pose_so2);

        /**
        * @brief get point in the middle of the edge.
        */
        Eigen::Vector2d getPointAlongEdgeInWorldFrame(edge_idx idx);

        /**
        * @brief get line coefficients of the edge with respect to odometry frame.
        */
        Eigen::Vector2d getEdgeDirectionInBaseFrame();

        /**
        * @brief get yaw angle of the edge with respect to odometry frame.
        */
        double getEdgeYawAngleInWorldFrame(edge_idx idx);

        /**
        * @brief get the number of detected edges.
        */
        int numberOfDetectedEdges();

        void setFakeEdges(const Eigen::Vector3d & base_pose);
        /**
        * @brief get height of the idx-th step in the array of stored edges (from closest to the robot to the furthest).
        */
        double getStepHeight(edge_idx idx);

        edge_idx findNextEdge();

    protected:

        /**
        * @brief get height of step corresponding to the edge closest to the robot.
        */
        double getNextStepHeight();


    private:

        Eigen::Vector2d convertImageToOdomFrame(const double & resolution, const Eigen::Array2i & grid_size, const int & p_x, const int & p_y);
        double computeLength(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2);
        double computeEdgeOrientation(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2);
        double computeStepHeight(const Eigen::Vector2d & p1_bf, const Eigen::Vector2d & p2_bf, double & z_coordinate);
        double computeHeight(const Eigen::Vector2d & edge_normal, const Eigen::Vector2d & point2check, double & z_coordinate);
        bool isInsideEllipse(const double & edge_yaw, const Eigen::Vector2d & ellipse_center, const Eigen::Vector2d & p, const double & d1, const double & d2);
        double GetHeight(double & x, double & y);
        bool isEdgeRedundant(const Eigen::Vector2d & p1_wf, const Eigen::Vector2d & p2_wf);
        bool isEdgeFacingRobot(const double & edge_yaw_wf, const double & robot_yaw_angle);
        void findNewEdges(const std::vector<cv::Vec4i> & lines, const Eigen::Vector3d & base_pose_so2);
        void checkExistingEdges(const Eigen::Vector3d & base_pose);
        void sortEdgesFromClosestToFurthest(const Eigen::Vector3d & base_pose);
        bool hasSimilarLineCoefficients(const EdgeContainer & existing_edge, const Eigen::Vector2d & p1, const Eigen::Vector2d & p2, const Eigen::Vector2d & base_pos);

        /**
        * @brief compute distance point to line as in https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        * @param (in) first point on the line (given in the base frame)
        * @param (in) second point on the line (given in the base frame)
        */
        double computeDistance(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2);
        double computeDistanceBtwEdgeAndBaseInWorldFrame(const Eigen::Vector2d & p1_bf, const Eigen::Vector2d & p2_bf, const Eigen::Vector2d & base_pos);
        double computeSignedDistanceBtwEdgeAndBaseInWorldFrame(const Eigen::Vector2d & p1_bf, const Eigen::Vector2d & p2_bf, const Eigen::Vector2d & base_pos);

        grid_map::GridMap gridMap_;
        double deltaFiniteDifferentiation_;
        locomotion_viewer::LocomotionViewerPtr edges_publisher_;
        ros::NodeHandle node_handle_;
        std::vector<cv::Vec4i> linesP_; // will hold the results of the detectio
        Eigen::Vector2d edge_direction_;
        Eigen::Vector2d middle_point_;
        double edge_distance_;
        double min_length_;
        double max_length_;
        double min_height_;
        double max_height_;
        double target_yaw_angle_;
        std::vector<edge_idx> orthogonal_edge_indices_;
        edge_idx closest_orthogonal_edge_index_;

        std::vector<edge_detection::EdgeContainer> edges_;

        Eigen::Vector3d base_pose_;


    }; //end class EdgeDetection

} //end namespace

#endif //EDGE_DETECTION_EDGE_DETECTION_H
