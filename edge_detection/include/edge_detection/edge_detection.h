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

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <edge_detection/edge_container.h>

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

        /**
        * @brief use list of ground truth edges (for debugging)
        */
        void setFakeEdges(const Eigen::Vector3d & base_pose);

        /**
        * @brief get height of the idx-th step in the array of stored edges (from closest to the robot to the furthest).
        */
        double getStepHeight(edge_idx idx);

        /**
        * @brief go through the output of the Hough transform and select new edges
        */
        edge_idx findNextEdge();

    protected:

        /**
        * @brief get height of step corresponding to the edge closest to the robot.
        */
        double getNextStepHeight();

        std::vector<edge_detection::EdgeContainer> edges_;
        std::string frame_name_;
        edge_idx closest_orthogonal_edge_index_;


    private:

        /**
        * @brief convert the positions on the image to positions in the odom frame
        */
        Eigen::Vector2d convertImageToOdomFrame(const double & resolution, const Eigen::Array2i & grid_size, const int & p_x, const int & p_y);

        /**
        * @brief get horizontal lenght of an edge given the two corners.
        */
        double computeLength(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2);

        /**
        * @brief get yaw angle of an edge w.r.t. the x axis of the odom frame given the two corners.
        */
        double computeEdgeOrientation(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2);

        /**
        * @brief get z coordinate of uppee edge and the height of the step.
        */
        double computeStepHeight(const Eigen::Vector2d & p1_bf, const Eigen::Vector2d & p2_bf, double & z_coordinate);

        /**
        * @brief get z coordinate of a candidate edge and the height of the step.
        */
        double computeHeight(const Eigen::Vector2d & edge_normal, const Eigen::Vector2d & point2check, double & z_coordinate);

        /**
        * @brief check whether the considered point "p" is inside an ellipse centered in "ellipse_center" and oriented a "yaw", with upper and lower diagonals "d1" and "d2"
        */
        bool isInsideEllipse(const double & yaw, const Eigen::Vector2d & ellipse_center, const Eigen::Vector2d & p, const double & d1, const double & d2);

        /**
        * @brief get z coordinate on the elevation map given the (x,y) coordinates
        */
        double GetHeight(double & x, double & y);

        /**
        * @brief check if the given edge corresponds to an already existing edge
        */
        bool isEdgeRedundant(const Eigen::Vector2d & p1_wf, const Eigen::Vector2d & p2_wf);

        /**
        * @brief check if the given edge is within a desired angle range w.r.t. the robot
        */
        bool isEdgeFacingRobot(const double & edge_yaw_wf, const double & robot_yaw_angle);

        /**
        * @brief check the output of the Hough transform to see if there is any edge/line
        */
        void findNewEdges(const std::vector<cv::Vec4i> & lines, const Eigen::Vector3d & base_pose_so2);

        /**
        * @brief double check the list of existing edges to check if some parameters have changed (useful for initially occluded zones)
        */
        void checkExistingEdges(const Eigen::Vector3d & base_pose);

        /**
        * @brief sort edges according to their relative distance to the robot
        */
        void sortEdgesFromClosestToFurthest(const Eigen::Vector3d & base_pose);

        /**
        * @brief compare how similar are two given edges given their line coefficients (a*x + b*y = c)
        */
        bool hasSimilarLineCoefficients(const EdgeContainer & existing_edge, const Eigen::Vector2d & p1, const Eigen::Vector2d & p2, const Eigen::Vector2d & base_pos);

        /**
        * @brief sort edges in clock-wise order with respect to the odom frame
        */
        void clockwiseSort(EdgeContainer & edge);

        /**
        * @brief wrap angle between +- M_PI
        */
        double limitAngle(const double & yaw);

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
        ros::NodeHandle node_handle_;
        std::vector<cv::Vec4i> linesP_; // will hold the results of the detection
        Eigen::Vector2d edge_direction_;
        Eigen::Vector2d middle_point_;
        double edge_distance_;
        double min_length_;
        double max_length_;
        double min_height_;
        double max_height_;
        std::vector<edge_idx> orthogonal_edge_indices_;
        Eigen::Vector3d base_pose_;


    }; //end class EdgeDetection

} //end namespace

#endif //EDGE_DETECTION_EDGE_DETECTION_H
