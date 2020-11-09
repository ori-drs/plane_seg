/**
 * @file edge_container.hpp
 * @brief The container for the detected edges in the robot's environment
 * @author Romeo Orsolino (rorsolino@robots.ox.ac.uk)
 * @bug No known bugs.
 * @date 22/09/2020
 * @version 1.0
 * @copyright 2020, Romeo Orsolino. BSD-3-Clause
 */
#ifndef EDGE_CONTAINER_H
#define EDGE_CONTAINER_H

#pragma once
#include <Eigen/Core>
#include <vector>

namespace edge_detection {

    using edge_idx      =  int;

    struct EdgeContainer {
        double length;
        double height;
        double yaw;
        double z;
        Eigen::Vector2d point1_wf;
        Eigen::Vector2d point2_wf;
        Eigen::Vector2d line_coeffs;
    };
} /* namespace edge_detection */

#endif //EDGE_CONTAINER_H
