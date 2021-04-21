#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "plane_seg/Tracker.hpp"
#include "plane_seg/StepCreator.hpp"
#include <geometry_msgs/PolygonStamped.h>

namespace planeseg{

struct line_strip {
    visualization_msgs::Marker centroidsMarker;
    int id;
};

class Visualizer {
public:
    Visualizer();
    ~Visualizer();

//    sensor_msgs::PointCloud2 displayCentroids(std::vector<planeseg::plane> &planes);
    visualization_msgs::Marker displayCentroid(planeseg::plane plane);
    visualization_msgs::Marker displayString(planeseg::plane plane);
    visualization_msgs::Marker displayLineStrip(planeseg::plane plane);
    visualization_msgs::Marker displayHull(planeseg::plane plane);
//    geometry_msgs::PolygonStamped displayRectangle(planeseg::contour contour);
    double getR(int id);
    double getG(int id);
    double getB(int id);

private:

    std::vector<double> colors_;
    std::vector<line_strip> lineStrips;

};

} // namespace planeseg
