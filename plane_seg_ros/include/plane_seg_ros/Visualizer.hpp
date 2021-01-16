#ifndef _planeseg_Visualizer_hpp_
#define _planeseg_Visualizer_hpp_
#endif

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "plane_seg/Tracker.hpp"

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
    double getR(int id);
    double getG(int id);
    double getB(int id);

private:

    std::vector<double> colors_;
    std::vector<line_strip> lineStrips;

};

} // namespace planeseg
