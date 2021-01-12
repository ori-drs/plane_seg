#ifndef _planeseg_Visualizer_hpp_
#define _planeseg_Visualizer_hpp_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "plane_seg/Tracker.hpp"

struct line_strip {
    visualization_msgs::Marker centroidsMarker;
    int id;
}

class Visualizer{
public:
    Visualizer();
    explicit Visualizer(ros::Publisher centroids_pub, ros::Publisher linestrip_pub);
    ~Visualizer();

private:
    sensor_msgs::PointCloud2 displayCentroids(std::vector<planeseg::plane> &planes);
    visualization_msgs::Marker displayString(int id, std::string string_, geometry_msgs::Point point_);
    visualization_msgs::Marker displayLineStrip(int id, pcl::PointXYZ newCentroid);
    std::vector<double> colors_;
    std::vector<line_strip> lineStrips;
    unsigned getR(int id);
    unsigned getG(int id);
    unsigned getB(int id);
}
