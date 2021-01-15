#include <ros/ros.h>
#include <ros/console.h>
#include "plane_seg_ros/Visualizer.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "plane_seg/Tracker.hpp"
#include <ros/time.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace planeseg {

Visualizer::Visualizer(){
    colors_ = {
        1, 1, 1, // 42
        255, 255, 120,
        1, 120, 1,
        1, 225, 1,
        120, 255, 1,
        1, 255, 255,
        120, 1, 1,
        255, 120, 255,
        120, 1, 255,
        1, 1, 120,
        255, 255, 255,
        120, 120, 1,
        120, 120, 120,
        1, 1, 255,
        255, 1, 255,
        120, 120, 255,
        120, 255, 120,
        1, 120, 120,
        1, 1, 255,
        255, 1, 1,
        155, 1, 120,
        120, 1, 120,
        255, 120, 1,
        1, 120, 255,
        255, 120, 120,
        1, 255, 120,
        255, 255, 1};

}

Visualizer::~Visualizer(){}


visualization_msgs::Marker Visualizer::displayCentroid(planeseg::plane plane){

    int id = plane.id;
    pcl::PointXYZ centroid = plane.centroid;

    // convert centroid to geometry_msgs/Point
    geometry_msgs::Point centroidGM;
    centroidGM.x = centroid.x;
    centroidGM.y = centroid.y;
    centroidGM.z = centroid.z;

    visualization_msgs::Marker centroidMarker;
    centroidMarker.type = visualization_msgs::Marker::SPHERE;
    centroidMarker.header.frame_id = "odom";
    centroidMarker.header.stamp = ros::Time();
    centroidMarker.ns = "centroids";
    centroidMarker.id = id;
    centroidMarker.scale.x = 0.05;
    centroidMarker.scale.y = 0.05;
    centroidMarker.scale.z = 0.05;
    centroidMarker.color.r = getR(id);
    centroidMarker.color.g = getG(id);
    centroidMarker.color.b = getB(id);
    centroidMarker.color.a = 1;
    centroidMarker.pose.orientation.w = 1.0;
    centroidMarker.pose.position.x = centroidGM.x;
    centroidMarker.pose.position.y = centroidGM.y;
    centroidMarker.pose.position.z = centroidGM.z;

    return centroidMarker;
}

visualization_msgs::Marker Visualizer::displayLineStrip(int id, pcl::PointXYZ newCentroid){

    bool point_added;
    point_added = false;

    // convert newCentroid to geometry_msgs/Point
    geometry_msgs::Point newCentroidgm;
    newCentroidgm.x = newCentroid.x;
    newCentroidgm.y = newCentroid.y;
    newCentroidgm.z = newCentroid.z;

    // create visualization markers
    visualization_msgs::Marker centroidMarker;
    visualization_msgs::Marker lineStripMarker;
    visualization_msgs::Marker newPlaneMarker;

    // create string of centroid_ID
    std::ostringstream s;
    s << "centroid_" << id;
    std::string centroid_id;
    centroid_id = s.str();

    // first we must find out if the newCentroid should be added to an already existing linestrip
    for (unsigned i = 0; i < lineStrips.size() && point_added == false; i++){
        if (id == lineStrips[i].id){
            lineStrips[i].centroidsMarker.points.push_back(newCentroidgm);
            centroidMarker = lineStrips[i].centroidsMarker;
            centroidMarker.header.frame_id = "odom";
            centroidMarker.header.stamp = ros::Time::now();
            centroidMarker.ns = "linestrips";
            centroidMarker.id = id;

            // declare outgoing marker as the linestrip with matching ID
            lineStripMarker = centroidMarker;
            point_added = true;
        }
    }
    // if it is the centroid of a new plane, we must create a new linestrip for it
    if (point_added == false){

        // publish a point for the first marker of the new ID
        newPlaneMarker.type = visualization_msgs::Marker::POINTS;
        newPlaneMarker.header.frame_id = "odom";
        newPlaneMarker.header.stamp = ros::Time::now();
        newPlaneMarker.ns = "linestrips";
        newPlaneMarker.id = id;
        newPlaneMarker.scale.x = 0.1;
        newPlaneMarker.scale.y = 0.1;
        newPlaneMarker.color.r = getR(id);
        newPlaneMarker.color.g = getG(id);
        newPlaneMarker.color.b = getB(id);
        newPlaneMarker.color.a = 1;
        newPlaneMarker.pose.orientation.w = 1.0;
        newPlaneMarker.points.push_back(newCentroidgm);

        // create a new linestrip for the new ID
        centroidMarker.type = visualization_msgs::Marker::LINE_STRIP;
        centroidMarker.scale.x = 0.1;
        centroidMarker.color.r = getR(id);
        centroidMarker.color.g = getG(id);
        centroidMarker.color.b = getB(id);
        centroidMarker.color.a = 1;
        centroidMarker.pose.orientation.w = 1.0;
        centroidMarker.points.push_back(newCentroidgm);
        line_strip newLineStrip;
        newLineStrip.centroidsMarker = centroidMarker;
        newLineStrip.id = id;
        lineStrips.push_back(newLineStrip);

        // declare the outgoing marker as the new marker
        lineStripMarker = newPlaneMarker;
    }

    return lineStripMarker;
}

visualization_msgs::Marker Visualizer::displayString(planeseg::plane plane){

    int id = plane.id;
    pcl::PointXYZ point = plane.centroid;

    // convert pcl::PointXYZ into geometry_msgs::Point
    geometry_msgs::Point pointGM;
    pointGM.x = point.x;
    pointGM.y = point.y;
    pointGM.z = point.z;

    visualization_msgs::Marker stringMarker;
    stringMarker.header.frame_id = "odom";
    stringMarker.header.stamp = ros::Time();
    stringMarker.ns = "strings";
    stringMarker.id = id;
    stringMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    stringMarker.action = visualization_msgs::Marker::ADD;
    stringMarker.pose.position.x = pointGM.x;
    stringMarker.pose.position.y = pointGM.y;
    stringMarker.pose.position.z = pointGM.z;
    stringMarker.scale.x = 0.2;
    stringMarker.scale.y = 0.2;
    stringMarker.scale.z = 0.2;
    stringMarker.color.a = 1;
    stringMarker.color.r = 1;
    stringMarker.color.g = 1;
    stringMarker.color.b = 1;

    std::string id_string = std::to_string(id);
    stringMarker.text = id_string;

    return stringMarker;
}

visualization_msgs::Marker Visualizer::displayHull(planeseg::plane plane){

    geometry_msgs::Point pointGM;
    std_msgs::ColorRGBA point_color;
    visualization_msgs::Marker hullMarker;
    std::string FrameID;

    hullMarker.header.frame_id = "odom";
    hullMarker.header.stamp = ros::Time();
    hullMarker.ns = "hull lines";
    hullMarker.id = plane.id;
    hullMarker.type = visualization_msgs::Marker::LINE_STRIP;
    hullMarker.action = visualization_msgs::Marker::ADD;
    hullMarker.pose.orientation.w = 1.0;
    hullMarker.scale.x = 0.03;
    hullMarker.color.r = getR(plane.id);
    hullMarker.color.g = getG(plane.id);
    hullMarker.color.b = getB(plane.id);
    hullMarker.color.a = 1;

    for (size_t i = 0; i < plane.cloud.size(); ++i){
        pointGM.x = plane.cloud[i].x;
        pointGM.y = plane.cloud[i].y;
        pointGM.z = plane.cloud[i].z;
        hullMarker.points.push_back(pointGM);
    }

    return hullMarker;
}

double Visualizer::getR(int id){
    double j;
    j = id % (colors_.size()/3);
    return colors_[3*j];
}

double Visualizer::getG(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j+1];
}

double Visualizer::getB(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j+2];
}
} // namespace planeseg
