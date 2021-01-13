#include <ros/ros.h>
#include <ros/console.h>
#include "plane_seg_ros/Visualizer.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "plane_seg/Tracker.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

Visualizer::Visualizer(ros::Publisher centroids_pub, ros:: Publisher linestrip_pub)//:
 //   centroids_pub_(centroids_pub),
 //   linestrip_pub_(linestrip_pub)
{
    colors_ = {
         51/255.0, 160/255.0, 44/255.0,  //0
         166/255.0, 206/255.0, 227/255.0,
         178/255.0, 223/255.0, 138/255.0,//6
         31/255.0, 120/255.0, 180/255.0,
         251/255.0, 154/255.0, 153/255.0,// 12
         227/255.0, 26/255.0, 28/255.0,
         253/255.0, 191/255.0, 111/255.0,// 18
         106/255.0, 61/255.0, 154/255.0,
         255/255.0, 127/255.0, 0/255.0, // 24
         202/255.0, 178/255.0, 214/255.0,
         1.0, 0.0, 0.0, // red // 30
         0.0, 1.0, 0.0, // green
         0.0, 0.0, 1.0, // blue// 36
         1.0, 1.0, 0.0,
         1.0, 0.0, 1.0, // 42
         0.0, 1.0, 1.0,
         0.5, 1.0, 0.0,
         1.0, 0.5, 0.0,
         0.5, 0.0, 1.0,
         1.0, 0.0, 0.5,
         0.0, 0.5, 1.0,
         0.0, 1.0, 0.5,
         1.0, 0.5, 0.5,
         0.5, 1.0, 0.5,
         0.5, 0.5, 1.0,
         0.5, 0.5, 1.0,
         0.5, 1.0, 0.5,
         0.5, 0.5, 1.0};

}

/* NOT FINISHED YET
void Visualizer::publish(std::vector<plane> &planes){

    // publish coloured centroids
    sensor_msgs::PointCloud2 displayCentroidsMsg(displayCentroids(planes));
    centroids_pub_.publish(displayCentroidsMsg);

    // publish line segments
    visualization_msgs::Marker displayLineStripMsg(displayCentroidsMsg);
}
*/

sensor_msgs::PointCloud2 Visualizer::displayCentroids(std::vector<planeseg::plane> &planes){
    int id;
    pcl::PointXYZRGB centroid;
    pcl::PointCloud<pcl::PointXYZRGB> display_centroids;

    for (unsigned i = 0; i < planes.size(); i++){
        id = planes[i].id;
        centroid.r = getR(id);
        centroid.g = getG(id);
        centroid.b = getB(id);
        centroid.x = planes[i].centroid.x;
        centroid.y = planes[i].centroid.y;
        centroid.z = planes[i].centroid.z;
        display_centroids.push_back(centroid);
    }
    sensor_msgs::PointCloud2 displayCentroidsMsg;
    pcl::toROSMsg(display_centroids, displayCentroidsMsg);
    return displayCentroidsMsg;
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

visualization_msgs::Marker Visualizer::displayString(int id, geometry_msgs::Point point){
    visualization_msgs::Marker stringMarker;
    stringMarker.header.frame_id = "odom";
    stringMarker.header.stamp = ros::Time();
    stringMarker.ns = "strings";
    stringMarker.id = id;
    stringMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    stringMarker.action = visualization_msgs::Marker::ADD;
    stringMarker.pose.position.x = point.x;
    stringMarker.pose.position.y = point.y;
    stringMarker.pose.position.z = point.z;
    stringMarker.scale.x = 1;
    stringMarker.color.a = 1;
    stringMarker.color.r = 1;
    stringMarker.color.g = 1;
    stringMarker.color.b = 1;

    std::string id_string = std::to_string(id);
    stringMarker.text = id_string;

    return stringMarker;
}

unsigned Visualizer::getR(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j]*255;
}

unsigned Visualizer::getG(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j+1]*255;
}

unsigned Visualizer::getB(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j+2]*255;
}
