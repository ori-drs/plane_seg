#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "plane_seg/ImageProcessor.hpp"
#include "plane_seg/BlockFitter.hpp"
#include "plane_seg/Tracker.hpp"

#include <grid_map_cv/grid_map_cv.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/impl/centroid.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_msgs/GridMap.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/foreach.hpp>
#include <boost/variant.hpp>

#include "plane_seg_ros/Pass.hpp"
#include "plane_seg_ros/geometry_utils.hpp"


#define foreach BOOST_FOREACH


using namespace planeseg;

Pass::Pass(ros::NodeHandle& node):
    node_(node),
    filter_chain_("grid_map::GridMap")
{

  std::string input_body_pose_topic;
  node_.getParam("input_body_pose_topic", input_body_pose_topic);
//  std::string filename;
//  node_.getParm("/rosbag_pass/filename", filename);
/*
  grid_map_sub_ = node_.subscribe("/elevation_mapping/elevation_map", 100,
                                    &Pass::elevationMapCallback, this);
  point_cloud_sub_ = node_.subscribe("/plane_seg/point_cloud_in", 100,
                                    &Pass::pointCloudCallback, this);
  pose_sub_ = node_.subscribe("/state_estimator/pose_in_odom", 100,
                                    &Pass::robotPoseCallBack, this);
*/

  received_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/received_cloud", 10);
  hull_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/hull_cloud", 10);
  hull_markers_pub_ = node_.advertise<visualization_msgs::Marker>("/plane_seg/hull_markers", 10);
  look_pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/plane_seg/look_pose", 10);
  elev_map_pub_ = node_.advertise<grid_map_msgs::GridMap>("/rooster_elevation_mapping/elevation_map", 1);
  pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/vilens/pose", 1);
  id_strings_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/plane_seg/id_strings", 10);
  centroids_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/plane_seg/centroids", 10);
  hulls_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/plane_seg/hulls", 10);
  linestrips_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/plane_seg/linestrips", 10);
  filtered_map_pub_ = node_.advertise<grid_map_msgs::GridMap>("/rooster_elevation_mapping/filtered_map", 1, true);

  last_robot_pose_ = Eigen::Isometry3d::Identity();

  tracking_ = planeseg::Tracker();
  visualizer_ = planeseg::Visualizer();
  imgprocessor_ = planeseg::ImageProcessor();
  listener_ = new tf::TransformListener();

  node_.param("input_topic", elevation_map_topic_, std::string("/rooster_elevation_mapping/elevation_map"));
  node_.param("erode_radius", erode_radius_, 0.2);
  ROS_INFO("Erode Radius [%f]", erode_radius_);
  node_.param("traversability_threshold", traversability_threshold_, 0.8);
  ROS_INFO("traversability_threshold [%f]", traversability_threshold_);
  node_.param("verbose_timer", verbose_timer_, true);

  // Setup filter chain
  if (!filter_chain_.configure("grid_map_filters", node_)){
      std::cout << "couldn't configure filter chain" << std::endl;
      return;
  }

  colors_ = {
      1, 1, 1, // 42
      255, 255, 120,
      1, 120, 1,
      225, 120, 1,
      1, 255, 1,
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
      120, 255, 255,
      255, 1, 1,
      155, 1, 120,
      120, 1, 120,
      255, 120, 1,
      1, 120, 255,
      255, 120, 120,
      1, 255, 120,
      255, 255, 1};

}

void Pass::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
  //std::cout << "got pose\n";
  tf::poseMsgToEigen(msg->pose.pose, last_robot_pose_);
}


void Pass::elevationMapCallback(const grid_map_msgs::GridMap& msg){
  std::cout << "got grid map / ev map\n";

  // convert message to GridMap, to PointCloud to LabeledCloud
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, "elevation", pointCloud);
  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::fromROSMsg(pointCloud, *inCloud);

  Eigen::Vector3f origin, lookDir;
  origin << last_robot_pose_.translation().cast<float>();
  lookDir = convertRobotPoseToSensorLookDir(last_robot_pose_);

  processCloud(inCloud, origin, lookDir);
}


// process a point cloud 
// This method is mostly for testing
// To transmit a static point cloud:
// rosrun pcl_ros pcd_to_pointcloud 06.pcd   _frame_id:=/odom /cloud_pcd:=/plane_seg/point_cloud_in
void Pass::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){

  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::fromROSMsg(*msg,*inCloud);

  Eigen::Vector3f origin, lookDir;
  origin << last_robot_pose_.translation().cast<float>();
  lookDir = convertRobotPoseToSensorLookDir(last_robot_pose_);

  processCloud(inCloud, origin, lookDir);
}


void Pass::processFromFile(int test_example){

  // to allow ros connections to register
  sleep(2);

  std::string inFile;
  std::string home_dir = ros::package::getPath("plane_seg_ros");
  Eigen::Vector3f origin, lookDir;
  if (test_example == 0){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/data/terrain/tilted-steps.pcd";
    origin <<0.248091, 0.012443, 1.806473;
    lookDir <<0.837001, 0.019831, -0.546842;
  }else if (test_example == 1){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/data/terrain/terrain_med.pcd";
    origin << -0.028862, -0.007466, 0.087855;
    lookDir << 0.999890, -0.005120, -0.013947;
  }else if (test_example == 2){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/data/terrain/terrain_close_rect.pcd";
    origin << -0.028775, -0.005776, 0.087898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 3){ // RGBD (Realsense D435) example from ANYmal
    inFile = home_dir + "/data/terrain/anymal/ori_entrance_stair_climb/06.pcd";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 4){ // Leica map
    inFile = home_dir + "/data/leica/race_arenas/RACE_crossplaneramps_sub1cm_cropped_meshlab_icp.ply";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 5){ // Leica map
    inFile = home_dir + "/data/leica/race_arenas/RACE_stepfield_sub1cm_cropped_meshlab_icp.ply";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }

  std::cout << "\nProcessing test example " << test_example << "\n";
  std::cout << inFile << "\n";

  std::size_t found_ply = inFile.find(".ply");
  std::size_t found_pcd = inFile.find(".pcd");

  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  if (found_ply!=std::string::npos){
    std::cout << "readply\n";
    pcl::io::loadPLYFile(inFile, *inCloud);
  }else if (found_pcd!=std::string::npos){
    std::cout << "readpcd\n";
    pcl::io::loadPCDFile(inFile, *inCloud);
  }else{
    std::cout << "extension not understood\n";
    return;
  }

  processCloud(inCloud, origin, lookDir);
}

void Pass::stepThroughFile(std::string filename){

    std::cout << filename <<std::endl;
    int frame = -1;
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/rooster_elevation_mapping/elevation_map"));
    topics.push_back(std::string("/vilens/pose"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view){
        grid_map_msgs::GridMap::ConstPtr s = m.instantiate<grid_map_msgs::GridMap>();

        if (s != NULL){
            std::cin.get();
            ++frame;

            std::cout << "frames = " << frame << std::endl;
//            std::cout << "received gridmap at time " << m.getTime().toNSec() << " with resolution:" <<   s->info.resolution << "and time: " << s->info.header.stamp << std::endl;
            std::cout << "rosbag time: " << s->info.header.stamp << std::endl;
            elevationMapCallback(*s);
            elev_map_pub_.publish(*s);
            std::cout << "Press [Enter] to continue to next gridmap message" << std::endl;
        }

        geometry_msgs::PoseWithCovarianceStamped::ConstPtr i = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (i !=NULL){
      //      std::cout << "position (x, y, z): " << i->pose.pose.position.x << ", " << i->pose.pose.position.y << ", "  << i->pose.pose.position.z << std::endl;
            robotPoseCallBack(i);
            pose_pub_.publish(*i);
            }
    }

    bag.close();
}


void Pass::processCloud(planeseg::LabeledCloud::Ptr& inCloud, Eigen::Vector3f origin, Eigen::Vector3f lookDir){

  std::cout << "entered processCloud" << std::endl;
  planeseg::BlockFitter fitter;
  fitter.setSensorPose(origin, lookDir);
  fitter.setCloud(inCloud);
  fitter.setDebug(false); // MFALLON modification
  fitter.setRemoveGround(false); // MFALLON modification from default

  // this was 5 for LIDAR. changing to 10 really improved elevation map segmentation
  // I think its because the RGB-D map can be curved
  fitter.setMaxAngleOfPlaneSegmenter(10);

  result_ = fitter.go();
  tracking_.reset();
  tracking_.convertResult(result_);
//  tracking_.printIds();

  Eigen::Vector3f rz = lookDir;
  Eigen::Vector3f rx = rz.cross(Eigen::Vector3f::UnitZ());
  Eigen::Vector3f ry = rz.cross(rx);
  Eigen::Matrix3f rotation;
  rotation.col(0) = rx.normalized();
  rotation.col(1) = ry.normalized();
  rotation.col(2) = rz.normalized();
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.linear() = rotation;
  pose.translation() = origin;
  Eigen::Isometry3d pose_d = pose.cast<double>();

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time(0, 0);
  msg.header.frame_id = "odom";
  tf::poseEigenToMsg(pose_d, msg.pose);
  look_pose_pub_.publish(msg);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*inCloud, output);
  output.header.stamp = ros::Time(0, 0);
  output.header.frame_id = "odom";
  received_cloud_pub_.publish(output);

  //printResultAsJson();
  publishResult();
}


void Pass::printResultAsJson(){
  std::string json;

  for (int i = 0; i < (int)result_.mBlocks.size(); ++i) {
    const auto& block = result_.mBlocks[i];
    std::string dimensionString = vecToStr(block.mSize);
    std::string positionString = vecToStr(block.mPose.translation());
    std::string quaternionString = rotToStr(block.mPose.rotation());
    Eigen::Vector3f color(0.5, 0.4, 0.5);
    std::string colorString = vecToStr(color);
    float alpha = 1.0;
    std::string uuid = "0_" + std::to_string(i+1);
    
    json += "    \"" + uuid + "\": {\n";
    json += "      \"classname\": \"BoxAffordanceItem\",\n";
    json += "      \"pose\": [[" + positionString + "], [" +
      quaternionString + "]],\n";
    json += "      \"uuid\": \"" + uuid + "\",\n";
    json += "      \"Dimensions\": [" + dimensionString + "],\n";
    json += "      \"Color\": [" + colorString + "],\n";
    json += "      \"Alpha\": " + std::to_string(alpha) + ",\n";
    json += "      \"Name\": \" mNamePrefix " +
      std::to_string(i) + "\"\n";
    json += "    },\n";

  }

  std::cout << json << "\n";
}


void Pass::publishResult(){
  // convert result to a vector of point clouds
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs;
  for (size_t i=0; i<result_.mBlocks.size(); ++i){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const auto& block = result_.mBlocks[i];
    for (size_t j =0; j < block.mHull.size(); ++j){
      pcl::PointXYZ pt;
      pt.x =block.mHull[j](0);
      pt.y =block.mHull[j](1);
      pt.z =block.mHull[j](2);
      cloud.points.push_back(pt);
    }
    cloud.height = cloud.points.size();
    cloud.width = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    cloud_ptr = cloud.makeShared();
    cloud_ptrs.push_back(cloud_ptr);
  }

  publishIdsAsStrings();
  publishCentroidsAsSpheres();
  publishHullsAsMarkers();
  publishLineStrips();
  publishHullsAsCloud(cloud_ptrs, 0, 0);

  //pcl::PCDWriter pcd_writer_;
  //pcd_writer_.write<pcl::PointXYZ> ("/home/mfallon/out.pcd", cloud, false);
  //std::cout << "blocks: " << result_.mBlocks.size() << " blocks\n";
  //std::cout << "cloud: " << cloud.points.size() << " pts\n";
}

// combine the individual clouds into one, with a different each
void Pass::publishHullsAsCloud(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs){


  pcl::PointCloud<pcl::PointXYZRGB> combined_cloud;
  for (size_t i=0; i<cloud_ptrs.size(); ++i){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_ptrs[i], *cloud_rgb);

    int nColor = i % (colors_.size()/3);
    double r = colors_[nColor*3]*255.0;
    double g = colors_[nColor*3+1]*255.0;
    double b = colors_[nColor*3+2]*255.0;
    for (size_t j = 0; j < cloud_rgb->points.size (); j++){
        cloud_rgb->points[j].r = r;
        cloud_rgb->points[j].g = g;
        cloud_rgb->points[j].b = b;
    }
    combined_cloud += *cloud_rgb;
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(combined_cloud, output);

  output.header.stamp = ros::Time(secs, nsecs);
  output.header.frame_id = "odom";
  hull_cloud_pub_.publish(output);

}


void Pass::publishHullsAsMarkersOLD(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs){
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::Marker marker;
  std::string frameID;

  // define markers
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time(secs, nsecs);
  marker.ns = "hull lines";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST; //visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;

  for (size_t i = 0; i < cloud_ptrs.size (); i++){

    int nColor = i % (colors_.size()/3);
    double r = colors_[nColor*3]*255.0;
    double g = colors_[nColor*3+1]*255.0;
    double b = colors_[nColor*3+2]*255.0;

    for (size_t j = 1; j < cloud_ptrs[i]->points.size (); j++){
      point.x = cloud_ptrs[i]->points[j-1].x;
      point.y = cloud_ptrs[i]->points[j-1].y;
      point.z = cloud_ptrs[i]->points[j-1].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);

      //
      point.x = cloud_ptrs[i]->points[j].x;
      point.y = cloud_ptrs[i]->points[j].y;
      point.z = cloud_ptrs[i]->points[j].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);
    }

    // start to end line:
    point.x = cloud_ptrs[i]->points[0].x;
    point.y = cloud_ptrs[i]->points[0].y;
    point.z = cloud_ptrs[i]->points[0].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);

    point.x = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].x;
    point.y = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].y;
    point.z = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);
  }
  marker.frame_locked = true;
//  hull_markers_pub_.publish(marker);
}

void Pass::publishIdsAsStrings(){
    visualization_msgs::MarkerArray strings_array;
    for (size_t i = 0; i < tracking_.newStairs.size(); ++i){
        visualization_msgs::Marker id_marker;
        id_marker = visualizer_.displayString(tracking_.newStairs[i]);
        id_marker.frame_locked = true;
        strings_array.markers.push_back(id_marker);
    }
    id_strings_pub_.publish(strings_array);
}

void Pass::publishCentroidsAsSpheres(){
    visualization_msgs::MarkerArray centroids_array;
    for (size_t i = 0; i < tracking_.newStairs.size(); ++i){
        visualization_msgs::Marker centroid_marker;
        centroid_marker = visualizer_.displayCentroid(tracking_.newStairs[i]);
        centroid_marker.frame_locked = true;
        centroids_array.markers.push_back(centroid_marker);
    }
    centroids_pub_.publish(centroids_array);
}

void Pass::publishHullsAsMarkers(){

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
    for (size_t k = 0; k < tracking_.newStairs.size(); ++k){
        clouds.push_back(tracking_.newStairs[k].cloud);
    }

    std::vector<int> ids;
    for (size_t m = 0; m < tracking_.newStairs.size(); ++m){
        ids.push_back(tracking_.newStairs[m].id);
    }

  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::Marker hullMarker;
  std::string frameID;

  // define markers
  hullMarker.header.frame_id = "odom";
  hullMarker.header.stamp = ros::Time(0, 0);
  hullMarker.ns = "hull lines";
  hullMarker.id = 0;
  hullMarker.type = visualization_msgs::Marker::LINE_LIST; //visualization_msgs::Marker::POINTS;
  hullMarker.action = visualization_msgs::Marker::ADD;
  hullMarker.pose.position.x = 0;
  hullMarker.pose.position.y = 0;
  hullMarker.pose.position.z = 0;
  hullMarker.pose.orientation.x = 0.0;
  hullMarker.pose.orientation.y = 0.0;
  hullMarker.pose.orientation.z = 0.0;
  hullMarker.pose.orientation.w = 1.0;
  hullMarker.scale.x = 0.03;
  hullMarker.scale.y = 0.03;
  hullMarker.scale.z = 0.03;
  hullMarker.color.a = 1.0;

  for (size_t i = 0; i < clouds.size (); i++){

      double r = visualizer_.getR(ids[i]);
      double g = visualizer_.getG(ids[i]);
      double b = visualizer_.getB(ids[i]);

    for (size_t j = 1; j < clouds[i].points.size (); j++){
      point.x = clouds[i].points[j-1].x;
      point.y = clouds[i].points[j-1].y;
      point.z = clouds[i].points[j-1].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      hullMarker.colors.push_back(point_color);
      hullMarker.points.push_back(point);

      //
      point.x = clouds[i].points[j].x;
      point.y = clouds[i].points[j].y;
      point.z = clouds[i].points[j].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      hullMarker.colors.push_back(point_color);
      hullMarker.points.push_back(point);
    }

    // start to end line:
    point.x = clouds[i].points[0].x;
    point.y = clouds[i].points[0].y;
    point.z = clouds[i].points[0].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    hullMarker.colors.push_back(point_color);
    hullMarker.points.push_back(point);

    point.x = clouds[i].points[ clouds[i].points.size()-1 ].x;
    point.y = clouds[i].points[ clouds[i].points.size()-1 ].y;
    point.z = clouds[i].points[ clouds[i].points.size()-1 ].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    hullMarker.colors.push_back(point_color);
    hullMarker.points.push_back(point);
  }
  hullMarker.frame_locked = true;
  hull_markers_pub_.publish(hullMarker);
}


void Pass::publishLineStrips(){
    visualization_msgs::MarkerArray linestrips_array;
    for (size_t i = 0; i < tracking_.newStairs.size(); ++i){
        visualization_msgs::Marker linestrip_marker;
        linestrip_marker = visualizer_.displayLineStrip(tracking_.newStairs[i]);
        linestrip_marker.frame_locked = true;
        linestrips_array.markers.push_back(linestrip_marker);
    }
    linestrips_pub_.publish(linestrips_array);
}


void Pass::extractNthCloud(std::string filename, int n){

    std::cout << filename <<std::endl;
    int frame = -1;
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/rooster_elevation_mapping/elevation_map"));
    topics.push_back(std::string("/vilens/pose"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view){
        grid_map_msgs::GridMap::ConstPtr s = m.instantiate<grid_map_msgs::GridMap>();

 //       std::cout << " frame = " << frame << std::endl;

        if (s != NULL){
            ++frame;
            if (frame == n){
                std::cin.get();
                grid_map_msgs::GridMap n;
                n = gridMapCallback(*s);
                elevationMapCallback(n);
//                elev_map_pub_.publish(*s);
//                imageProcessingCallback(*s);
            }
        }

        geometry_msgs::PoseWithCovarianceStamped::ConstPtr i = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (i !=NULL){
            if (frame == n){
                robotPoseCallBack(i);
                pose_pub_.publish(*i);
            }
        }
    }


bag.close();
}

void Pass::imageProcessingCallback(const grid_map_msgs::GridMap &msg){
    grid_map::GridMap gridmap;
    grid_map::GridMapRosConverter::fromMessage(msg, gridmap);
    grid_map::GridMapRosConverter::toCvImage(gridmap, "elevation", sensor_msgs::image_encodings::MONO8, imgprocessor_.original_img_);

//                cv_bridge::CvImage img_rgb;
//                cv::applyColorMap(image.image, img_rgb.image, cv::COLORMAP_JET);

    imgprocessor_.process();

}

void Pass::tic(){
  last_time_ = std::chrono::high_resolution_clock::now();
}

std::chrono::duration<double> Pass::toc(){
  auto now_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - last_time_);
  last_time_ = now_time;
  // std::cout << elapsedTime.count() << "ms elapsed" << std::endl;
  return elapsed_time;
}

grid_map_msgs::GridMap Pass::gridMapCallback(const grid_map_msgs::GridMap& msg){
  tic();

  // Convert message to map.
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(msg, input_map);

  // Apply filter chain.
  grid_map::GridMap output_map;
  if (!filter_chain_.update(input_map, output_map)) {
    std::cout << "couldn't update the grid map filter chain" << std::endl;
    grid_map_msgs::GridMap failmessage;
    grid_map::GridMapRosConverter::toMessage(input_map, failmessage);
    return failmessage;
  }

  if (verbose_timer_) {
    std::cout << toc().count() << " ms: filter chain\n";
  }

  tic();

  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  tf::StampedTransform transform;
  try {
    listener_.waitForTransform("odom", "base", ros::Time(0), ros::Duration(10.0) );
    listener_.lookupTransform("odom", "base", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

  tf::transformTFToEigen(transform, pose_robot);

  // Threshold traversability conservatively
  grid_map::Matrix& data = output_map["traversability_clean"];
  for (grid_map::GridMapIterator iterator(output_map); !iterator.isPastEnd(); ++iterator){
    const grid_map::Index index(*iterator);

    // make cells very near robot traversable
    grid_map::Position pos_cell;
    output_map.getPosition(index, pos_cell);
    grid_map::Position pos_robot(pose_robot.translation().head(2));
    double dist = (pos_robot - pos_cell).norm();
    if (dist <1.0){
      data(index(0), index(1)) = 1.0;
    }

    if (data(index(0), index(1)) < traversability_threshold_){
      data(index(0), index(1)) =0.0;
    }else{
      data(index(0), index(1)) =1.0;
    }
  }

  // Erode the traversable image
  cv::Mat original_image, erode_image;
  grid_map::GridMapCvConverter::toImage<unsigned short, 1>(output_map, "traversability_clean", CV_16UC1, 0.0, 0.3, original_image);
  //std::cout << "write original\n";
  //cv::imwrite( "original_image.png", original_image);

  int erode_size =  int(floor(erode_radius_ / msg.info.resolution)); // was 25 for conservative
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                       cv::Size(2*erode_size + 1, 2*erode_size + 1),
                                       cv::Point(erode_size, erode_size));
  /// Apply the dilation operation
  cv::erode(original_image, erode_image, element);
  //std::cout << "write eroded\n";
  //cv::imwrite( "erode_image.png", erode_image);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(erode_image, "traversability_clean_eroded", output_map, 0.0, 0.3);

  if (verbose_timer_) {
    std::cout << toc().count() << " ms: traversability edition (i.e. erosion)\n";
  }

  // Publish filtered output grid map.
  grid_map_msgs::GridMap output_msg;
  grid_map::GridMapRosConverter::toMessage(output_map, output_msg);
  filtered_map_pub_.publish(output_msg);

  std::vector<std::string> outlayers, inlayers;
  outlayers = output_map.getLayers();
  inlayers = input_map.getLayers();
  std::cout << "Input map layers: " << std::endl;
  for (size_t i=0; i<inlayers.size(); ++i){
      std::cout << inlayers[i] << std::endl;
  }
  std::cout << "Output map layers: " << std::endl;
  for (size_t i=0; i<outlayers.size(); ++i){
      std::cout << outlayers[i] << std::endl;
  }


  return output_msg;

}
