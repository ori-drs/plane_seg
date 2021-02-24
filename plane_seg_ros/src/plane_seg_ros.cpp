#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <numeric>

#include "plane_seg/ImageProcessor.hpp"
#include "plane_seg/BlockFitter.hpp"
#include "plane_seg/Tracker.hpp"
//#include "plane_seg/StepCreator.hpp"

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/impl/centroid.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/grid_map_core.hpp>
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
  if(!node_.getParam("input_body_pose_topic", input_body_pose_topic)){
   ROS_WARN_STREAM("Couldn't get parameter: input_body_pose_topic");
  }

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
  rectangles_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/opencv/rectangles", 100);
  test_pub_ = node_.advertise<visualization_msgs::Marker>("/opencv/test", 10);

  last_robot_pose_ = Eigen::Isometry3d::Identity();

  tracking_ = planeseg::Tracker();
  visualizer_ = planeseg::Visualizer();
  imgprocessor_ = planeseg::ImageProcessor();
  stepcreator_ = planeseg::StepCreator();

  if(!node_.param("algorithm", algorithm_, std::string("A"))){
    ROS_WARN_STREAM("Couldn't get parameter: algorithm");
  }
  ROS_INFO("ALGORITHM %s", algorithm_.c_str());

  if(!node_.getParam("input_topic", elevation_map_topic_)){
    ROS_WARN_STREAM("Couldn't get parameter: input_topic");
  }
  if(!node_.getParam("erode_radius", erode_radius_)){
    ROS_WARN_STREAM("Couldn't get parameter: erode_radius");
  }
  ROS_INFO("Erode Radius [%f]", erode_radius_);
  if(!node_.getParam("traversability_threshold", traversability_threshold_)){
    ROS_WARN_STREAM("Couldn't get parameter: traversability_threshold");
  }
  ROS_INFO("traversability_threshold [%f]", traversability_threshold_);
  if(!node_.getParam("verbose_timer", verbose_timer_)){
    ROS_WARN_STREAM("Couldn't get parameter: verbose_timer");
  }
  if(!node_.getParam("verbose_timer", verbose_timer_)){
    ROS_WARN_STREAM("Couldn't get parameter: verbose_timer");
  }
  if(!node_.getParam("grid_map_sub_topic", grid_map_sub_topic_)){
    ROS_WARN_STREAM("Couldn't get parameter: grid_map_sub_topic");
  }
  if(!node_.getParam("point_cloud_sub_topic", point_cloud_sub_topic_)){
    ROS_WARN_STREAM("Couldn't get parameter: point_cloud_sub_topic");
  }
  if(!node_.getParam("pose_sub_topic", pose_sub_topic_)){
      ROS_WARN_STREAM("Couldn't get parameter: pose_sub_topic");
    }

  std::string param_name = "grid_map_filters";
  XmlRpc::XmlRpcValue config;
  if(!node_.getParam(param_name, config)) {
    ROS_ERROR("Could not load the filter chain configuration from parameter %s, are you sure it was pushed to the parameter server? Assuming that you meant to leave it empty.", param_name.c_str());
    return;
  }

  // Setup filter chain
  if (!filter_chain_.configure(param_name, node_)){
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

  if (algorithm_ == "C"){
      std::cout << "C" << std::endl;
      gridMapFilterChain(map);
      imageProcessing(map);
      stepCreation(map);
      reset();
      return;
  }

  if (algorithm_ == "B"){
      std::cout << "B" << std::endl;
      gridMapFilterChain(map);
      imageProcessing(map);
      reset();
  }

  sensor_msgs::PointCloud2 pointCloud;

  if (algorithm_ == "A"){
      std::cout << "A end" << std::endl;
      grid_map::GridMapRosConverter::toPointCloud(map, "elevation", pointCloud); // takes "elevation" for legacy algorithm
  } else {
      std::cout << "B end" << std::endl;
      grid_map::GridMapRosConverter::toPointCloud(map, "product", pointCloud); // takes "product" for masking algorithm
  }

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
    std::vector<double> timing_vector;

    foreach(rosbag::MessageInstance const m, view){
        grid_map_msgs::GridMap::ConstPtr s = m.instantiate<grid_map_msgs::GridMap>();

        if (s != NULL){
            std::cin.get();
            tic();

            ++frame;

            std::cout << "frames = " << frame << std::endl;
//            std::cout << "received gridmap at time " << m.getTime().toNSec() << " with resolution:" <<   s->info.resolution << "and time: " << s->info.header.stamp << std::endl;
            std::cout << "rosbag time: " << s->info.header.stamp << std::endl;
            elevationMapCallback(*s);

            std::cout << "Press [Enter] to continue to next gridmap message" << std::endl;
            double frame_time;
            frame_time = toc().count();
            std::cout << frame_time << " ms: frame_" << frame << std::endl;
            timing_vector.push_back(frame_time);
        }

        geometry_msgs::PoseWithCovarianceStamped::ConstPtr i = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (i !=NULL){
      //      std::cout << "position (x, y, z): " << i->pose.pose.position.x << ", " << i->pose.pose.position.y << ", "  << i->pose.pose.position.z << std::endl;
            robotPoseCallBack(i);
            pose_pub_.publish(*i);
            }
    }

    bag.close();


    double average, sum;
    for (size_t k = 0; k < timing_vector.size(); ++k){
        sum = sum + timing_vector[k];
    }
    average = sum / timing_vector.size();
//    average = std::accumulate(timing_vector.begin(), timing_vector.end(), 0.0)/timing_vector.size();
    std::cout << "The average time per frame is" << average << std::endl;
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

void Pass::publishRectangles(){
    std::cout << "Entered publishRectangles" << std::endl;
    visualization_msgs::MarkerArray rectangles_array;

    std::cout << "Elevation values: " << std::endl;
    for (size_t i = 0; i < stepcreator_.rectangles_.size(); ++i){

        std::cout << stepcreator_.rectangles_[i].elevation_ << std::endl;

        planeseg::contour contour = stepcreator_.rectangles_[i];

        visualization_msgs::Marker rectMarker;
        rectMarker.header.frame_id = "odom";
        rectMarker.header.stamp = ros::Time::now();
        rectMarker.ns = "rectangles";
        rectMarker.id = i;
        rectMarker.type = visualization_msgs::Marker::LINE_STRIP;
        rectMarker.action = visualization_msgs::Marker::ADD;
        rectMarker.pose.orientation.w = 0.0;
        rectMarker.scale.x = 0.05;
        rectMarker.color.r = 255;
        rectMarker.color.g = 255;
        rectMarker.color.b = 1;
        rectMarker.color.a = 1;
        std::vector<geometry_msgs::Point> pointsGM(contour.points_.size()+1);
        int count;

        for (size_t r = 0; r < contour.points_.size(); ++r){

            float x, y, z;
            x = (static_cast<float>(contour.points_[r].x) * gm_resolution_); //contour.points_[0].x
            y = (static_cast<float>(contour.points_[r].y) * gm_resolution_); //contour.points_[0].y
            z = static_cast<float>(contour.elevation_);

            pointsGM[r].x = -y + ((imgprocessor_.final_img_.image.rows / 2) * gm_resolution_) + gm_position_[0];
            pointsGM[r].y = -x + ((imgprocessor_.final_img_.image.cols / 2) * gm_resolution_) + gm_position_[1];
            pointsGM[r].z = z;
            count = r;
        }

        // re-add first point to close the loop
        float x, y, z;
        x = (static_cast<float>(contour.points_[0].x) * gm_resolution_);
        y = (static_cast<float>(contour.points_[0].y) * gm_resolution_);
        z = static_cast<float>(contour.elevation_);

        pointsGM[count+1].x = -y + ((imgprocessor_.final_img_.image.rows / 2) * gm_resolution_) + gm_position_[0];
        pointsGM[count+1].y = -x + ((imgprocessor_.final_img_.image.cols / 2) * gm_resolution_) + gm_position_[1];
        pointsGM[count+1].z = z;

        rectMarker.points = pointsGM;
        rectMarker.frame_locked = true;
        rectangles_array.markers.push_back(rectMarker);
    }

    std::cout << "3" << std::endl;
    rectangles_pub_.publish(rectangles_array);

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

        if (s != NULL){
            ++frame;
            if (frame == n){
                std::cin.get();
                tic();
                elevationMapCallback(*s);
                std::cout << toc().count() << " ms: frame_" << frame << std::endl;
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

void Pass::imageProcessing(grid_map::GridMap &gridmap){

    const float nanValue = 1;
    replaceNan(gridmap.get("slope"), nanValue);

    convertGridmapToFloatImage(gridmap, "slope", imgprocessor_.original_img_, true);

    imgprocessor_.process();;


    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(imgprocessor_.final_img_.image, "mask", gridmap);

    if (algorithm_ == "B"){
        gridmap.add("product");
        multiplyLayers(gridmap.get("elevation"), gridmap.get("mask"), gridmap.get("product"));
        replaceZeroToNan(gridmap.get("product"));
    }

    // Publish updated grid map.
    grid_map_msgs::GridMap output_msg;
    grid_map::GridMapRosConverter::toMessage(gridmap, output_msg);
    filtered_map_pub_.publish(output_msg);
}

void Pass::stepCreation(grid_map::GridMap &gridmap){

    std::cout << "Gridmap resolution = " << gridmap.getResolution() << std::endl;
    std::cout << "Gridmap position = " << gridmap.getPosition()[0] << ", " << gridmap.getPosition()[1] << std::endl;
    gm_resolution_ = gridmap.getResolution();
    gm_position_.push_back(gridmap.getPosition()[0]);
    gm_position_.push_back(gridmap.getPosition()[1]);

    convertGridmapToFloatImage(gridmap, "elevation", stepcreator_.elevation_, true);
    stepcreator_.pnts_ = imgprocessor_.all_contours_.contours_rect_;
    stepcreator_.processed_ = imgprocessor_.final_img_;
    stepcreator_.go();
//    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(stepcreator_.elevation_masked_.image, "reconstructed", gridmap);
    publishRectangles();
}

void Pass::reset(){
    gm_position_.clear();
    imgprocessor_.reset();
    stepcreator_.reset();
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

void Pass::gridMapFilterChain(grid_map::GridMap& input_map){
//  tic();

  // Apply filter chain.
  grid_map::GridMap output_map;
  if (!filter_chain_.update(input_map, output_map)) {
    std::cout << "couldn't update the grid map filter chain" << std::endl;
    grid_map_msgs::GridMap failmessage;
    grid_map::GridMapRosConverter::toMessage(input_map, failmessage);
    return;
  }
/*
  if (verbose_timer_) {
    std::cout << toc().count() << " ms: filter chain\n";
  }

  tic();
*/
  // Publish filtered output grid map.
  grid_map_msgs::GridMap output_msg;
  grid_map::GridMapRosConverter::toMessage(output_map, output_msg);
  filtered_map_pub_.publish(output_msg);

  input_map = output_map;
}

void Pass::saveGridMapMsgAsPCD(const grid_map_msgs::GridMap& msg, int frame){
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(msg, grid_map);
    sensor_msgs::PointCloud2 pointCloud_sensor_msg;
    grid_map::GridMapRosConverter::toPointCloud(grid_map, "elevation", pointCloud_sensor_msg);
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(pointCloud_sensor_msg, point_cloud);

    std::string pcd_filename;
    pcd_filename = "/home/christos/rosbags/pcd_by_frame/pcd_frame_" + std::to_string(frame) + ".pcd";

    pcl::io::savePCDFile(pcd_filename, point_cloud);
    std::cout << "Saved " << point_cloud.size () << " data points to " << pcd_filename << std::endl;
}

void Pass::replaceNan(grid_map::GridMap::Matrix& m, const double newValue){

  for(int r = 0; r < m.rows(); r++)
  {
    for(int c = 0; c < m.cols(); c++)
    {
      if (std::isnan(m(r,c)))
      {
        m(r,c) = newValue;
      }
    }
  }
}

void Pass::replaceZeroToNan(grid_map::GridMap::Matrix& m){

  for(int r = 0; r < m.rows(); r++)
  {
    for(int c = 0; c < m.cols(); c++)
    {
      if (m(r,c) == 0)
      {
        m(r,c) = NAN;
      }
    }
  }
}

void Pass::multiplyLayers(grid_map::GridMap::Matrix& factor1, grid_map::GridMap::Matrix& factor2, grid_map::GridMap::Matrix& result){

  for(int r = 0; r < result.rows(); r++)
  {
    for(int c = 0; c < result.cols(); c++)
    {
        result(r,c) = factor1(r,c) * factor2(r,c);
    }
  }
}

bool Pass::convertGridmapToFloatImage(const grid_map::GridMap& gridMap, const std::string& layer, cv_bridge::CvImage& cvImage, bool negative){
    std::cout << "Entered convertGridmapToFloatImage" << std::endl;
    cvImage.header.stamp.fromNSec(gridMap.getTimestamp());
    cvImage.header.frame_id = gridMap.getFrameId();
    cvImage.encoding = CV_32F;

    if (negative == false) {
        return grid_map::GridMapCvConverter::toImage<float, 1>(gridMap, layer, CV_32F, 0, 1, cvImage.image);
    } else {
    return toImageWithNegatives(gridMap, layer, CV_32F, 0, 1, cvImage.image);
    }
}

bool Pass::toImageWithNegatives(const grid_map::GridMap& gridMap, const std::string& layer, const int encoding, const float lowerValue, const float upperValue, cv::Mat& image){

  // Initialize image.
  if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0) {
    image = cv::Mat::zeros(gridMap.getSize()(0), gridMap.getSize()(1), encoding);
  } else {
    std::cerr << "Invalid grid map?" << std::endl;
    return false;
  }

  // Get max image value.
  float imageMax;
  imageMax = 1;

  grid_map::GridMap map = gridMap;
  const grid_map::Matrix& data = map[layer];

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const float& value = data(index(0), index(1));
    if (std::isfinite(value)) {
      const float imageValue = (float)(((value - lowerValue) / (upperValue - lowerValue)) * (float)imageMax);
      const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
      unsigned int channel = 0;
      image.at<cv::Vec<float, 1>>(imageIndex(0), imageIndex(1))[channel] = imageValue;
    }
  }

  return true;
}

void Pass::setupSubscribers(){
    grid_map_sub_ = node_.subscribe(grid_map_sub_topic_, 100,
                                      &Pass::elevationMapCallback, this);
    point_cloud_sub_ = node_.subscribe(point_cloud_sub_topic_, 100,
                                      &Pass::pointCloudCallback, this);
    pose_sub_ = node_.subscribe(pose_sub_topic_, 100,
                                      &Pass::robotPoseCallBack, this);
}
