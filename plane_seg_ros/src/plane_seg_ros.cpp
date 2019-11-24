#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "plane_seg/BlockFitter.hpp"


// convenience methods
auto vecToStr = [](const Eigen::Vector3f& iVec) {
  std::ostringstream oss;
  oss << iVec[0] << ", " << iVec[1] << ", " << iVec[2];
  return oss.str();
};
auto rotToStr = [](const Eigen::Matrix3f& iRot) {
  std::ostringstream oss;
  Eigen::Quaternionf q(iRot);
  oss << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z();
  return oss.str();
};


class Pass{
  public:
    Pass(ros::NodeHandle node_);
    
    ~Pass(){
    }

    void elevationMapCallback(const grid_map_msgs::GridMap& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    void processCloud(planeseg::LabeledCloud::Ptr& inCloud, Eigen::Vector3f origin, Eigen::Vector3f lookDir);
    void processFromFile(int test_example);

    void publishHullsAsCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs,
                                 int secs, int nsecs);

    void publishHullsAsMarkers(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs);
    void printResultAsJson();
    void publishResult();

  private:
    ros::NodeHandle node_;
    std::vector<double> colors_;

    ros::Subscriber point_cloud_sub_, grid_map_sub_, pose_sub_;
    ros::Publisher received_cloud_pub_, hull_cloud_pub_, hull_markers_pub_, look_pose_pub_;

    Eigen::Isometry3d last_robot_pose_;
    planeseg::BlockFitter::Result result_;
};

Pass::Pass(ros::NodeHandle node_):
    node_(node_){

  std::string input_body_pose_topic;
  node_.getParam("input_body_pose_topic", input_body_pose_topic);

  grid_map_sub_ = node_.subscribe("/elevation_mapping/elevation_map", 100,
                                    &Pass::elevationMapCallback, this);
  point_cloud_sub_ = node_.subscribe("/plane_seg/point_cloud_in", 100,
                                    &Pass::pointCloudCallback, this);
  pose_sub_ = node_.subscribe("/state_estimator/pose_in_odom", 100,
                                    &Pass::robotPoseCallBack, this);

  received_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/received_cloud", 10);
  hull_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/hull_cloud", 10);
  hull_markers_pub_ = node_.advertise<visualization_msgs::Marker>("/plane_seg/hull_markers", 10);
  look_pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/plane_seg/look_pose", 10);

  last_robot_pose_ = Eigen::Isometry3d::Identity();

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

void Pass::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
  //std::cout << "got pose\n";
  tf::poseMsgToEigen(msg->pose.pose, last_robot_pose_);
}


void quat_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+q2*q2));
  pitch = asin(2.0*(q0*q2-q3*q1));
  yaw = atan2(2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3));
}

Eigen::Vector3f convertRobotPoseToSensorLookDir(Eigen::Isometry3d robot_pose){

  Eigen::Quaterniond quat = Eigen::Quaterniond( robot_pose.rotation() );
  double r,p,y;
  quat_to_euler(quat, r, p, y);
  //std::cout << r*180/M_PI << ", " << p*180/M_PI << ", " << y*180/M_PI << " rpy in Degrees\n";

  double yaw = y;
  double pitch = -p;
  double xDir = cos(yaw)*cos(pitch);
  double yDir = sin(yaw)*cos(pitch);
  double zDir = sin(pitch);
  return Eigen::Vector3f(xDir, yDir, zDir);
}


void Pass::elevationMapCallback(const grid_map_msgs::GridMap& msg){
  //std::cout << "got grid map / ev map\n";

  // convert message to GridMap, to PointCloud to LabeledCloud
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, "elevation", pointCloud);
  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::fromROSMsg(pointCloud,*inCloud);

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
  std::string home_dir = getenv("HOME");
  Eigen::Vector3f origin, lookDir;
  if (test_example == 0){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/drs_testing_data/terrain/tilted-steps.pcd";
    origin <<0.248091, 0.012443, 1.806473;
    lookDir <<0.837001, 0.019831, -0.546842;
  }else if (test_example == 1){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/drs_testing_data/terrain/terrain_med.pcd";
    origin << -0.028862, -0.007466, 0.087855;
    lookDir << 0.999890, -0.005120, -0.013947;
  }else if (test_example == 2){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/drs_testing_data/terrain/terrain_close_rect.pcd";
    origin << -0.028775, -0.005776, 0.087898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 3){ // RGBD (Realsense D435) example from ANYmal
    inFile = home_dir + "/drs_testing_data/terrain/anymal/ori_entrance_stair_climb/06.pcd";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 4){ // Leica map
    inFile = home_dir + "/drs_testing_data/leica/race_arenas/RACE_crossplaneramps_sub1cm_cropped_meshlab_icp.ply";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 5){ // Leica map
    inFile = home_dir + "/drs_testing_data/leica/race_arenas/RACE_stepfield_sub1cm_cropped_meshlab_icp.ply";
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


void Pass::processCloud(planeseg::LabeledCloud::Ptr& inCloud, Eigen::Vector3f origin, Eigen::Vector3f lookDir){

  planeseg::BlockFitter fitter;
  fitter.setSensorPose(origin, lookDir);
  fitter.setCloud(inCloud);
  fitter.setDebug(false); // MFALLON modification
  fitter.setRemoveGround(false); // MFALLON modification from default

  // this was 5 for LIDAR. changing to 10 really improved elevation map segmentation
  // I think its because the RGB-D map can be curved
  fitter.setMaxAngleOfPlaneSegmenter(10);

  result_ = fitter.go();


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

  publishHullsAsCloud(cloud_ptrs, 0, 0);
  publishHullsAsMarkers(cloud_ptrs, 0, 0);

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


void Pass::publishHullsAsMarkers(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
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
  hull_markers_pub_.publish(marker);
}


int main( int argc, char** argv ){
  // Turn off warning message about labels
  // TODO: look into how labels are used
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


  ros::init(argc, argv, "plane_seg");
  ros::NodeHandle nh("~");
  Pass *app = new Pass(nh);

  ROS_INFO_STREAM("plane_seg ros ready");
  ROS_INFO_STREAM("=============================");

  bool run_test_program = false;
  nh.param("/plane_seg/run_test_program", run_test_program, false); 
  std::cout << "run_test_program: " << run_test_program << "\n";


  // Enable this to run the test programs
  if (run_test_program){
    std::cout << "Running test examples\n";
    app->processFromFile(0);
    app->processFromFile(1);
    app->processFromFile(2);
    app->processFromFile(3);
    // RACE examples don't work well
    //app->processFromFile(4);
    //app->processFromFile(5);
    exit(-1);
  }

  ROS_INFO_STREAM("Waiting for ROS messages");
  ros::spin();

  return 1;
}
