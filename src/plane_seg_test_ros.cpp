#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
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

    void publishHullsAsCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs,
                                 int secs, int nsecs);

    void publishHullsAsMarkers(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs);

    void processFromFile();
    void printResultAsJson();
    void publishResult();


  private:
    ros::NodeHandle node_;
    std::vector<double> colors_;

    ros::Publisher input_cloud_pub_, hull_cloud_pub_, hull_markers_pub_, look_pose_pub_;
    planeseg::BlockFitter::Result result_;
};

Pass::Pass(ros::NodeHandle node_):
    node_(node_){

  input_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/input_cloud", 10);
  hull_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/hull_cloud", 10);
  hull_markers_pub_ = node_.advertise<visualization_msgs::Marker>("/plane_seg/hull_markers", 10);
  look_pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/plane_seg/look_pose", 10);

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


void Pass::processFromFile(){

  sleep(2);


  std::string home_dir = getenv("HOME");

  int test_example = 2;

  std::string inFile;
  Eigen::Vector3f origin, lookDir;
  if (test_example == 0){
    inFile = home_dir + "/drs_testing_data/terrain/tilted-steps.pcd";
    origin <<0.248091, 0.012443, 1.806473;
    lookDir <<0.837001, 0.019831, -0.546842;
  }else if (test_example == 1){
    inFile = home_dir + "/drs_testing_data/terrain/terrain_med.pcd";
    origin << -0.028862, -0.007466, 0.087855;
    lookDir << 0.999890, -0.005120, -0.013947;
  }else if (test_example == 2){
    inFile = home_dir + "/drs_testing_data/terrain/terrain_close_rect.pcd";
    origin << -0.028775, -0.005776, 0.087898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }

  inFile = home_dir + "/Dropbox/desktop/stair_climb/03.pcd";
  origin << -0.028775, -0.005776, 0.987898;
  lookDir << 0.999956, -0.005003, 0.007958;


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



  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::io::loadPCDFile(inFile, *inCloud);

  planeseg::BlockFitter fitter;
  fitter.setSensorPose(origin, lookDir);
  fitter.setCloud(inCloud);

  fitter.setRemoveGround(false); // MFALLON modification from default
  result_ = fitter.go();  


  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time(0, 0);
  msg.header.frame_id = "odom";
  tf::poseEigenToMsg(pose_d, msg.pose);
  look_pose_pub_.publish(msg);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*inCloud, output);
  output.header.stamp = ros::Time(0, 0);
  output.header.frame_id = "odom";
  input_cloud_pub_.publish(output);


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


void Pass::publishHullsAsCloud(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs){

  // 
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
  //for (size_t i = 0; i < 2; i++){

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

    // end points
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

  ros::init(argc, argv, "position_controller"); 
  ros::NodeHandle nh("~");
  Pass *app = new Pass(nh);

  ROS_INFO_STREAM("=============================");
  app->processFromFile();

  return 1;
}
