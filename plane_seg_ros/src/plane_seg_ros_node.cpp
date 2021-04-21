#include <ros/node_handle.h>
#include "plane_seg_ros/Pass.hpp"
#include <ros/console.h>
#include <pcl/console/print.h>
#include <iostream>

using namespace planeseg;

int main( int argc, char** argv ){
  // Turn off warning message about labels
  // TODO: look into how labels are used
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


  ros::init(argc, argv, "plane_seg");
  ros::NodeHandle nh("~");

  std::unique_ptr<Pass> app = std::make_unique<Pass>(nh);

  ROS_INFO_STREAM("plane_seg ros ready");
  ROS_INFO_STREAM("=============================");

  bool run_test_program = false;
  nh.param("run_test_program", run_test_program, false);
  std::cout << "run_test_program: " << std::boolalpha << run_test_program << "\n";

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

    std::cout << "Finished!\n";
    return 0;
  }

  bool run_sequential_test = false;
  nh.param("run_sequential_test", run_sequential_test, false);
  std::cout << "run_sequential_test: " << std::boolalpha << run_sequential_test << "\n";

  //Enable this to run the sequential test
  if (run_sequential_test){
      std::cout << "Running sequential test\n";
      
      std::string filename_;
      nh.getParam("rosbag_path", filename_);

      app->stepThroughFile(filename_);
      std::cout << "Finished!\n";
      return 0;
  }

  bool run_nth_cloud = false;
  nh.param("run_nth_cloud", run_nth_cloud, false);
  std::cout << "run_nth_cloud: " << std::boolalpha << run_nth_cloud << "\n";

  // Enable this to run nth coud
  if (run_nth_cloud){
    int n_ = 1;
    nh.getParam("n", n_);
      std::cout << "Running " << n_ << "-th cloud\n";
      std::string filename_;
      nh.getParam("rosbag_path", filename_);

      app->extractNthCloud(filename_, n_);
      std::cout << "Finshed!\n";
      return 0;
  }

  app->setupSubscribers();
  ROS_INFO_STREAM("Waiting for ROS messages");
  ros::spin();

  return 0;
}
