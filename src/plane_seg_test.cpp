#include <pcl/io/pcd_io.h>

#include "plane_seg/BlockFitter.hpp"

int main() {
  // read pcd file
  std::string home_dir = getenv("HOME");

  std::string inFile = home_dir + "/drs_testing_data/terrain/tilted-steps.pcd";
  Eigen::Vector3f origin(0.248091, 0.012443, 1.806473);
  Eigen::Vector3f lookDir(0.837001, 0.019831, -0.546842);

  inFile = home_dir + "/drs_testing_data/terrain/terrain_med.pcd";
  origin << -0.028862, -0.007466, 0.087855;
  lookDir << 0.999890, -0.005120, -0.013947;

  inFile = home_dir + "/drs_testing_data/terrain/terrain_close_rect.pcd";
  origin << -0.028775, -0.005776, 0.087898;
  lookDir << 0.999956, -0.005003, 0.007958;
  /*
  */

  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::io::loadPCDFile(inFile, *inCloud);

  planeseg::BlockFitter fitter;
  fitter.setSensorPose(origin, lookDir);
  fitter.setCloud(inCloud);
  auto result = fitter.go();
  return 1;
}
