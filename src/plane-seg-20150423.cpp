#include <string>
#include <fstream>
#include <unordered_map>
#include <chrono>

#include <drc_utils/RansacGeneric.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include "RectangleFitter.hpp"
#include "RobustNormalEstimator.hpp"
#include "PlaneSegmenter.hpp"

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointXYZL> LabeledCloud;
typedef Eigen::Matrix<float,Eigen::Dynamic,3> MatrixX3f;


int main() {
  // read pcd file
  std::string inFile = "/home/antone/data/tilted-steps.pcd";
  Eigen::Vector3f origin(0.248091, 0.012443, 1.806473);
  Eigen::Vector3f lookDir(0.837001, 0.019831, -0.546842);

  /*
  inFile = "/home/antone/data/2015-03-11_testbed/terrain_med.pcd";
  origin << -0.028862, -0.007466, 0.087855;
  lookDir << 0.999890, -0.005120, -0.013947;

  inFile = "/home/antone/data/2015-03-11_testbed/terrain_close.pcd";
  origin << -0.028775, -0.005776, 0.087898;
  lookDir << 0.999956, -0.005003, 0.007958;
  */

  LabeledCloud::Ptr inCloud(new LabeledCloud());
  pcl::io::loadPCDFile(inFile, *inCloud);
  std::cout << "Original cloud size " << inCloud->size() << std::endl;

  // voxelize
  LabeledCloud::Ptr cloud(new LabeledCloud());
  pcl::VoxelGrid<pcl::PointXYZL> voxelGrid;
  voxelGrid.setInputCloud(inCloud);
  voxelGrid.setLeafSize (0.01f, 0.01f, 0.01f);
  voxelGrid.filter(*cloud);
  for (int i = 0; i < (int)cloud->size(); ++i) cloud->points[i].label = i;
  std::cout << "Voxelized cloud size " << cloud->size() << std::endl;

  // write cloud
  pcl::io::savePCDFileBinary("/home/antone/temp/cloud.pcd", *cloud);

  // pose
  cloud->sensor_origin_.head<3>() = origin;
  cloud->sensor_origin_[3] = 1;
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

  // kdtree based normals
  auto t0 = std::chrono::high_resolution_clock::now();
  std::cout << "computing normals..." << std::flush;
  RobustNormalEstimator normalEstimator;
  normalEstimator.setMaxEstimationError(0.01);
  normalEstimator.setRadius(0.1);
  normalEstimator.setMaxCenterError(0.02);
  normalEstimator.setMaxIterations(100);
  NormalCloud::Ptr normals(new NormalCloud());
  normalEstimator.go(cloud, *normals);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
  std::cout << "finished in " << dt.count()/1e3 << " sec" << std::endl;

  // write normals
  pcl::io::savePCDFileBinary("/home/antone/temp/robust_normals.pcd",
                             *normals);

  // try custom plane segmentation
  t0 = std::chrono::high_resolution_clock::now();
  std::cout << "segmenting planes..." << std::flush;
  PlaneSegmenter segmenter;
  segmenter.setData(cloud, normals);
  segmenter.setMaxError(0.05);
  segmenter.setMaxAngle(10);
  segmenter.setMinPoints(100);
  PlaneSegmenter::Result result = segmenter.go();
  t1 = std::chrono::high_resolution_clock::now();
  dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
  std::cout << "finished in " << dt.count()/1e3 << " sec" << std::endl;
  {
    std::ofstream ofs("/home/antone/temp/labels.txt");
    for (const int lab : result.mLabels) {
      ofs << lab << std::endl;
    }
    ofs.close();
  }

  {
    std::ofstream ofs("/home/antone/temp/planes.txt");
    for (auto it : result.mPlanes) {
      auto& plane = it.second;
      ofs << it.first << " " << plane.transpose() << std::endl;
    }
    ofs.close();
  }

  // create point clouds
  std::unordered_map<int,std::vector<Eigen::Vector3f>> cloudMap;
  for (int i = 0; i < (int)result.mLabels.size(); ++i) {
    int label = result.mLabels[i];
    if (label <= 0) continue;
    cloudMap[label].push_back(cloud->points[i].getVector3fMap());
  }
  struct Plane {
    MatrixX3f mPoints;
    Eigen::Vector4f mPlane;
  };
  std::vector<Plane> planes;
  planes.reserve(cloudMap.size());
  for (auto it : cloudMap) {
    int n = it.second.size();
    Plane plane;
    plane.mPoints.resize(n,3);
    for (int i = 0; i < n; ++i) plane.mPoints.row(i) = it.second[i];
    plane.mPlane = result.mPlanes[it.first];
    planes.push_back(plane);
  }

  std::vector<RectangleFitter::Result> results;
  for (auto& plane : planes) {
    RectangleFitter fitter;
    fitter.setData(plane.mPoints, plane.mPlane);
    auto result = fitter.go();
    results.push_back(result);
  }

  {
    std::ofstream ofs("/home/antone/temp/boxes.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      for (auto& p : res.mPolygon) {
        ofs << i << " " << p.transpose() << std::endl;
      }
    }
    ofs.close();
  }

  {
    std::ofstream ofs("/home/antone/temp/hulls.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      for (auto& p : res.mConvexHull) {
        ofs << i << " " << p.transpose() << std::endl;
      }
    }
    ofs.close();
  }

  return 1;
}
