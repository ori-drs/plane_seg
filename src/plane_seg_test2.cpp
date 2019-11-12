#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

#include "plane_seg/RobustNormalEstimator.hpp"

#include <opencv2/opencv.hpp>

int main() {
  // read pcd file
  std::string home_dir = getenv("HOME");

  std::string inFile = home_dir + "/drs_testing_data/terrain/tilted-steps.pcd";
  Eigen::Vector3f origin(0.248091, 0.012443, 1.806473);
  Eigen::Vector3f lookDir(0.837001, 0.019831, -0.546842);
  Eigen::Vector3f centerPoint(1.27638733386993, -0.106330007314682, 0.348578572273254);


  inFile = home_dir +  "/drs_testing_data/terrain/terrain_med.pcd";
  origin << -0.028862, -0.007466, 0.087855;
  lookDir << 0.999890, -0.005120, -0.013947;

  inFile = home_dir + "/drs_testing_data/terrain/terrain_close_rect.pcd";
  origin << -0.028775, -0.005776, 0.087898;
  lookDir << 0.999956, -0.005003, 0.007958;
  

  std::cout << inFile << "\n";
  /*
  */

  Eigen::Vector3f blockSize(15+3/8.0, 15+5/8.0, 5+5/8.0);
  blockSize *= 0.0254;

  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::io::loadPCDFile(inFile, *inCloud);

  // filter points at some radius around center point
  {
    const float kRadius = 0.35;
    std::vector<std::pair<int,float>> indexDistances;
    for (int i = 0; i < (int)inCloud->size(); ++i) {
      Eigen::Vector3f p = inCloud->points[i].getVector3fMap();
      float dist = (p-centerPoint).norm();
      if (dist < kRadius) {
        indexDistances.push_back(std::make_pair(i,dist));
      }
    }

    std::sort(indexDistances.begin(), indexDistances.end(),
              [](const std::pair<int,float>& iA,
                 const std::pair<int,float>& iB){
                return iA.second<iB.second;});

    planeseg::LabeledCloud::Ptr tempCloud(new planeseg::LabeledCloud());
    tempCloud->resize(indexDistances.size());
    for (int i = 0; i < (int)tempCloud->size(); ++i) {
      tempCloud->points[i] = inCloud->points[indexDistances[i].first];
    }

    std::swap(inCloud, tempCloud);
  }

  // find normals
  planeseg::RobustNormalEstimator est;
  planeseg::NormalCloud::Ptr normals(new planeseg::NormalCloud());
  est.setRadius(0.1);
  est.setMaxEstimationError(0.01);
  est.setMaxCenterError(0.05);
  est.setMaxIterations(200);
  est.computeCurvature(true);
  est.go(inCloud, *normals);

  // grab normal at click point
  Eigen::Vector3f centerNormal(normals->points[0].normal_x,
                               normals->points[0].normal_y,
                               normals->points[0].normal_z);
  // plane at click point
  Eigen::Vector4f plane;
  plane.head<3>() = centerNormal;
  plane[3] = -centerNormal.dot(inCloud->points[0].getVector3fMap());
  std::cout << "PLANE " << plane.transpose() << std::endl;

  {
    std::string outFilePlane = home_dir + "/drs_testing_data/terrain/test-output-plane-at-click.txt";
    std::ofstream ofs(outFilePlane);
    for (int i = 0; i < (int)inCloud->size(); ++i) {
      Eigen::Vector3f p = inCloud->points[i].getVector3fMap();
      Eigen::Vector3f norm(normals->points[i].normal_x,
                           normals->points[i].normal_y,
                           normals->points[i].normal_z);
      ofs << p.transpose() << " " << norm.transpose() << std::endl;
    }
    ofs.close();
  }

  // project points to height map
  {
    const float kSampleSize = 0.01;

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    Eigen::Vector3f rz = centerNormal;
    Eigen::Vector3f ry = rz.cross(Eigen::Vector3f::UnitX());
    Eigen::Vector3f rx = ry.cross(rz);
    pose.linear().col(0) = rx.normalized();
    pose.linear().col(1) = ry.normalized();
    pose.linear().col(2) = rz.normalized();
    pose.translation() = centerPoint;
    Eigen::Affine3f transform(pose.matrix());

    planeseg::LabeledCloud::Ptr tempCloud(new planeseg::LabeledCloud());
    pcl::transformPointCloud(*inCloud, *tempCloud, transform.inverse());

    // flatten cloud
    planeseg::LabeledCloud::Ptr flatCloud(new planeseg::LabeledCloud());
    pcl::copyPointCloud(*tempCloud, *flatCloud);
    for (auto& p : flatCloud->points) p.getVector3fMap()[2] = 0;

    // find edge-like structures using kdtree
    const float kHeightThresh = 0.05;
    pcl::search::KdTree<pcl::PointXYZL>::Ptr tree
      (new pcl::search::KdTree<pcl::PointXYZL>());
    tree->setInputCloud(flatCloud);
    std::vector<int> indices;
    std::vector<float> distances;
    std::vector<float> bins(8);
    std::vector<float> radialDistances(8);
    std::vector<float> radialHeights(8);
    std::string outFileEdges = home_dir + "/drs_testing_data/terrain/test-output-edges.txt";
    std::ofstream ofs(outFileEdges);
    for (int i = 0; i < (int)tempCloud->size(); ++i) {
      const auto& p0 = tempCloud->points[i].getVector3fMap();
      tree->radiusSearch(i, 0.1, indices, distances);
      std::fill(radialDistances.begin(), radialDistances.end(), 1e10);
      std::fill(radialHeights.begin(), radialHeights.end(), 1e10); // TODO
      for (const auto idx : indices) {
        if (idx == i) continue;
        const auto& p = tempCloud->points[idx].getVector3fMap();
        Eigen::Vector3f d = p-p0;
        float theta = std::atan2(d[1],d[0]);
        if (theta < 0) theta += 2*M_PI;
        int binIdx = std::floor(theta/2/M_PI*8);
        float radialDistance = d.head<2>().norm();
        if (radialDistance < radialDistances[binIdx]) {
          radialDistances[binIdx] = radialDistance;
          radialHeights[binIdx] = d[2];
        }
      }
      bool isEdge = false;
      for (const auto height : radialHeights) {
        if (height < -kHeightThresh) {
          isEdge = true;
          break;
        }
      }
      if (isEdge) ofs << inCloud->points[i].getVector3fMap().transpose() << std::endl;
    }
    ofs.close();













    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*tempCloud, minPoint, maxPoint);

    Eigen::Vector2f dimensions = (maxPoint - minPoint).head<2>();
    Eigen::Vector2f imageDimensions = dimensions/kSampleSize;
    int w = std::ceil(imageDimensions[0]);
    int h = std::ceil(imageDimensions[1]);
    std::vector<float> heights(w*h);
    std::fill(heights.begin(), heights.end(), -1000);

    for (int i = 0; i < (int)tempCloud->size(); ++i) {
      Eigen::Vector3f p = tempCloud->points[i].getVector3fMap() -
        minPoint.head<3>();
      p.head<2>() /= kSampleSize;
      int x = std::round(p[0]);
      int y = std::round(p[1]);
      float& curHeight = heights[y*w+x];
      if (p[2] > curHeight) curHeight = p[2];
    }

    {

      std::string outFileTxt = home_dir + "/drs_testing_data/terrain/test-output-heightmap.txt";
      std::ofstream ofs(outFileTxt);
      for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
          ofs << heights[i*w+j] << " ";
        }
        ofs << std::endl;
      }
      ofs.close();
    }
    
    std::string outFile = home_dir + "/drs_testing_data/terrain/test-output-aligned.pcd";

    pcl::io::savePCDFileBinary(outFile, *tempCloud);
  }

  // find edges in cloud

  // fit block
  

  return 1;
}
