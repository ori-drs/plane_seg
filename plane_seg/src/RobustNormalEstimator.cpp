#include "plane_seg/RobustNormalEstimator.hpp"

#include "plane_seg/PlaneFitter.hpp"
#include <pcl/search/kdtree.h>

#include "plane_seg/Types.hpp"

using namespace planeseg;

RobustNormalEstimator::
RobustNormalEstimator() {
  setRadius(0.1);
  setMaxEstimationError(0.01);
  setMaxCenterError(0.02);
  setMaxIterations(100);
  computeCurvature(true);
}

void RobustNormalEstimator::
setRadius(const float iRadius) {
  mRadius = iRadius;
}

void RobustNormalEstimator::
setMaxEstimationError(const float iDist) {
  mMaxEstimationError = iDist;
}

void RobustNormalEstimator::
setMaxCenterError(const float iDist) {
  mMaxCenterError = iDist;
}

void RobustNormalEstimator::
setMaxIterations(const int iIters) {
  mMaxIterations = iIters;
}

void RobustNormalEstimator::
computeCurvature(const bool iVal) {
  mComputeCurvature = iVal;
}

bool RobustNormalEstimator::
go(const LabeledCloud::Ptr& iCloud, NormalCloud& oNormals) {

  // plane fitter
  PlaneFitter planeFitter;
  planeFitter.setMaxIterations(mMaxIterations);
  planeFitter.setMaxDistance(mMaxEstimationError);
  planeFitter.setRefineUsingInliers(true);
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(1000);

  // kd tree
  pcl::search::KdTree<Point>::Ptr tree
    (new pcl::search::KdTree<Point>());
  tree->setInputCloud(iCloud);
  std::vector<int> indices;
  std::vector<float> distances;

  // loop
  const int n = iCloud->size();
  oNormals.width = n;
  oNormals.height = 0;
  oNormals.resize(n);
  oNormals.is_dense = false;

  for (int i = 0; i < n; ++i) {

    tree->radiusSearch(i, mRadius, indices, distances);
    pts.clear();
    for (const auto idx : indices) {
      pts.push_back(iCloud->points[idx].getVector3fMap());
    }
    auto& norm = oNormals.points[i];
    norm.normal_x = norm.normal_y = norm.normal_z = 0;
    norm.curvature = -1;
    if (pts.size() < 3) continue;

    // solve for plane
    Eigen::Vector3f pt = iCloud->points[i].getVector3fMap();
    planeFitter.setCenterPoint(pt);
    auto res = planeFitter.go(pts);
    auto& plane = res.mPlane;
    if (plane[2] < 0) plane = -plane;
    Eigen::Vector3f normal = plane.head<3>();
    if (std::abs(normal.dot(pt) + plane[3]) > mMaxCenterError) continue;
    if (normal[2]<0) normal = -normal;
    norm.normal_x = normal[0];
    norm.normal_y = normal[1];
    norm.normal_z = normal[2];
    if (mComputeCurvature) norm.curvature = res.mCurvature;
    else norm.curvature = plane[3];
  }

  return true;
}
