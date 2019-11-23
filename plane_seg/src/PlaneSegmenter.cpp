#include "plane_seg/PlaneSegmenter.hpp"

#include <queue>
#include <pcl/search/kdtree.h>

#include "plane_seg/IncrementalPlaneEstimator.hpp"

using namespace planeseg;

PlaneSegmenter::
PlaneSegmenter() {
  setMaxError(0.02);
  setMaxAngle(30);
  setSearchRadius(0.03);
  setMinPoints(500);
}

void PlaneSegmenter::
setData(const LabeledCloud::Ptr& iCloud,
        const NormalCloud::Ptr& iNormals) {
  mCloud = iCloud;
    mNormals = iNormals;
}

void PlaneSegmenter::
setMaxError(const float iError) {
  mMaxError = iError;
}

void PlaneSegmenter::
setMaxAngle(const float iAngle) {
  mMaxAngle = iAngle*M_PI/180;
}

void PlaneSegmenter::
setSearchRadius(const float iRadius) {
  mSearchRadius = iRadius;
}


void PlaneSegmenter::
setMinPoints(const int iMin) {
  mMinPoints = iMin;
}

PlaneSegmenter::Result PlaneSegmenter::
go() {
  Result result;
  const int n = mCloud->size();

  // create kdtree and get nearest neighbors list
  pcl::search::KdTree<Point>::Ptr tree
    (new pcl::search::KdTree<Point>());
  tree->setInputCloud(mCloud);
  std::vector<std::vector<int>> neighbors(n);
  std::vector<float> distances;
  for (int i = 0; i < n; ++i) { 
    tree->radiusSearch(i, mSearchRadius, neighbors[i], distances);
    auto& neigh = neighbors[i];
    std::vector<std::pair<int,float>> pairs(neigh.size());
    for (int j = 0; j < (int)neigh.size(); ++j) {
      pairs[j].first = neigh[j];
      pairs[j].second = distances[j];
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const std::pair<int,float>& iA,
                 const std::pair<int,float>& iB){
                return iA.second<iB.second;});
    for (int j = 0; j < (int)neigh.size(); ++j) {
      neigh[j] = pairs[j].first;
    }
  }

  // hitmask
  std::vector<bool> hitMask(n);
  for (int i = 0; i < n; ++i) {
    hitMask[i] = (mNormals->points[i].curvature < 0);
  }

  // create labels
  std::vector<int> labels(n);
  std::fill(labels.begin(), labels.end(), 0);
  IncrementalPlaneEstimator planeEst;
  int curLabel = 1;
  std::deque<int> workQueue;

  // create label-to-plane mapping
  struct Plane {
    Eigen::Vector4f mPlane;
    int mCount;
    int mLabel;
  };
  std::vector<Plane> planes;

  auto processPoint = [&](const int iIndex) {
    if (hitMask[iIndex]) return false;
    const Eigen::Vector3f& pt = mCloud->points[iIndex].getVector3fMap();
    const auto& cloudNorm = mNormals->points[iIndex];
    const Eigen::Vector3f norm(cloudNorm.normal_x, cloudNorm.normal_y,
                               cloudNorm.normal_z);
    if (planeEst.tryPoint(pt, norm, mMaxError, mMaxAngle)) {
      labels[iIndex] = curLabel;
      hitMask[iIndex] = true;
      planeEst.addPoint(pt);
      for (const auto idx : neighbors[iIndex]) {
        if (!hitMask[idx] && (labels[idx]<=0)) workQueue.push_back(idx);
      }
      return true;
    }
    return false;
  };

  // create list of points ordered by curvature
  std::vector<int> allIndices;
  allIndices.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (!hitMask[i]) allIndices.push_back(i);
  }
  std::sort(allIndices.begin(), allIndices.end(),
            [this](const int iA, const int iB)
            { return mNormals->points[iA].curvature <
              mNormals->points[iB].curvature; });

  // iterate over points
  for (const auto idx : allIndices) {
    if (hitMask[idx]) continue;
    if (labels[idx] > 0) continue;

    // start new component
    planeEst.reset();
    workQueue.clear();
    workQueue.push_back(idx);

    while (workQueue.size() > 0) {
      processPoint(workQueue.front());
      workQueue.pop_front();
    }

    // add new plane
    Plane plane;
    plane.mPlane = planeEst.getCurrentPlane();
    plane.mCount = planeEst.getNumPoints();
    plane.mLabel = curLabel;
    planes.push_back(plane);

    ++curLabel;
  }

  // second pass

  // unlabel small components
  std::sort(planes.begin(), planes.end(),
            [](const Plane& iA, const Plane& iB) {
              return iA.mCount>iB.mCount;});
  std::vector<int> lut(curLabel);
  for (int i = 0; i < (int)lut.size(); ++i) lut[i] = i;
  for (int i = 0; i < (int)planes.size(); ++i) {
    if (planes[i].mCount < mMinPoints) lut[planes[i].mLabel] = -1;
  }
  for (int i = 0; i < n; ++i) {
    if (!hitMask[i]) continue;
    int newLabel = lut[labels[i]];
    if (newLabel < 0) {
      hitMask[i] = false;
      labels[i] = 0;
    }
  }

  // remap labels
  std::unordered_map<int,Plane> planeMap;
  for (const auto& plane : planes) planeMap[plane.mLabel] = plane;
  int counter = 1;
  std::unordered_map<int,Eigen::Vector4f> planeMapNew;
  for (int& idx : lut) {
    if (idx <= 0) continue;
    Plane plane = planeMap[idx];
    idx = counter++;
    plane.mLabel = idx;
    planeMapNew[idx] = plane.mPlane;
  }
  for (int i = 0; i < n; ++i) {
    auto& label = labels[i];
    label = lut[label];
  }
    
  result.mLabels = labels;
  result.mPlanes = planeMapNew;
  return result;
}
