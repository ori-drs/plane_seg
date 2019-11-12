#include "plane_seg/RectangleFitter.hpp"

#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

using namespace planeseg;

RectangleFitter::
RectangleFitter() {
  setDimensions(Eigen::Vector2f(0,0));
  setAlgorithm(Algorithm::MinimumArea);
}

void RectangleFitter::
setDimensions(const Eigen::Vector2f& iSize) {
  mRectangleSize = iSize;
}

void RectangleFitter::
setAlgorithm(const Algorithm iAlgorithm) {
  mAlgorithm = iAlgorithm;
}

void RectangleFitter::
setData(const MatrixX3f& iPoints, const Eigen::Vector4f& iPlane) {
  mPoints = iPoints;
  mPlane = iPlane;
}

RectangleFitter::Result
RectangleFitter::go() {
  // project points onto plane
  Eigen::VectorXf distances = (mPoints*mPlane.head<3>()).array() + mPlane[3];
  MatrixX3f points = mPoints - distances*mPlane.head<3>().transpose();

  // copy points into cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZ>());
  cloud->resize(points.rows());
  for (int i = 0; i < (int)points.rows(); ++i) {
    cloud->points[i].getVector3fMap() = points.row(i);
  }

  // compute convex hull
  pcl::ConvexHull<pcl::PointXYZ> chull;
  pcl::PointCloud<pcl::PointXYZ> hull;
  chull.setInputCloud(cloud);
  chull.reconstruct(hull);

  // convenience structure for evaluating each candidate
  struct Entry {
    int mEdgeIndex;
    Eigen::Vector3f mPointMin;
    Eigen::Vector3f mPointMax;
    Eigen::Isometry3f mTransform;
    float mArea;
    Entry() : mEdgeIndex(-1), mPointMin(0,0,0), mPointMax(0,0,0),
              mTransform(Eigen::Isometry3f::Identity()), mArea(0) {}
  };
  Entry bestEntry;
  float bestScore = 1e10;

  // loop over and evaluate each convex hull edge
  int n = hull.size();
  pcl::PointCloud<pcl::PointXYZ> flatCloud;
  for (int i = 0; i < n; ++i) {

    // get edge endpoints
    Eigen::Vector3f p0 = hull.points[i].getVector3fMap();
    Eigen::Vector3f p1 = hull.points[(i+1)%n].getVector3fMap();

    // check for repeated point (thus invalid edge direction)
    if ((p1-p0).norm() < 1e-6) continue;

    // transform points to plane parallel to z=0
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    Eigen::Matrix3f rot;
    rot.col(2) = mPlane.head<3>();
    rot.col(0) = (p1-p0).normalized();
    rot.col(1) = rot.col(2).cross(rot.col(0)).normalized();
    transform.translation() = p0;
    transform.linear() = rot;
    transform = transform.inverse();
    pcl::transformPointCloud(hull, flatCloud,
                             Eigen::Affine3f(transform.matrix()));

    // find extents
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(flatCloud, minPoint, maxPoint);

    // create entry for this edge
    Entry entry;
    entry.mEdgeIndex = i;
    entry.mPointMin << minPoint[0], minPoint[1], 0;
    entry.mPointMax << maxPoint[0], maxPoint[1], 0;
    entry.mArea = (entry.mPointMax-entry.mPointMin).head<2>().prod();
    entry.mTransform = transform;

    // compute score
    float score;

    // score is closest area to prior size area
    if (mAlgorithm == Algorithm::ClosestToPriorSize) {
      Eigen::Vector2f size = (entry.mPointMax-entry.mPointMin).head<2>();
      score = (size-mRectangleSize).norm();
    }

    // score is simply minimum area
    else if (mAlgorithm == Algorithm::MinimumArea) {
      score = entry.mArea;
    }

    // score based on how much of the convex hull is near the perimeter
    else {
      score = 0;
      // TODO
    }

    // replace best score
    if (score < bestScore) {
      bestEntry = entry;
      bestScore = score;
    }
  }

  // construct rectangle corners
  Eigen::Vector3f center = 0.5*(bestEntry.mPointMin + bestEntry.mPointMax);
  Eigen::Vector3f size = bestEntry.mPointMax-bestEntry.mPointMin;
  if (mAlgorithm == Algorithm::ClosestToPriorSize) {
    size.head<2>() = mRectangleSize;
  }
  Eigen::Vector3f p0(center[0]-size[0]/2, center[1]-size[1]/2, 0);
  Eigen::Vector3f p1(center[0]-size[0]/2, center[1]+size[1]/2, 0);
  Eigen::Vector3f p2(center[0]+size[0]/2, center[1]+size[1]/2, 0);
  Eigen::Vector3f p3(center[0]+size[0]/2, center[1]-size[1]/2, 0);

  // compute area of convex hull
  float convexArea = 0;
  for (int i = 0; i < (int)flatCloud.size(); ++i) {
    Eigen::Vector2f p1, p2;
    p1 = flatCloud.points[i].getVector3fMap().head<2>();
    p2 = flatCloud.points[(i+1)%flatCloud.size()].getVector3fMap().head<2>();
    convexArea += std::abs(p1[0]*p2[1] - p1[1]*p2[0]);
  }
  convexArea /= 2;

  // fill result structure
  Result result;
  result.mPlane = mPlane;
  result.mPolygon = { p0, p1, p2, p3 };
  result.mSize = size.head<2>();
  result.mArea = bestEntry.mArea;
  result.mConvexArea = convexArea;
  Eigen::Isometry3f transformInv = bestEntry.mTransform.inverse();
  result.mPose = transformInv;
  result.mPose.translation() = transformInv*center;
  for (auto& pt : result.mPolygon) pt = transformInv*pt;
  for (auto& pt : hull.points) {
    result.mConvexHull.push_back(pt.getVector3fMap());
  }

  // adjust result so that max dimension matches prior size
  if (mRectangleSize.norm() > 1e-5) {
    const auto& size1 = result.mSize;
    const auto& size2 = mRectangleSize;
    if (((size1[1] > size1[0]) && (size2[0] > size2[1])) ||
        ((size1[0] > size1[1]) && (size2[1] > size2[0]))) {
      Eigen::AngleAxisf angleAxis(M_PI/2, Eigen::Vector3f::UnitZ());
      result.mPose.linear() *= angleAxis.matrix();
      std::swap(result.mSize[0], result.mSize[1]);
    }
  }

  return result;
}
