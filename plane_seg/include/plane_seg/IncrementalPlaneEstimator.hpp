#ifndef _planeseg_IncrementalPlaneEstimator_hpp_
#define _planeseg_IncrementalPlaneEstimator_hpp_

#include <vector>
#include "Types.hpp"

namespace planeseg {

class IncrementalPlaneEstimator {
protected:
  std::vector<Eigen::Vector3f> mPoints;
  Eigen::Vector3d mSum;
  Eigen::Matrix3d mSumSquared;
  int mCount;

protected:
  Eigen::Vector4f getPlane(const Eigen::Vector3d& iSum,
                           const Eigen::Matrix3d& iSumSquared,
                           const double iCount);

  inline float computeError(const Eigen::Vector4f& iPlane,
                            const Eigen::Vector3f& iPoint) {
    float e = iPoint.dot(iPlane.head<3>()) + iPlane[3];
    return e*e;
  }


public:
  IncrementalPlaneEstimator();

  void reset();
  int getNumPoints() const;
  void addPoint(const Eigen::Vector3f& iPoint);

  std::vector<float>
  computeErrors(const Eigen::Vector4f& iPlane,
                const std::vector<Eigen::Vector3f>& iPoints);

  bool tryPoint(const Eigen::Vector3f& iPoint, const float iMaxError);
  bool tryPoint(const Eigen::Vector3f& iPoint, const Eigen::Vector3f& iNormal,
                const float iMaxError, const float iMaxAngle);

  Eigen::Vector4f getCurrentPlane();
};

}

#endif
