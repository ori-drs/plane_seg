#ifndef _planeseg_RobustNormalEstimator_hpp_
#define _planeseg_RobustNormalEstimator_hpp_

#include "Types.hpp"

namespace planeseg {

class RobustNormalEstimator {
public:
  RobustNormalEstimator();

  void setRadius(const float iRadius);
  void setMaxEstimationError(const float iDist);
  void setMaxCenterError(const float iDist);
  void setMaxIterations(const int iIters);
  void computeCurvature(const bool iVal);

  bool go(const LabeledCloud::Ptr& iCloud, NormalCloud& oNormals);

protected:
  float mRadius;
  float mMaxEstimationError;
  float mMaxCenterError;
  int mMaxIterations;
  bool mComputeCurvature;
};

}

#endif
