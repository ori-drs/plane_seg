#ifndef _planeseg_PlaneSegmenter_hpp_
#define _planeseg_PlaneSegmenter_hpp_

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

#include "Types.hpp"

namespace planeseg {

class PlaneSegmenter {
public:
  struct Result {
    std::vector<int> mLabels;
    std::unordered_map<int,Eigen::Vector4f> mPlanes;
  };

public:
  PlaneSegmenter();

  void setData(const LabeledCloud::Ptr& iCloud,
               const NormalCloud::Ptr& iNormals);

  void setMaxError(const float iError);
  void setMaxAngle(const float iAngle);
  void setSearchRadius(const float iRadius);
  void setMinPoints(const int iMin);

  Result go();

protected:
  LabeledCloud::Ptr mCloud;
  NormalCloud::Ptr mNormals;
  float mMaxError;
  float mMaxAngle;
  float mSearchRadius;
  int mMinPoints;
};

}

#endif
