#include "plane_seg/PlaneFitter.hpp"

#include <limits>

#include <plane_seg/RansacGeneric.hpp>

using namespace planeseg;

namespace {

struct SimpleProblemBase {
  struct Solution {
    Eigen::Vector4f mPlane;
    float mCurvature;
    Eigen::Vector3f mCenterPoint;
  };

  MatrixX3f mPoints;
  Eigen::Vector3f mCenterPoint;
  bool mCheckNormal = false;
  Eigen::Vector3f mNormalPrior = Eigen::Vector3f(0,0,0);
  float mMaxAngleDeviation = 0;

  SimpleProblemBase(const std::vector<Eigen::Vector3f>& iPoints) {
    mPoints.resize(iPoints.size(),3);
    for (int i = 0; i < (int)iPoints.size(); ++i) {
      mPoints.row(i) = iPoints[i];
    }
  }
  int getSampleSize() const { return 3; }
  int getNumDataPoints() const { return mPoints.rows(); }

  Solution estimate(const std::vector<int>& iIndices) const {
    Solution sol;
    const int n = iIndices.size();
    if (n == 3) {
      Eigen::Vector3f p1 = mPoints.row(iIndices[0]);
      Eigen::Vector3f p2 = mPoints.row(iIndices[1]);
      Eigen::Vector3f p3 = mPoints.row(iIndices[2]);
      sol.mPlane.head<3>() = ((p3-p1).cross(p2-p1)).normalized();
      sol.mPlane[3] = -sol.mPlane.head<3>().dot(p1);
      sol.mCurvature = 0;
      float centerDist = sol.mPlane.head<3>().dot(mCenterPoint) + sol.mPlane[3];
      if (std::abs(centerDist) > 0.02f)  sol.mPlane[3] = 1e10;
    }
    else {
      sol = estimateFull(iIndices);
    }
    return sol;
  }

  Solution estimateFull(const std::vector<int>& iIndices) const {
    Solution sol;
    const int n = iIndices.size();
    MatrixX3f data(n,3);
    for (int i = 0; i < n; ++i) data.row(i) = mPoints.row(iIndices[i]);
    Eigen::Vector3f avg = data.colwise().mean();
    data.rowwise() -= avg.transpose();
    auto svd = data.jacobiSvd(Eigen::ComputeFullV);
    sol.mPlane.head<3>() = svd.matrixV().col(2);
    sol.mPlane[3] = -sol.mPlane.head<3>().dot(avg);
    sol.mCurvature = svd.singularValues()[2]/n;
    sol.mCenterPoint = avg;
    return sol;
  }
  
  std::vector<double> computeSquaredErrors(const Solution& iSolution) const {
    const auto& plane = iSolution.mPlane;
    Eigen::Vector3f normal = plane.head<3>();
    if (mCheckNormal) {
      float dot = std::abs(normal.dot(mNormalPrior));
      if (dot < std::cos(mMaxAngleDeviation)) {
        return std::vector<double>();
      }
    }
    Eigen::VectorXf errors = (mPoints*normal).array() + plane[3];
    std::vector<double> errors2(errors.size());
    for (int i = 0; i < (int)errors2.size(); ++i) {
      errors2[i] = errors[i]*errors[i];
    }
    return errors2;
  }
};

struct SimpleProblem : public SimpleProblemBase {
  SimpleProblem(const std::vector<Eigen::Vector3f>& iPoints) :
    SimpleProblemBase(iPoints) {}

  int getSampleSize() const { return 2; }

  Solution estimate(const std::vector<int>& iIndices) const {
    Solution sol;
    const int n = iIndices.size();
    if (n == 2) {
      Eigen::Vector3f p1 = mPoints.row(iIndices[0]);
      Eigen::Vector3f p2 = mPoints.row(iIndices[1]);
      const Eigen::Vector3f& p3 = mCenterPoint;
      sol.mPlane.head<3>() = ((p3-p1).cross(p2-p1)).normalized();
      sol.mPlane[3] = -sol.mPlane.head<3>().dot(p1);
      sol.mCurvature = 0;
    }
    else {
      sol = estimateFull(iIndices);
    }
    return sol;
  }

};

}


PlaneFitter::
PlaneFitter() {
  setMaxDistance(0.01);
  setMaxIterations(100);
  setRefineUsingInliers(true);
  float badValue = std::numeric_limits<float>::infinity();
  setCenterPoint(Eigen::Vector3f(badValue, badValue, badValue));
  setNormalPrior(Eigen::Vector3f(0,0,0), 2*M_PI);
}

PlaneFitter::
~PlaneFitter() {
}

void PlaneFitter::
setMaxDistance(const float iDistance) {
  mMaxDistance = iDistance;
}

void PlaneFitter::
setCenterPoint(const Eigen::Vector3f& iPoint) {
  mCenterPoint = iPoint;
}

void PlaneFitter::
setMaxIterations(const int iIterations, const float iSkipFactor) {
  mMaxIterations = iIterations;
  mSkippedIterationFactor = iSkipFactor;
}

void PlaneFitter::
setRefineUsingInliers(const bool iVal) {
  mRefineUsingInliers = iVal;
}

void PlaneFitter::
setNormalPrior(const Eigen::Vector3f& iNormal,
               const float iMaxAngleDeviation) {
  mNormalPrior = iNormal;
  if (mNormalPrior.norm() > 1e-5) mNormalPrior.normalize();
  mMaxAngleDeviation = iMaxAngleDeviation;
  mCheckNormal = (iNormal.norm() > 1e-5) && (iMaxAngleDeviation < M_PI);
}

PlaneFitter::Result PlaneFitter::
go(const std::vector<Eigen::Vector3f>& iPoints) const {
  if (std::isinf(mCenterPoint[0])) return solve<SimpleProblemBase>(iPoints);
  else return solve<SimpleProblem>(iPoints);
}

template<typename T>
PlaneFitter::Result PlaneFitter::
solve(const std::vector<Eigen::Vector3f>& iPoints) const {
  Result result;
  drc::RansacGeneric<T> ransac;
  ransac.setMaximumError(mMaxDistance);
  ransac.setRefineUsingInliers(mRefineUsingInliers);
  ransac.setMaximumIterations(mMaxIterations);
  ransac.setSkippedIterationFactor(mSkippedIterationFactor);

  T problem(iPoints);
  problem.mCenterPoint = mCenterPoint;
  problem.mCheckNormal = mCheckNormal;
  problem.mNormalPrior = mNormalPrior;
  problem.mMaxAngleDeviation = mMaxAngleDeviation;

  auto res = ransac.solve(problem);
  result.mSuccess = res.mSuccess;
  result.mPlane = res.mSolution.mPlane;
  result.mCenterPoint = res.mSolution.mCenterPoint;
  result.mInliers = res.mInliers;
  result.mCurvature = res.mSolution.mCurvature;

  return result;
}
