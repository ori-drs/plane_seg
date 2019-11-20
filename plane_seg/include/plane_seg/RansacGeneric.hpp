#ifndef _drc_RansacGeneric_hpp_
#define _drc_RansacGeneric_hpp_

#include <cmath>
#include <vector>
#include <algorithm>

namespace drc {

template<typename Problem>
class RansacGeneric {
public:
  struct Result {
    bool mSuccess;
    typename Problem::Solution mSolution;
    std::vector<int> mInliers;
    int mNumIterations;
  };

public:
  RansacGeneric() {
    setMaximumIterations(5000);
    setSkippedIterationFactor(1);
    setGoodSolutionProbability(1-1e-8);
    setRefineUsingInliers(false);
    setMaximumError(-1);
  }

  virtual ~RansacGeneric() {}

  void setMaximumIterations(const int iIters) { mMaximumIterations = iIters; }
  void setSkippedIterationFactor(const double iFactor) {
    mSkippedIterationFactor = iFactor;
  }
  void setGoodSolutionProbability(const double iProb) {
    mGoodSolutionProbability = iProb;
  }
  void setRefineUsingInliers(const bool iVal) { mRefineUsingInliers = iVal; }
  void setMaximumError(const double iVal) { mMaximumError = iVal; }

  Result solve(const Problem& iProblem) const {
    // set up initial (empty) result
    Result result;
    result.mSuccess = false;
    result.mNumIterations = 0;

    // ensure that there are enough data points to proceed
    const int sampleSize = iProblem.getSampleSize();
    const int n = iProblem.getNumDataPoints();
    if (n < sampleSize) {
      return result;
    }

    const double epsilon = 1e-10;

    // best results are currently invalid
    int bestScore = 0;
    bool success = false;

    // start number of iterations as infinite, then reduce as we go
    double numIterationsNeeded = 1e10;
    int iterationCount = 0;
    int skippedSampleCount = 0;

    // for random sample index generation
    std::vector<int> allIndices(n);

    // iterate until adaptive number of iterations are exceeded
    while (iterationCount < numIterationsNeeded) {

      // determine random sample indices
      for (int i = 0; i < n; ++i) {
        allIndices[i] = i;
      }
      for (int i = 0; i < sampleSize; ++i) {
        int randIndex = std::rand() % n;
        std::swap(allIndices[i], allIndices[randIndex]);
      }
      std::vector<int> sampleIndices(allIndices.begin(),
                                     allIndices.begin() + sampleSize);

      // compute solution on minimal set
      typename Problem::Solution solution = iProblem.estimate(sampleIndices);

      // compute errors over all data points
      std::vector<double> errors2 = iProblem.computeSquaredErrors(solution);

      // check whether this is a valid sample
      // TODO: this should be done via a method in Problem class, but would
      // require changing all existing usages to include that method
      if (errors2.size() == 0) {
        ++skippedSampleCount;
        if (skippedSampleCount >=
            mMaximumIterations*mSkippedIterationFactor) break;
        continue;
      }
      skippedSampleCount = 0;

      // compute error threshold to be applied to each term
      double thresh = mMaximumError;
      if (thresh < 0) {
        std::sort(errors2.begin(), errors2.end());
        double median = (n % 2 == 0) ?
          (0.5*(errors2[n/2]+errors2[n/2+1])) : errors2[n/2];
        thresh = 1.4826*std::sqrt(median)*4.6851;
      }
      thresh *= thresh;

      // determine inliers
      std::vector<int> inliers;
      inliers.reserve(n);
      for (int i = 0; i < n; ++i) {
        if (errors2[i] <= thresh) {
          inliers.push_back(i);
        }
      }

      // if this is the best score, update solution and convergence criteria
      int score = inliers.size();
      if (score > bestScore) {
        bestScore = score;
        result.mInliers = inliers;
        result.mSolution = solution;
        success = true;
        double inlierProbability = double(inliers.size()) / n;
        double anyOutlierProbability = 1 - pow(inlierProbability,sampleSize);
        anyOutlierProbability = std::min(anyOutlierProbability, 1-epsilon);
        anyOutlierProbability = std::max(anyOutlierProbability, epsilon);
        numIterationsNeeded =
          log(1-mGoodSolutionProbability) / log(anyOutlierProbability);
      }

      // bump up iteration count and terminate if it exceeds hard max
      ++iterationCount;
      if (iterationCount > mMaximumIterations) {
        break;
      }
    }

    // finish off result params
    result.mSuccess = success;
    result.mNumIterations = iterationCount;

    // refine result using all inliers if specified
    if (result.mSuccess && mRefineUsingInliers) {
      result.mSolution = iProblem.estimate(result.mInliers);
    }

    // done
    return result;
  }

protected:
  bool mRefineUsingInliers;
  int mMaximumIterations;
  double mSkippedIterationFactor;
  double mGoodSolutionProbability;
  double mMaximumError;
};

}

#endif
