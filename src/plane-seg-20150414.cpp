#include <string>
#include <fstream>
#include <limits>
#include <deque>
#include <unordered_map>

#include <maps/DepthImageView.hpp>
#include <maps/DepthImage.hpp>
#include <maps/Utils.hpp>
#include <drc_utils/RansacGeneric.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/opencv.hpp>

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointXYZL> LabeledCloud;

class IncrementalPlaneEstimator {
protected:
  std::vector<Eigen::Vector3f> mPoints;
  Eigen::Vector3d mSum;
  Eigen::Matrix3d mSumSquared;
  int mCount;

protected:
  Eigen::Vector4f getPlane(const Eigen::Vector3d& iSum,
                           const Eigen::Matrix3d& iSumSquared,
                           const double iCount) {
    Eigen::Vector3d mean = iSum/iCount;
    Eigen::Matrix3d cov = iSumSquared/iCount - mean*mean.transpose();
    Eigen::Vector4d plane;
    plane.head<3>() = cov.jacobiSvd(Eigen::ComputeFullV).matrixV().col(2);
    plane[3] = -plane.head<3>().dot(mean);
    return plane.cast<float>();
  }    

  inline float computeError(const Eigen::Vector4f& iPlane,
                            const Eigen::Vector3f& iPoint) {
    float e = iPoint.dot(iPlane.head<3>()) + iPlane[3];
    return e*e;
  }


public:
  IncrementalPlaneEstimator() {
    reset();
  }

  void reset() {
    mSum.setZero();
    mSumSquared.setZero();
    mPoints.clear();
    mCount = 0;
  }

  int getNumPoints() const { return mPoints.size(); }

  void addPoint(const Eigen::Vector3f& iPoint) {
    mPoints.push_back(iPoint);
    Eigen::Vector3d p = iPoint.cast<double>();
    mSum += p;
    mSumSquared += p*p.transpose();
  }

  std::vector<float>
  computeErrors(const Eigen::Vector4f& iPlane,
                const std::vector<Eigen::Vector3f>& iPoints) {
    const int n = iPoints.size();
    std::vector<float> errors(n);
    for (int i = 0; i < n; ++i) {
      errors[i] = computeError(iPlane, iPoints[i]);
    }
    return errors;
  }

  bool tryPoint(const Eigen::Vector3f& iPoint, const float iMaxError) {
    const int n = mPoints.size();
    if (n <= 2) return true;
    Eigen::Vector3d p = iPoint.cast<double>();
    Eigen::Vector3d sum = mSum+p;
    Eigen::Matrix3d sumSquared = mSumSquared + p*p.transpose();
    Eigen::Vector4f plane = getPlane(sum, sumSquared, n+1);
    std::vector<float> errors2 = computeErrors(plane, mPoints);
    errors2.push_back(computeError(plane, iPoint));
    float thresh2 = iMaxError*iMaxError;
    int numInliers = 0;
    float totalError2 = 0;
    for (float e2 : errors2) {
      totalError2 += e2;
      numInliers += (e2<=thresh2);
    }
    return numInliers==(n+1);
  }

  bool tryPoint(const Eigen::Vector3f& iPoint, const Eigen::Vector3f& iNormal,
                const float iMaxError, const float iMaxAngle) {
    const int n = mPoints.size();
    if (n < 2) return true;

    Eigen::Vector3d p = iPoint.cast<double>();
    Eigen::Vector3d sum = mSum+p;
    Eigen::Matrix3d sumSquared = mSumSquared + p*p.transpose();
    Eigen::Vector4f plane = getPlane(sum, sumSquared, n+1);
    if (std::abs(plane.head<3>().cast<float>().dot(iNormal)) <
        std::cos(iMaxAngle)) return false;

    std::vector<float> prevErrors2 = computeErrors(getCurrentPlane(), mPoints);
    float prevTotalError2 =
      std::accumulate(prevErrors2.begin(), prevErrors2.end(), 0.0f);

    std::vector<float> errors2 = computeErrors(plane, mPoints);
    errors2.push_back(computeError(plane, iPoint));
    float thresh2 = iMaxError*iMaxError;
    int numInliers = 0;
    float totalError2 = 0;
    for (float e2 : errors2) {
      totalError2 += e2;
      numInliers += (e2<=thresh2);
    }
    float deltaError2 = totalError2/(n+1) - prevTotalError2/n;
    return deltaError2 < thresh2/n;
    //return numInliers==(n+1);
  }


  Eigen::Vector4f getCurrentPlane() {
    return getPlane(mSum, mSumSquared, mPoints.size());
  }
};

class PlaneSegmenter {
public:
  struct Result {
    cv::Mat mLabels;
  };

protected:
  LabeledCloud::Ptr mCloud;
  NormalCloud::Ptr mNormals;
  float mMaxError;
  float mMaxAngle;
  int mMinPoints;

public:
  PlaneSegmenter() {
    setMaxError(0.02);
    setMaxAngle(30);
    setMinPoints(500);
  }

  void setData(const LabeledCloud::Ptr& iCloud,
               const NormalCloud::Ptr& iNormals) {
    mCloud = iCloud;
    mNormals = iNormals;
  }

  void setMaxError(const float iError) {
    mMaxError = iError;
  }

  void setMaxAngle(const float iAngle) {
    mMaxAngle = iAngle*M_PI/180;
  }

  void setMinPoints(const int iMin) {
    mMinPoints = iMin;
  }

  Result go() {
    Result result;

    // create initial hit mask
    int width = mCloud->width;
    int height = mCloud->height;
    cv::Mat hitMask(height, width, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        if (!std::isfinite(mCloud->points[i*width+j].z)) {
          hitMask.at<uint8_t>(i,j) = 255;
        }
      }
    }

    // create label image
    cv::Mat labels(height, width, CV_32SC1, cv::Scalar(0));
    IncrementalPlaneEstimator planeEst;
    int curLabel = 1;
    std::deque<Eigen::Vector2i> workQueue;

    // create label-to-plane mapping
    struct Plane {
      Eigen::Vector4f mPlane;
      int mCount;
      int mLabel;
    };
    std::vector<Plane> planes;

    auto processPoint = [&](const Eigen::Vector2i& iPoint) {
      const auto& x = iPoint[0];
      const auto& y = iPoint[1];
      const int w(hitMask.cols), h(hitMask.rows);
      if ((x<0) || (x>=w) || (y<0) || (y>=h)) return false;
      if ((hitMask.at<uint8_t>(y,x) > 0) ||
          (labels.at<int32_t>(y,x) > 0)) return false;
      int idx = y*w + x;
      const Eigen::Vector3f& pt = mCloud->points[idx].getVector3fMap();
      const auto& cloudNorm = mNormals->points[idx];
      const Eigen::Vector3f norm(cloudNorm.normal_x, cloudNorm.normal_y,
                                 cloudNorm.normal_z);
      if (planeEst.tryPoint(pt, norm, mMaxError, mMaxAngle)) {
        labels.at<int32_t>(y,x) = curLabel;
        hitMask.at<uint8_t>(y,x) = 255;
        planeEst.addPoint(pt);
        workQueue.push_back(Eigen::Vector2i(x+1,y));
        workQueue.push_back(Eigen::Vector2i(x,y-1));
        workQueue.push_back(Eigen::Vector2i(x-1,y));
        workQueue.push_back(Eigen::Vector2i(x,y+1));
        /*
        workQueue.push_back(Eigen::Vector2i(x+1,y+1));
        workQueue.push_back(Eigen::Vector2i(x-1,y+1));
        workQueue.push_back(Eigen::Vector2i(x+1,y-1));
        workQueue.push_back(Eigen::Vector2i(x-1,y-1));
        */
        return true;
      }
      return false;
    };

    // create ordered list of points
    struct Point {
      int mX;
      int mY;
      float mVal;
    };
    std::vector<Point> allPoints;
    allPoints.reserve(width*height);
    for (int i = 0, idx = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j, ++idx) {
        if (hitMask.at<uint8_t>(i,j) == 0) {
          Point pt;
          pt.mX = j;
          pt.mY = i;
          pt.mVal = mNormals->points[i*width+j].curvature;
          allPoints.push_back(pt);
        }
      }
    }
    std::sort(allPoints.begin(), allPoints.end(),
              [](const Point& iA, const Point& iB)
              { return iA.mVal<iB.mVal; });

    // iterate over pixels
    for (const auto& p : allPoints) {
      if (hitMask.at<uint8_t>(p.mY,p.mX) > 0) continue;
      if (labels.at<int32_t>(p.mY,p.mX) > 0) continue;

      // start new component
      planeEst.reset();
      workQueue.clear();
      workQueue.push_back(Eigen::Vector2i(p.mX,p.mY));

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
    std::vector<int> lut(planes.size());
    for (int i = 0; i < (int)planes.size(); ++i) {
      int newLabel = (planes[i].mCount > mMinPoints) ? planes[i].mLabel : -1;
      lut[planes[i].mLabel] = newLabel;
    }
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        if (hitMask.at<uint8_t>(i,j) == 0) continue;
        int newLabel = lut[labels.at<int32_t>(i,j)];
        if (newLabel < 0) {
          hitMask.at<uint8_t>(i,j) = 0;
          labels.at<int32_t>(i,j) = 0;
        }
      }
    }
    
    result.mLabels = labels;
    return result;
  }
};

struct SimpleProblem {
  typedef Eigen::Vector4f Solution;
  Eigen::Matrix<float,Eigen::Dynamic,3> mPoints;
  SimpleProblem(const std::vector<Eigen::Vector3f>& iPoints) {
    mPoints.resize(iPoints.size(),3);
    for (int i = 0; i < (int)iPoints.size(); ++i) {
      mPoints.row(i) = iPoints[i];
    }
  }
  int getSampleSize() const { return 3; }
  int getNumDataPoints() const { return mPoints.rows(); }

  Solution estimate(const std::vector<int>& iIndices) const {
    Solution sol;
    if (iIndices.size() == 3) {
      Eigen::Vector3f p1 = mPoints.row(iIndices[0]);
      Eigen::Vector3f p2 = mPoints.row(iIndices[1]);
      Eigen::Vector3f p3 = mPoints.row(iIndices[2]);
      sol.head<3>() = ((p3-p1).cross(p2-p1)).normalized();
      sol[3] = -sol.head<3>().dot(p1);
    }
    else {
      Eigen::Matrix<float,Eigen::Dynamic,3> data(iIndices.size(),3);
      for (int i = 0; i < (int)iIndices.size(); ++i) {
        data.row(i) = mPoints.row(iIndices[i]);
      }
      Eigen::Vector3f avg = data.colwise().mean();
      data.rowwise() -= avg.transpose();
      sol.head<3>() = data.jacobiSvd(Eigen::ComputeFullV).matrixV().col(2);
      sol[3] = -sol.head<3>().dot(avg);
    }
    return sol;
  }
  
  std::vector<double> computeSquaredErrors(const Solution& iSolution) const {
    Eigen::VectorXf errors =
      (mPoints*iSolution.head<3>()).array() + iSolution[3];
    std::vector<double> errors2(errors.size());
    for (int i = 0; i < (int)errors2.size(); ++i) {
      errors2[i] = errors[i]*errors[i];
    }
    return errors2;
  }


};

struct Problem {
  typedef Eigen::Vector4f Solution;

  Eigen::Matrix<float,Eigen::Dynamic,3> mPoints;
  Eigen::Matrix<float,Eigen::Dynamic,3> mNormals;
  float mAngleThresh;
  Eigen::Vector3f mOrigin;
  const bool mSinglePoint = false;

  Problem(const LabeledCloud::Ptr& iCloud,
          const NormalCloud::Ptr& iNormals,
          const float iAngleThreshDegrees,
          const Eigen::Vector3f& iOrigin) {
    const int n = iCloud->size();
    mPoints.resize(n,3);
    mNormals.resize(n,3);
    mAngleThresh = iAngleThreshDegrees*M_PI/180;
    mOrigin = iOrigin;
    for (int i = 0; i < n; ++i) {
      mPoints.row(i) = iCloud->points[i].getVector3fMap();
      const auto& normal = iNormals->points[i];
      mNormals.row(i) << normal.normal_x, normal.normal_y, normal.normal_z;
      mNormals.row(i).normalize();
    }
  }

  int getSampleSize() const { return mSinglePoint ? 1 : 3; }
  int getNumDataPoints() const { return mPoints.rows(); }

  Solution estimate(const std::vector<int>& iIndices) const {
    Solution sol;
    if (mSinglePoint) {
      sol.head<3>() = mNormals.row(iIndices[0]);
      sol[3] = -sol.head<3>().dot(mPoints.row(iIndices[0]));
    }
    else {
      if (iIndices.size() == 3) {
        Eigen::Vector3f p1 = mPoints.row(iIndices[0]);
        Eigen::Vector3f p2 = mPoints.row(iIndices[1]);
        Eigen::Vector3f p3 = mPoints.row(iIndices[2]);
        sol.head<3>() = ((p3-p1).cross(p2-p1)).normalized();
        sol[3] = -sol.head<3>().dot(p1);
      }
      else {
        Eigen::Matrix<float,Eigen::Dynamic,3> data(iIndices.size(),3);
        for (int i = 0; i < (int)iIndices.size(); ++i) {
          data.row(i) = mPoints.row(iIndices[i]);
        }
        Eigen::Vector3f avg = data.colwise().mean();
        data.rowwise() -= avg.transpose();
        sol.head<3>() = data.jacobiSvd(Eigen::ComputeFullV).matrixV().col(2);
        sol[3] = -sol.head<3>().dot(avg);
      }
      if (sol.head<3>().dot(mOrigin) < 0) sol = -sol;
    }
    return sol;
  }

  std::vector<double> computeSquaredErrors(const Solution& iSolution) const {
    Eigen::VectorXf errors =
      (mPoints*iSolution.head<3>()).array() + iSolution[3];
    Eigen::VectorXf dots = mNormals*iSolution.head<3>();
    std::vector<double> errors2(errors.size());
    for (int i = 0; i < (int)errors2.size(); ++i) {
      float angle = std::acos(dots[i]);
      if (angle < mAngleThresh) errors2[i] = errors[i]*errors[i];
      else errors2[i] = 10;
    }
    return errors2;
  }

};


int main() {
  // read pcd file
  std::string inFile = "/home/antone/data/tilted-steps.pcd";
  LabeledCloud::Ptr inCloud(new LabeledCloud());
  pcl::io::loadPCDFile(inFile, *inCloud);
  std::cout << "BEFORE " << inCloud->size() << std::endl;

  // voxelize
  LabeledCloud::Ptr cloud(new LabeledCloud());
  pcl::VoxelGrid<pcl::PointXYZL> voxelGrid;
  voxelGrid.setInputCloud(inCloud);
  voxelGrid.setLeafSize (0.01f, 0.01f, 0.01f);
  voxelGrid.filter(*cloud);
  for (int i = 0; i < (int)cloud->size(); ++i) cloud->points[i].label = i;
  std::cout << "AFTER " << cloud->size() << std::endl;

  // write cloud
  pcl::io::savePCDFileBinary("/home/antone/temp/cloud.pcd", *cloud);
  

  // depth map
  //

  // pose
  Eigen::Vector3f origin(0.248091, 0.012443, 1.806473);
  cloud->sensor_origin_.head<3>() = origin;
  cloud->sensor_origin_[3] = 1;
  Eigen::Vector3f lookDir(0.837001, 0.019831, -0.546842);
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

  // calibration matrix
  int width = 500;
  int height = 500;
  Eigen::Matrix3f calib = Eigen::Matrix3f::Identity();
  float fovDegrees = 120;
  float focalLength = width/2/std::tan(fovDegrees*M_PI/180/2);
  calib(0,0) = calib(1,1) = focalLength;
  calib(0,2) = width/2;
  calib(1,2) = height/2;
  std::cout << "fov " << fovDegrees << " degrees" << std::endl;
  std::cout << "CALIB" << std::endl << calib << std::endl;

  // overall transform
  Eigen::Projective3f transform = Eigen::Projective3f::Identity();
  maps::Utils::composeViewMatrix(transform, calib, pose, false);

  // view
  maps::PointCloud::Ptr mapsCloud(new maps::PointCloud());
  pcl::copyPointCloud(*cloud, *mapsCloud);
  maps::DepthImageView::Ptr depthImageView(new maps::DepthImageView());
  depthImageView->setSize(width, height);
  depthImageView->setTransform(transform);
  depthImageView->set(mapsCloud);

  // depth image
  maps::DepthImage::Ptr depthImage = depthImageView->getDepthImage();

  // filter depth image
  {
    std::vector<float> depths =
      depthImage->getData(maps::DepthImage::TypeDepth);
    const float invalidValue =
      depthImage->getInvalidValue(maps::DepthImage::TypeDepth);
    for (auto& d : depths) if (d == invalidValue) d = 0;
    cv::Mat img(depthImage->getHeight(), depthImage->getWidth(), CV_32FC1,
                depths.data());
    for (int iter = 0; iter < 10; ++iter) {
      cv::Mat mask = (img==0);
      cv::Mat filtered;
      cv::medianBlur(img, filtered, 3);
      filtered.copyTo(img, mask);
    }
    depthImage->setData(depths, maps::DepthImage::TypeDepth);
  }

  // write depth image
  std::vector<float> depths = depthImage->getData(maps::DepthImage::TypeDepth);
  std::ofstream ofs("/home/antone/temp/depth_image.txt");
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      ofs << depths[i*width+j] << " ";
    }
    ofs << std::endl;
  }
  ofs.close();

  // create organized cloud from depth image
  LabeledCloud::Ptr organizedCloud(new LabeledCloud());
  organizedCloud->width = width;
  organizedCloud->height = height;
  organizedCloud->is_dense = false;
  organizedCloud->resize(width*height);
  Eigen::Matrix3f calibInv = calib.inverse();
  const float inf = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      float depth = depths[i*width+j];
      Eigen::Vector3f pt;
      if (depth==0) {
        pt << inf,inf,inf;
      }
      else {
        pt << j,i,1;
        Eigen::Vector3f ray = calibInv*pt;
        pt = ray/ray[2]*depth;
        // TODO: pt = pose*pt;
      }
      organizedCloud->points[i*width+j].getVector3fMap() = pt;
    }
  }
  organizedCloud->sensor_origin_.head<3>() << 0,0,0;
  organizedCloud->sensor_origin_[3] = 1;

  if (false) {
    // angle-angle range image
    const float angularResDegrees = 0.25f;
    const float angularRes = angularResDegrees*M_PI/180;
    int rangeWidth = fovDegrees/angularResDegrees;
    int rangeHeight = fovDegrees/angularResDegrees;
    std::vector<float> ranges(rangeWidth*rangeHeight);
    std::fill(ranges.begin(), ranges.end(), 1e6);
    Eigen::Isometry3f poseInv = pose.inverse();
    for (const auto& pt : cloud->points) {
      Eigen::Vector3f p = pt.getVector3fMap();
      p = poseInv*p;
      float range = p.norm();
      float angleX = atan2(p[0],p[2]);
      float angleY = asin(p[1]/range);
      int pixelX = angleX/angularRes + rangeWidth/2.0f + 0.5;
      int pixelY = angleY/angularRes + rangeHeight/2.0f + 0.5;
      if ((pixelX < 0) || (pixelY < 0) ||
          (pixelX >= rangeWidth) || (pixelY >= rangeHeight)) continue;
      int idx = pixelY*rangeWidth + pixelX;
      if (range < ranges[idx]) ranges[idx] = range;
    }
    for (auto& r : ranges) if (r > 1e3) r = 0;
  
    std::cout << "RANGE IMAGE SIZE " << rangeWidth << " " << rangeHeight << std::endl;

    // filter range image
    {
      cv::Mat img(rangeHeight, rangeWidth, CV_32FC1, ranges.data());
      for (int iter = 0; iter < 10; ++iter) {
        cv::Mat mask = (img==0);
        cv::Mat filtered;
        cv::medianBlur(img, filtered, 5);
        filtered.copyTo(img,mask);
      }
    }

    // write range image
    ofs.open("/home/antone/temp/ranges.txt");
    for (int i = 0; i < rangeHeight; ++i) {
      for (int j = 0; j < rangeWidth; ++j) {
        ofs << ranges[i*rangeWidth + j] << " ";
      }
      ofs << std::endl;
    }
    ofs.close();

    // new organized cloud
    organizedCloud.reset(new LabeledCloud());
    organizedCloud->width = rangeWidth;
    organizedCloud->height = rangeHeight;
    organizedCloud->is_dense = false;
    organizedCloud->resize(rangeWidth*rangeHeight);
    const float inf = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < rangeHeight; ++i) {
      float y = (i-rangeHeight/2.0f)*angularRes;
      float cosY = std::cos(y);
      for (int j = 0; j < rangeWidth; ++j) {
        float x = (j-rangeWidth/2.0f)*angularRes;
        float range = ranges[i*rangeWidth+j];
        Eigen::Vector3f pt;
        if (range == 0) {
          pt << inf, inf, inf;
        }
        else {
          pt << (range*std::sin(x)*cosY), (range*std::sin(y)),
            (range*std::cos(x)*cosY);
        }
        organizedCloud->points[i*rangeWidth+j].getVector3fMap() = pt;
      }
    }
  }

  width = organizedCloud->width;
  height = organizedCloud->height;

  // write organized cloud
  pcl::io::savePCDFileBinary("/home/antone/temp/organized_points.pcd",
                             *organizedCloud);

  // compute organized normals
  NormalCloud::Ptr organizedNormals(new NormalCloud());
  if (false) {
    pcl::IntegralImageNormalEstimation<pcl::PointXYZL, pcl::Normal> intNormalEst;
    intNormalEst.setNormalEstimationMethod(intNormalEst.COVARIANCE_MATRIX);
    intNormalEst.setMaxDepthChangeFactor(0.02f);
    intNormalEst.setNormalSmoothingSize(10.0f);
    intNormalEst.setInputCloud(organizedCloud);
    intNormalEst.compute(*organizedNormals);
  }
  else {

    drc::RansacGeneric<SimpleProblem> ransac;
    ransac.setMaximumError(0.01);
    ransac.setRefineUsingInliers(true);
    ransac.setMaximumIterations(100);

    std::cout << "computing organized normals manually" << std::endl;
    const int radiusPixels = 10;
    const float radiusMeters = 0.1;
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(9);
    organizedNormals->resize(width*height);
    organizedNormals->width = width;
    organizedNormals->height = height;
    organizedNormals->is_dense = false;

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        if (depths[i*width+j] == 0) continue;

        Eigen::Vector3f normal(0,0,0);
        float curvature = 0;
        auto& norm = organizedNormals->points[i*width+j];
        const Eigen::Vector3f centerPoint =
          organizedCloud->points[i*width+j].getVector3fMap();

        pts.clear();
        for (int m = -radiusPixels; m <= radiusPixels; ++m) {
          int y = i+m;
          if ((y < 0) || (y >= height)) continue;
          for (int n = -radiusPixels; n <= radiusPixels; ++n) {
            int x = j+n;
            if ((x < 0) || (x >= width)) continue;
            int idx = y*width + x;
            const float z = depths[idx];
            if (z == 0) continue;
            const Eigen::Vector3f& point =
              organizedCloud->points[idx].getVector3fMap();
            if ((point-centerPoint).norm() > radiusMeters) continue;
            pts.push_back(point);
          }
        }
        if (pts.size() >= 3) {

          SimpleProblem problem(pts);
          auto res = ransac.solve(problem);
          normal = res.mSolution.head<3>();
          curvature = 0;

          /*
          Eigen::Matrix<float,Eigen::Dynamic,3> matx(pts.size(),3);
          for (int k = 0; k < (int)pts.size(); ++k) matx.row(k) = pts[k];
          Eigen::Vector3f avg = matx.colwise().mean();
          matx.rowwise() -= avg.transpose();
          auto svd = matx.jacobiSvd(Eigen::ComputeFullV);
          normal = svd.matrixV().col(2);
          //if ((centerPoint-origin).dot(normal) > 0) normal = -normal;
          curvature = svd.singularValues()[2]/pts.size();
          */
          if (normal[2] > 0) normal = -normal;

        }
        norm.normal_x = normal[0];
        norm.normal_y = normal[1];
        norm.normal_z = normal[2];
        norm.curvature = curvature;
      }
    }

    std::cout << "done" << std::endl;
    if (false) {
      std::cout << "COMPUTING NORMALS FROM ORG CLOUD" << std::endl;
      pcl::NormalEstimation<pcl::PointXYZL, pcl::Normal> normalEst;
      normalEst.setInputCloud(organizedCloud);
      pcl::search::KdTree<pcl::PointXYZL>::Ptr tree
        (new pcl::search::KdTree<pcl::PointXYZL>());
      normalEst.setSearchMethod(tree);
      normalEst.setRadiusSearch(0.1);
      normalEst.compute(*organizedNormals);  
      std::cout << "DONE" << std::endl;
    }
  }

  // 

  // write organized normals
  pcl::io::savePCDFileBinary("/home/antone/temp/organized_normals.pcd",
                             *organizedNormals);


  // try custom plane segmentation
  PlaneSegmenter segmenter;
  segmenter.setData(organizedCloud, organizedNormals);
  segmenter.setMaxError(0.05);
  segmenter.setMaxAngle(5);
  segmenter.setMinPoints(100);
  PlaneSegmenter::Result result = segmenter.go();
  ofs.open("/home/antone/temp/labels.txt");
  for (int i = 0; i < result.mLabels.rows; ++i) {
    for (int j = 0; j < result.mLabels.cols; ++j) {
      ofs << result.mLabels.at<int32_t>(i,j) << " ";
    }
    ofs << std::endl;
  }
  ofs.close();

  // find all planes from organized cloud
  pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::Normal, pcl::Label>
    orgSeg;
  orgSeg.setMinInliers(500);
  orgSeg.setAngularThreshold(10*M_PI/180);
  orgSeg.setDistanceThreshold(0.02);
  orgSeg.setInputNormals(organizedNormals);
  orgSeg.setInputCloud(organizedCloud);

  // TODO
  std::vector<pcl::PlanarRegion<pcl::PointXYZL>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZL>>> regions;
  orgSeg.segmentAndRefine(regions);
  std::cout << "FOUND " << regions.size() << " REGIONS" << std::endl;

  std::vector<pcl::ModelCoefficients> coeffs;
  std::vector<pcl::PointIndices> inlierIndices;
  orgSeg.segment(coeffs, inlierIndices);
  std::cout << "FOUND " << coeffs.size() << " PLANES " << std::endl;

  // compute normals
  pcl::NormalEstimation<pcl::PointXYZL, pcl::Normal> normalEst;
  normalEst.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZL>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZL>());
  normalEst.setSearchMethod(tree);
  normalEst.setRadiusSearch(0.05);
  NormalCloud::Ptr normals(new NormalCloud());
  normalEst.compute(*normals);  

  // write normals
  pcl::io::savePCDFileBinary("/home/antone/temp/normals.pcd", *normals);

  // ransac
  //

  drc::RansacGeneric<Problem> ransac;
  ransac.setMaximumError(0.02);
  ransac.setRefineUsingInliers(false);
  ransac.setMaximumIterations(100);
    
  LabeledCloud::Ptr cloudCur(new LabeledCloud(*cloud));
  NormalCloud::Ptr normalsCur(new NormalCloud(*normals));
  std::vector<std::vector<int>> allIndices;
  std::vector<Problem::Solution> allPlanes;
  while (true) {
    std::cout << "starting ransac" << std::endl;
    Problem problem(cloudCur, normalsCur, 10, origin);
    drc::RansacGeneric<Problem>::Result result = ransac.solve(problem);

    std::vector<bool> isInlier(cloudCur->size());
    std::fill(isInlier.begin(), isInlier.end(), false);
    for (int idx : result.mInliers) isInlier[idx] = true;

    // copy new points
    LabeledCloud::Ptr cloudTemp(new LabeledCloud());
    NormalCloud::Ptr normalsTemp(new NormalCloud());
    std::vector<int> indices;
    for (int i = 0; i < (int)isInlier.size(); ++i) {
      if (!isInlier[i]) {
        cloudTemp->push_back(cloudCur->points[i]);
        normalsTemp->push_back(normalsCur->points[i]);
      }
      else {
        indices.push_back(cloudCur->points[i].label);
      }
    }
    cloudCur = cloudTemp;
    normalsCur = normalsTemp;
    allIndices.push_back(indices);
    allPlanes.push_back(result.mSolution);

    std::cout << "inliers " << result.mInliers.size() << std::endl;
    std::cout << "NUMS " << isInlier.size() << " " << cloudCur->size() << " " << indices.size() << std::endl;
    std::cout << "SOL " << result.mSolution.transpose() << std::endl;
    if (result.mInliers.size() < 200) break;
    if (allPlanes.size() >= 3) break;
  }

  for (auto& pt : cloud->points) pt.label = 0;
  for (int i = 0; i < (int)allIndices.size(); ++i) {
    for (int j = 0; j < (int)allIndices[i].size(); ++j) {
      cloud->points[allIndices[i][j]].label = i+1;
    }
  }

  /*
  ofs.open("/home/antone/temp/remaining_points.txt");
  for (int i = 0; i < (int)cloudCur->size(); ++i) {
    const auto& pt = cloudCur->points[i];
    const auto& normal = normalsCur->points[i];
    ofs << pt.x << " " << pt.y << " " << pt.z << " " <<
      normal.normal_x << " " << normal.normal_y << " " << normal.normal_z <<
      " " << pt.label << std::endl;
  }
  ofs.close();
  */

  ofs.open("/home/antone/temp/labeled_points.txt");
  for (const auto& pt : cloud->points) {
    ofs << pt.x << " " << pt.y << " " << pt.z << " " << pt.label << std::endl;
  }
  ofs.close();

  return 1;
}
