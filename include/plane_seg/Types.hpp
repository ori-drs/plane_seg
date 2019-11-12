#ifndef _planeseg_Types_hpp_
#define _planeseg_Types_hpp_

//#define PCL_NO_PRECOMPILE
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace planeseg {

  /*
struct Point {
  PCL_ADD_POINT4D;
  float delta;
  int label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, delta, delta)
                                  (int, label, label)
)
  */
typedef pcl::PointXYZL Point;

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<Point> LabeledCloud;
typedef Eigen::Matrix<float,Eigen::Dynamic,3> MatrixX3f;

}

#endif

