#ifndef _planeseg_Tracker_hpp_
#define _planeseg_Tracker_hpp_

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/impl/centroid.hpp>
#include "plane_seg/BlockFitter.hpp"

namespace planeseg {

struct plane {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int id;
    pcl::PointXYZ centroid;
};

// a vector to hold the ids of oldStairs and a flag for whether the id has been assigned
struct IdAssigned{
    bool taken;
    int id;
};

class Tracker {
public:
    Tracker();
    ~Tracker();
    
    int get_centroid_id(plane plane_);
    pcl::PointXYZ find_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
    void reset();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> convertResult(planeseg::BlockFitter::Result result);
    void test(planeseg::BlockFitter::Result result);
    void printStairs(std::vector<Eigen::Vector4f> centroid_list_);
    
private:
    std::vector<plane> newStairs;
    std::vector<plane> oldStairs;
    std::vector<IdAssigned> idAssigned;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs;
    std::vector<Eigen::Vector4f> icentroid_list_init;
    int totalIds;
};
}

#endif
