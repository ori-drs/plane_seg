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
    
    int get_plane_id(planeseg::plane plane_);
    pcl::PointXYZ find_centroid(pcl::PointCloud<pcl::PointXYZ> cloud );
    void reset();
    void convertResult(planeseg::BlockFitter::Result result);
    std::vector<int> planesToIds();
    void test(planeseg::BlockFitter::Result result);
    void printStairs(std::vector<plane> centroid_list_);
    void printIds();
    void printidAssigned();
    std::vector<plane> newStairs;
    
private:

    std::vector<plane> oldStairs;
    std::vector<IdAssigned> idAssigned;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs;
    std::vector<planeseg::plane> vector_of_planes;
    std::vector<int> vector_of_ids;
    std::vector<planeseg::plane> vec_planes_no_ids;
    std::vector<Eigen::Vector4f> icentroid_list_init;
    int totalIds;
};
}

#endif
