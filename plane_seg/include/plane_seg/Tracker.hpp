#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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

class Tracker3D {
public:
    Tracker3D();
    ~Tracker3D();
    
    int get_plane_id(planeseg::plane plane_);
    pcl::PointXYZ find_centroid(pcl::PointCloud<pcl::PointXYZ> cloud );
    void reset();
    void convertResult(planeseg::BlockFitter::Result result);
    void test(planeseg::BlockFitter::Result result);
    void printIds();
    std::vector<plane> newStairs;
    
private:

    std::vector<plane> oldStairs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs;
    int totalIds;
};

} // namespace planeseg
