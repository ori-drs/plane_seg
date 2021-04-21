#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "plane_seg/BlockFitter.hpp"
#include "plane_seg/StepCreator.hpp"

namespace planeseg {

class Tracker2D {
public:
    Tracker2D();
    ~Tracker2D();
    
    int get_contour_id(contour contour);
    pcl::PointXYZ find_centroid(pcl::PointCloud<pcl::PointXYZ> cloud );
    void reset();
    void assignIDs(std::vector<contour> contours);
    void printIds();
    std::vector<contour> newRects_;
    
private:

    std::vector<contour> oldRects_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs;
    int totalIds;
};

} // namespace planeseg
