#include "plane_seg/Tracker2D.hpp"
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "plane_seg/BlockFitter.hpp"
#include "plane_seg/Types.hpp"
#include "pcl/common/impl/centroid.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>

namespace planeseg {

Tracker2D::Tracker2D(){
    totalIds = 0;
}

Tracker2D::~Tracker2D(){}


// converts the result from the BlockFitter to a vector of planes complete with pointclouds, centroids and ids, and saves it as newStairs
void Tracker2D::assignIDs(std::vector<contour> contours){
    std::cout << "entered assignIDs" << std::endl;
    for (size_t i = 0; i < contours.size(); ++i){
        planeseg::contour newContour;
        newContour.points_ = contours[i].points_;
        newContour.elevation_ = contours[i].elevation_;
        newContour.id_ = get_contour_id(newContour);

        newRects_.push_back(newContour);
    }
}

int Tracker2D::get_contour_id(planeseg::contour contour){

    std::cout << "entered get_plane_id" << std::endl;
    int id;

    if(oldRects_.empty()){
        std::cout << "oldRects_ is empty" << std::endl;
        id = totalIds;
        ++totalIds;
    }

    else{
        std::cout << "oldRects_ has size " << oldRects_.size() << std::endl;
        double threshold = 0.1;
        double dist;
        int closest = -1;
        // start with closestDist being far larger than anything that would ever happen
        double closestDist = 1000000000;

        // go through each plane in oldStairs and compare it to the new plane

        for (size_t i = 0; i < oldRects_.size(); ++i){

            dist = fabs(oldRects_[i].elevation_ - contour.elevation_);
            std::cout << "oldRects_[" << i << "].elevation_ = " << oldRects_[i].elevation_;
            std::cout << "contour.elevation_ = " << contour.elevation_ << std::endl;
            std::cout << "dist = " << dist << std::endl;
            // is distz small enough to suggest that this point cloud represents the next iteration of an existing frame?
              if(dist < threshold){
  //              distance = pcl::euclideanDistance(oldStairs[i].centroid, plane.centroid);

                    if (dist < closestDist){
                        closest = i;
                        closestDist = dist;
                    }
                }
        }

        if (closest != -1){
            std::cout << "Here" << std::endl;
            id = oldRects_[closest].id_;
        }
        else{
            std::cout << "Here2" << std::endl;
            id = totalIds;
            ++totalIds;
        }
    }
    std::cout << "id = " << id << std::endl;
    return id;
}

void Tracker2D::reset(){
    std::cout << "entered Tracker2D reset" << std::endl;
    oldRects_ = newRects_;
    newRects_.clear();
    }

void Tracker2D::printIds(){
//    std::cout << "entered printIds" << std::endl;
    std::cout << "Total number of ids assigned: " << newRects_.size() << std::endl;
    for (size_t i = 0; i < newRects_.size(); ++i){
        std::cout << newRects_[i].id_ << std::endl;
    }
}

} // namespace plane_seg
