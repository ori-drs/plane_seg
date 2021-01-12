#include "plane_seg/Tracker.hpp"
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "plane_seg/BlockFitter.hpp"
#include "plane_seg/Types.hpp"
#include "pcl/common/impl/centroid.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>

using namespace planeseg;

Tracker::Tracker(){
    totalIds = 1;
}

Tracker::~Tracker(){}

pcl::PointXYZ Tracker::find_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
    std::cout << "Computing centroid" << std::endl;
    Eigen::Vector4f centroid_eigen;
    pcl::compute3DCentroid(*cloud, centroid_eigen);
    pcl::PointXYZ centroid;
    // NOTE: assuming the first three values of the centroid are the Eucledian
    // coordinates of the centroid. The fourth value is discarded.
    centroid.x = centroid_eigen(0);
    centroid.y = centroid_eigen(1);
    centroid.z = centroid_eigen(2);
    return centroid;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Tracker::convertResult(planeseg::BlockFitter::Result result_){
    for (size_t i=0; i<result_.mBlocks.size(); ++i){
      pcl::PointCloud<pcl::PointXYZ> cloud;
      const auto& block = result_.mBlocks[i];
      for (size_t j =0; j < block.mHull.size(); ++j){
        pcl::PointXYZ pt;
        pt.x =block.mHull[j](0);
        pt.y =block.mHull[j](1);
        pt.z =block.mHull[j](2);
        cloud.points.push_back(pt);
      }
      cloud.height = cloud.points.size();
      cloud.width = 1;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
      cloud_ptr = cloud.makeShared();
      cloud_ptrs.push_back(cloud_ptr);
    }
    return cloud_ptrs;
}

// First test is to just calculate and print centroids
void Tracker::test(planeseg::BlockFitter::Result result_){
    reset();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs_;
    cloud_ptrs = convertResult(result_);
    for (size_t i =0 ; i < cloud_ptrs_.size(); i++){
        newStairs[i].centroid = find_centroid(cloud_ptrs_[i]);
    }
    printStairs(newStairs);
}


int Tracker::get_centroid_id(planeseg::plane plane){

    bool test = true;
    int id;

    if(oldStairs.empty()){
        id = totalIds;
        ++totalIds;
    }

    else{
        pcl::PointXYZ centroid;
        double threshold = 1;
        double distance;
        double distz;
        int closest = -1;
        double closestDist = 0;

        std::cout << "Entered get_centroid_id" << std::endl;

        // go through each plane in oldStairs and compare it to the new plane

        for (size_t i = 0; i < oldStairs.size(); i++){

            centroid = oldStairs[i].centroid;

            if (!idAssigned[i].taken && test){
                distz = fabs(centroid.z = plane.centroid.z);
                if (distz < threshold) {

                    distance = pcl::euclideanDistance(oldStairs[i].centroid, plane.centroid);

                    if (closest == -1 || distance < closestDist){
                        closest = i;
                        closestDist = distance;
                    }
                }
            }
        }

        if (closest != -1){
            id = oldStairs[closest].id;
            idAssigned[closest].taken = true;
        }
        else{
            id = totalIds;
            ++totalIds;
        }
    }
    plane.id = id;
    newStairs.push_back(plane);
    return id;
}

void Tracker::reset(){
    oldStairs = newStairs;
    newStairs.clear();
    idAssigned.clear();
    IdAssigned temp;
    temp.taken = false;
    for (int i=0; i < oldStairs.size(); i++){
        temp.id = oldStairs[i].id;
        idAssigned.push_back(temp);
    }
}


void Tracker::printStairs(const std::vector<plane>& stairs){
  std::cout << "Centroids of " << stairs.size() << " steps: " << std::endl;
    for (unsigned i=0; i < stairs.size(); i++){
      std::cout << "Step " << i << " (x,y,z): ";
      std::cout << stairs[i].centroid.x << ", ";
      std::cout << stairs[i].centroid.y << ", ";
      std::cout << stairs[i].centroid.z << std::endl;
    }
}

