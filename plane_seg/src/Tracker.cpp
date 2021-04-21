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

namespace planeseg {

Tracker3D::Tracker3D(){
    totalIds = 0;
}

Tracker3D::~Tracker3D(){}

pcl::PointXYZ Tracker3D::find_centroid(pcl::PointCloud<pcl::PointXYZ> cloud ){
//    std::cout << "Computing centroid" << std::endl;
    Eigen::Vector4f centroid_eigen;
    pcl::compute3DCentroid(cloud, centroid_eigen);
    pcl::PointXYZ centroid;
    // NOTE: assuming the first three values of the centroid are the Eucledian
    // coordinates of the centroid. The fourth value is discarded.
    centroid.x = centroid_eigen(0);
    centroid.y = centroid_eigen(1);
    centroid.z = centroid_eigen(2);
    return centroid;
}

// converts the result from the BlockFitter to a vector of planes complete with pointclouds, centroids and ids, and saves it as newStairs
void Tracker3D::convertResult(planeseg::BlockFitter::Result result_){
    std::cout << "entered convertResult" << std::endl;
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

      pcl::PointXYZ cloud_centroid;
      cloud_centroid = find_centroid(cloud);

      planeseg::plane newPlane;
      newPlane.cloud = cloud;
      newPlane.centroid = cloud_centroid;
      newPlane.id = get_plane_id(newPlane);

      newStairs.push_back(newPlane);
    }

}

int Tracker3D::get_plane_id(planeseg::plane plane){

//    std::cout << "entered get_plane_id" << std::endl;
    int id;

    if(oldStairs.empty()){
        id = totalIds;
        ++totalIds;
    }

    else{
        pcl::PointXYZ oldCentroid;
        double threshold = 0.1;
        double distz;
        int closest = -1;
        // start with closestDist being far larger than anything that would ever happen
        double closestDist = 1000000000;

        // go through each plane in oldStairs and compare it to the new plane

        for (size_t i = 0; i < oldStairs.size(); ++i){

            distz = fabs(oldStairs[i].centroid.z - plane.centroid.z);

            // is distz small enough to suggest that this point cloud represents the next iteration of an existing frame?
              if(distz < threshold){
  //              distance = pcl::euclideanDistance(oldStairs[i].centroid, plane.centroid);

                    if (distz < closestDist){
                        closest = i;
                        closestDist = distz;
                    }
                }
        }

        if (closest != -1){
            id = oldStairs[closest].id;
        }
        else{
            id = totalIds;
            ++totalIds;
        }
    }

    return id;
}

void Tracker3D::reset(){
    std::cout << "entered Tracker reset" << std::endl;
    oldStairs = newStairs;
    newStairs.clear();
    }

void Tracker3D::printIds(){
//    std::cout << "entered printIds" << std::endl;
    std::cout << "Total number of ids assigned: " << newStairs.size() << std::endl;
    for (size_t i = 0; i < newStairs.size(); ++i){
        std::cout << newStairs[i].id << std::endl;
    }
}

} // namespace plane_seg
