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

Tracker::Tracker(){
    totalIds = 1;
}

Tracker::~Tracker(){}

pcl::PointXYZ Tracker::find_centroid(pcl::PointCloud<pcl::PointXYZ> cloud ){
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
void Tracker::convertResult(planeseg::BlockFitter::Result result_){
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

 //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
 //     cloud_ptr = cloud.makeShared();
 //     cloud_ptrs.push_back(cloud_ptr);
      planeseg::plane newPlane;
      newPlane.cloud = cloud;
      newPlane.centroid = cloud_centroid;
      newPlane.id = get_plane_id(newPlane);

//      std::vector<planeseg::planes> vec_planes_no_ids.push_back(plane_no_id);
//      vector_of_planes.push_back(plane_no_id);
      newStairs.push_back(newPlane);
    }

}

// *** NEXT STEP: create std::vector<int> = planesToIds(std::vector<planeseg::planes>) which goes through each plane in the vector and calls get_plane_id and assigns it to the plane

std::vector<int> Tracker::planesToIds(){
    std::cout << "entered planesToIds" << std::endl;
    for (size_t i = 0; i < vector_of_planes.size(); ++i){
        int current_id;
        current_id = Tracker::get_plane_id(vector_of_planes[i]);
        vector_of_ids.push_back(current_id);
    }
    return vector_of_ids;
}

int Tracker::get_plane_id(planeseg::plane plane){

//    std::cout << "entered get_plane_id" << std::endl;
    bool test = true;
    int id;

    if(oldStairs.empty()){
        id = totalIds;
        ++totalIds;
    }

    else{
        pcl::PointXYZ oldCentroid;
        double threshold = 0.1;
        double distance;
        double distz;
        int closest = -1;
        // start with closestDist being far larger than anything that would ever happen
        double closestDist = 1000000000;

        // go through each plane in oldStairs and compare it to the new plane

        for (size_t i = 0; i < oldStairs.size(); ++i){

            // if the id of the ith plane in oldStair exists
     //       if (!idAssigned[i].taken && test){
            distz = fabs(oldStairs[i].centroid.z - plane.centroid.z);
  //          std::cout << "distz = " << distz << std::endl;
              if(distz < threshold){
  //              distance = pcl::euclideanDistance(oldStairs[i].centroid, plane.centroid);

                    if (distz < closestDist){
                        closest = i;
                        closestDist = distz;
                    }
                }
           // }
        }

   //     std::cout << "closestDist = " << closestDist << std::endl;

        if (closest != -1){
            id = oldStairs[closest].id;
//            idAssigned[closest].taken = true;
        }
        else{
            id = totalIds;
            ++totalIds;
        }
    }
//    plane.id = id;
//    newStairs.push_back(plane);
    return id;
}

void Tracker::reset(){
    std::cout << "entered Tracker reset" << std::endl;
    oldStairs = newStairs;
    newStairs.clear();
    idAssigned.clear();

    // set the ids in the vector IdAssigned to be the same as the ids in the planes of oldStairs
    IdAssigned temp;
    temp.taken = false;
    for (size_t i=0; i < oldStairs.size(); ++i){
        temp.id = oldStairs[i].id;
        idAssigned.push_back(temp);
    }
}


void Tracker::printStairs(std::vector<plane> stairs){
  std::cout << "Centroids of " << stairs.size() << " steps: " << std::endl;
    for (unsigned i = 0; i < stairs.size(); ++i){
      std::cout << "Step " << i << " (x,y,z): ";
      std::cout << stairs[i].centroid.x << ", ";
      std::cout << stairs[i].centroid.y << ", ";
      std::cout << stairs[i].centroid.z << std::endl;
    }
}

void Tracker::printIds(){
//    std::cout << "entered printIds" << std::endl;
    std::cout << "Total number of ids assigned: " << newStairs.size() << std::endl;
    for (size_t i = 0; i < newStairs.size(); ++i){
        std::cout << newStairs[i].id << std::endl;
    }
}

void Tracker::printidAssigned(){
    std::cout << "idAssigned: " << std::endl;
    for (size_t i = 0; i < idAssigned.size(); ++i){
        std::cout << idAssigned[i].id << std::endl;
    }
}

} // namespace plane_seg
