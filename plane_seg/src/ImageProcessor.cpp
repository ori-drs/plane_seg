#include "plane_seg/ImageProcessor.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/highgui.hpp>

namespace planeseg {

ImageProcessor::ImageProcessor(){
}

ImageProcessor::~ImageProcessor(){}

void ImageProcessor::convertToImg(const grid_map_msgs::GridMap &msg){
    grid_map::GridMap gridmap;
    grid_map::GridMapRosConverter::fromMessage(msg, gridmap);
    std::string layer;
    grid_map::GridMapRosConverter::toCvImage(gridmap, "elevation", sensor_msgs::image_encodings::MONO8, original_img_);
}

void ImageProcessor::displayImage(cv_bridge::CvImage image, std::string process){
    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    cv::imshow(process, image.image);
}

void ImageProcessor::saveImage(cv_bridge::CvImage image_){
    std::string imagename;
    std::cout << "Enter filename to save image (don't forget .png!): " << std::endl;
    std::cin >> imagename;
    cv::imwrite("/home/christos/rosbags/" + imagename, image_.image);
}

void ImageProcessor::erodeImage(cv_bridge::CvImage originalImage){

    int erode_size = 10;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2*erode_size + 1, 2*erode_size + 1),
                                                cv::Point(erode_size, erode_size)
                                                );
    cv::erode(originalImage.image, erode_img_.image, element);
}

} //namespace planeseg
