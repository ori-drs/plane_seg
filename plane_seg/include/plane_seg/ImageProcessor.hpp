#ifndef _planeseg_ImageProcessor_hpp_
#define _planeseg_ImageProcessor_hpp_

#include <grid_map_msgs/GridMap.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

namespace planeseg {

class ImageProcessor {
public:
    ImageProcessor();
    ~ImageProcessor();

    void convertToImg(const grid_map_msgs::GridMap &msg);
    void displayImage(cv_bridge::CvImage image, std::string process);
    void saveImage(cv_bridge::CvImage image);
    void erodeImage(cv_bridge::CvImage originalImage);
    cv_bridge::CvImage original_img_;
    cv_bridge::CvImage erode_img_;

private:

};

} //namespace planeseg

#endif
