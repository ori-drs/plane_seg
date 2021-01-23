#pragma once
#include <cv_bridge/cv_bridge.h>

namespace planeseg {

class ImageProcessor {
public:
    ImageProcessor();
    ~ImageProcessor();

    void process();
    void displayImage(cv_bridge::CvImage image, std::string process);
    void saveImage(cv_bridge::CvImage image);
    void erodeImage(cv_bridge::CvImage originalImage);
    cv_bridge::CvImage original_img_;
    cv_bridge::CvImage erode_img_;

private:

};

} //namespace planeseg

