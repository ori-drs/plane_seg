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
    void erodeImage(cv_bridge::CvImage originalImage, int erode_size);
    void thresholdImage(cv_bridge::CvImage image);
    void dilateImage(cv_bridge::CvImage originalImage, int dilate_size);
    void blobDetector(cv_bridge::CvImage image);
    void removeBlobs(cv_bridge::CvImage image);
    cv_bridge::CvImage original_img_;
    cv_bridge::CvImage bit32_img_;
    cv_bridge::CvImage erode_img_;
    cv_bridge::CvImage threshold_img_;
    cv_bridge::CvImage dilate_img_;
    cv_bridge::CvImage adap_threshold_img_;
    cv_bridge::CvImage detect_img_;
    cv_bridge::CvImage large_contours_;
    cv_bridge::CvImage small_contours_;
    std::vector<std::vector<cv::Point>> contours1;

private:

};

} //namespace planeseg

