#pragma once
#include <cv_bridge/cv_bridge.h>

namespace planeseg {

struct contours{
    std::vector<std::vector<cv::Point>> contours_;
    void filterSmallContours();
    void filterMinConvexity(int min_convexity);
    void filterMinElongation(int min_elongation);
    void fitMinAreaRect();
    void approxAsPoly();
};


class ImageProcessor {
public:
    ImageProcessor();
    ~ImageProcessor();

    void process();
    void copyOrigToProc();
    void displayImage(std::string process);
    void saveImage(cv_bridge::CvImage image);
    void erodeImage(int erode_size);
    void thresholdImage(int threshold_value);
    void dilateImage(int dilate_size);
    void blobDetector(cv_bridge::CvImage image);
    void removeBlobs(cv_bridge::CvImage image);
    void extractContours();
    void splitContours();
    void mergeContours();
    void drawContoursIP(contours contour, std::string process);
    void displayResult();
    void reset();
    void fourierTransform(cv_bridge::CvImage image);
    cv_bridge::CvImage original_img_;
    cv_bridge::CvImage processed_img_;
    cv_bridge::CvImage final_img_;

    cv_bridge::CvImage fourier_img1_;
    cv_bridge::CvImage fourier_img2_;
    cv_bridge::CvImage magI;
    contours all_contours_;
    contours large_contours_;
    contours med_contours_;

private:

};

} //namespace planeseg

