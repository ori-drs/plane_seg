#pragma once
#include <cv_bridge/cv_bridge.h>

namespace planeseg {

struct contour{
    std::vector<cv::Point> points_;
    double elevation_;
};


class StepCreator{

public:
    StepCreator();
    ~StepCreator();

    void go();
    void setImages(cv_bridge::CvImage img1, cv_bridge::CvImage img2);
    void extractContours();
    void displayImage(std::string process, cv::Mat img, int n);
    void maskElevation(cv::Mat mask_);
    void reset();
    double medianMat(cv::Mat Input);
    cv::Mat createMask(contour cntr);
    cv_bridge::CvImage processed_;
    cv_bridge::CvImage elevation_;
    cv_bridge::CvImage elevation_masked_;
    cv_bridge::CvImage reconstructed_;
    std::vector<contour> rectangles_;

private:

};

} // namespace planeseg
