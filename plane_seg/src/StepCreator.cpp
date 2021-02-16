#include "plane_seg/StepCreator.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

namespace planeseg {

StepCreator::StepCreator(){
}

StepCreator::~StepCreator(){}

void StepCreator::go(){
    std::cout << "Entered go" << std::endl;
    displayImage("processed", processed_.image, 0);
    extractContours();

    for (size_t i = 0; i < rectangles_.size(); ++i){
        cv::Mat mask;
        mask = createMask(rectangles_[i]);
        rectangles_[i].elevation_ = cv::mean(elevation_.image, mask);
    }

    std::vector<std::vector<cv::Point>> v;
    std::vector<cv::Scalar> elev;
    elevation_masked_.image = cv::Mat::zeros(processed_.image.size(), CV_32F);

    for (size_t j = 0; j < rectangles_.size(); ++j){
        v.push_back(rectangles_[j].points_);
        elev.push_back(rectangles_[j].elevation_);
    }
    for (size_t k = 0; k < v.size(); ++k){
        cv::drawContours(elevation_masked_.image, v, k, elev[k], cv::FILLED);
    }
    displayImage("elevation masked", elevation_masked_.image, 4);

}

void StepCreator::setImages(cv_bridge::CvImage img1, cv_bridge::CvImage img2){
    processed_ = img1;
    elevation_ = img2;
}

void StepCreator::extractContours(){
    std::vector<std::vector<cv::Point>> pnts;
    cv::findContours(processed_.image, pnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t j = 0; j < pnts.size(); ++j){
        contour temp;
        temp.points_ = pnts[j];
        rectangles_.push_back(temp);
    }
}

cv::Mat StepCreator::createMask(contour cntr){
    cv_bridge::CvImage contour_img_;
    std::vector<std::vector<cv::Point>> c;
    c.push_back(cntr.points_);
    contour_img_.image = cv::Mat::zeros(processed_.image.size(), CV_8U);
    for(size_t k = 0; k < c.size(); ++k){
        cv::drawContours(contour_img_.image, c, k, cv::Scalar(255), cv::FILLED);
    }
    return contour_img_.image;
}

void StepCreator::displayImage(std::string process, cv::Mat img, int n) {
    std::cout << "Entered displayImage" << std::endl;

    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(process, 100 + img.cols * n, 50);
    cv::imshow(process, img);
    cv::waitKey(0);
}

void StepCreator::maskElevation(cv::Mat mask_){
    elevation_.image.copyTo(elevation_masked_.image, mask_);
}

void StepCreator::reset(){
    processed_.image = cv::Mat();
    elevation_.image = cv::Mat();
    elevation_masked_.image = cv::Mat();
    rectangles_.clear();
}

} // namespace planeseg
