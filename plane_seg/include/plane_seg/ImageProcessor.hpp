#pragma once
#include <cv_bridge/cv_bridge.h>

namespace planeseg {
/*
struct colour{
    double r;
    double g;
    double b;
};
*/
struct contours{
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<std::vector<cv::Point>> contours_rect_;
    void filterSmallContours();
    void filterMinConvexity(int min_convexity);
    void filterMinElongation(int min_elongation);
    void filterMinRectangularity(int min_rectangularity);
    void fitMinAreaRect();
    void approxAsPoly();
    void fitSquare();
    bool isSquare(std::vector<cv::Point> contour_);
    void print(std::vector<double> property);
    std::vector<double> elongations_;
    std::vector<double> convexities_;
    std::vector<double> rectangularities_;

/*
    void setColors();
    void assignColors();
    void assignIDs();
    double getR(int id);
    double getG(int id);
    double getB(int id);
    std::vector<colour> contour_colours_;
    std::vector<int> ids;
    std::vector<double> colors_;
*/
};

class ImageProcessor {
public:
    ImageProcessor();
    ~ImageProcessor();

    void process();
    void copyOrigToProc();
    void displayImage(std::string process, cv::Mat img, int n = 0);
    void saveImage(cv::Mat image);
    void erodeImage(int erode_size);
    void thresholdImage(float threshold_value);
    void dilateImage(int dilate_size);
    void blobDetector(cv_bridge::CvImage image);
    void removeBlobs(cv_bridge::CvImage image);
    void extractContours();
    void fillContours();
    void splitContours();
    void mergeContours();
    void drawContoursIP(contours contour, std::string process, int n = 0);
    void displayResult();
    void reset();
    void histogram(cv_bridge::CvImage img);
    cv::Mat createMask(cv_bridge::CvImage img);
    cv_bridge::CvImage original_img_;
    cv_bridge::CvImage processed_img_;
    cv_bridge::CvImage rect_img_;
    cv_bridge::CvImage final_img_;
    cv_bridge::CvImage colour_img_;

    contours all_contours_;
    contours large_contours_;
    contours med_contours_;

    std::vector<double> ip_elongations_;
    std::vector<double> ip_convexities_;
    std::vector<double> ip_rectangularities_;

private:

};

} //namespace planeseg

