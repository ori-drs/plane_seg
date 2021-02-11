#include "plane_seg/ImageProcessor.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <boost/filesystem/operations.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace planeseg {

void contours::filterMinConvexity(int min_convexity){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_.clear();

    for (size_t p = 0; p < temp.size(); ++p){
        std::vector<cv::Point> hull;
        double hull_perimeter;
        double contour_perimeter;
        double convexity;

        cv::convexHull(temp[p], hull);
        hull_perimeter = cv::arcLength(hull, true);
        contour_perimeter = cv::arcLength(temp[p], true);
        convexity = hull_perimeter / contour_perimeter;

        if (convexity >= min_convexity){
            contours_.push_back(temp[p]);
        }
    }
}

void contours::filterMinElongation(int min_elongation){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_.clear();

    for (size_t r = 0; r < temp.size(); ++r){
        double elongation;
        cv::RotatedRect minarea_rect;
        minarea_rect = cv::minAreaRect(temp[r]);
        double major_axis = std::max(minarea_rect.size.height, minarea_rect.size.width);
        double minor_axis = std::min(minarea_rect.size.height, minarea_rect.size.width);
        elongation = major_axis / minor_axis;

        if (elongation >= min_elongation){
            contours_.push_back(temp[r]);
        }
    }
}

void contours::approxAsPoly(){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_.clear();

    for(size_t l = 0; l < temp.size(); ++l){
        double epsilon;
        epsilon = 3;
        std::vector<cv::Point> approx;
        cv::approxPolyDP(temp[l], approx, epsilon, true);
        contours_.push_back(approx);
    }
}

void contours::fitMinAreaRect(){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_rect_.clear();

    for (size_t r = 0; r < temp.size(); ++r){
        cv::RotatedRect minarea_rect;
        cv::Point2f point2f_box[4];
        minarea_rect = cv::minAreaRect(temp[r]);
        minarea_rect.points(point2f_box);
        std::vector<cv::Point> point_box;
        for (int i = 0; i < 4; ++i){
            cv::Point pnt;
            pnt.x = point2f_box[i].x;
            pnt.y = point2f_box[i].y;
            point_box.push_back(pnt);
        }

        contours_rect_.push_back(point_box);

    }
}
/*
void contours::setColors(){
    colors_ = {
        1, 1, 1, // 42
        255, 255, 120,
        1, 120, 1,
        1, 225, 1,
        120, 255, 1,
        1, 255, 255,
        120, 1, 1,
        255, 120, 255,
        120, 1, 255,
        1, 1, 120,
        255, 255, 255,
        120, 120, 1,
        120, 120, 120,
        1, 1, 255,
        255, 1, 255,
        120, 120, 255,
        120, 255, 120,
        1, 120, 120,
        1, 1, 255,
        255, 1, 1,
        155, 1, 120,
        120, 1, 120,
        255, 120, 1,
        1, 120, 255,
        255, 120, 120,
        1, 255, 120,
        255, 255, 1};
}

void contours::assignIDs(){
    for (size_t i = 0; i < contours_.size(); ++i){
        ids[i] = i;
    }
}

void contours::assignColors(){
    for (size_t i = 0; i < contours_.size(); ++i){
        contour_colours_[i].r = getR(ids[i]);
        contour_colours_[i].g = getG(ids[i]);
        contour_colours_[i].b = getB(ids[i]);
    }
}

double contours::getR(int id){
    double j;
    j = id % (colors_.size()/3);
    return colors_[3*j];
}

double contours::getG(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j+1];
}

double contours::getB(int id){
    unsigned j;
    j = id % (colors_.size()/3);
    return colors_[3*j+2];
}
*/
ImageProcessor::ImageProcessor(){
}

ImageProcessor::~ImageProcessor(){}

void ImageProcessor::process(){

//    copyOrigToProc();
    displayImage("original", original_img_.image, 0);
//    double min, max;
//    cv::minMaxLoc(original_img_.image, &min, &max);
//    std::cout << "min = " << min << ", max = " << max << std::endl;
    histogram(original_img_);
    thresholdImage(1);
    convertImgType(processed_img_.image, CV_8U);
    std::cout << "processed_img_ has type " << processed_img_.image.type() << std::endl;
    displayImage("threshold", processed_img_.image, 1);
    erodeImage(1);
    dilateImage(2);
    erodeImage(1);
//    erodeImage(2);
//    dilateImage(2);
    displayImage("erode/dilate", processed_img_.image, 2);

    extractContours();
    splitContours();
    drawContoursIP(med_contours_, "med_contours", 3);
    drawContoursIP(large_contours_, "large contours", 4);
    med_contours_.filterMinConvexity(0.9); // have another look at the threshold for convexity
    drawContoursIP(med_contours_, "filtered by convexity", 5);
    med_contours_.filterMinElongation(2.5);
    drawContoursIP(med_contours_, "filtered by elongation", 6);
    med_contours_.fitMinAreaRect();
    mergeContours();
//    all_contours_.assignIDs();
//    all_contours_.assignColors();

    displayResult();
//    final_img_ = processed_img_;

    int p = cv::waitKey(0);
    if (p == 's'){
        saveImage(final_img_);
    }
}

void ImageProcessor::copyOrigToProc(){
    processed_img_ = original_img_;
}

void ImageProcessor::convertImgType(cv::Mat img, int type){
    img.convertTo(img, type);
}

void ImageProcessor::displayImage(std::string process, cv::Mat img, int n) {

    std::cout << "Entered displayImage()" << std::endl;
    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    std::cout << 3 << std::endl;
    cv::moveWindow(process, 100 + img.cols * n, 50);
    std::cout << 4 << std::endl;
    cv::imshow(process, img);
    std::cout << 5 << std::endl;
}

void ImageProcessor::displayResult(){

    colour_img_.image = cv::Mat::zeros(original_img_.image.size(), CV_8UC3);
    processed_img_.image = cv::Mat::zeros(original_img_.image.size(), CV_8U);

    for(size_t i = 0; i < all_contours_.contours_.size(); ++i){
        cv::RNG rng;
        cv::Scalar color(rand()&255, rand()&255, rand()&255);
        cv::drawContours(colour_img_.image, all_contours_.contours_, i, color, cv::FILLED);
        cv::drawContours(processed_img_.image, all_contours_.contours_, i, cv::Scalar(255), cv::FILLED);
    }
    for(size_t j = 0; j < all_contours_.contours_rect_.size(); ++j){
        cv::drawContours(colour_img_.image, all_contours_.contours_rect_, j, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    final_img_ = processed_img_;
    displayImage("final", colour_img_.image, 7);
}

void ImageProcessor::saveImage(cv_bridge::CvImage image_){
    std::string imagename;
    std::cout << "Enter filename to save image (don't forget .png!): " << std::endl;
    std::cin >> imagename;
    std::string homedir = getenv("HOME");
    if(!homedir.empty()){
      boost::filesystem::path output_path(homedir + "/rosbags/image_processing/");
      if(!boost::filesystem::is_directory(output_path)){
        if(boost::filesystem::create_directory(output_path)){
          std::cout << "Creating directory " << output_path.string() << " ... " << std::endl;
        } else {
          std::cerr << "ERROR: directory " << output_path.string() << " does not exists and couln't be created." << std::endl;
          return;
        }
      }
      cv::imwrite(output_path.string() + imagename, image_.image);
    } else {
      std::cerr << "ERROR: the $HOME variable is empty!" << std::endl;
      return;
    }
}

void ImageProcessor::erodeImage(int erode_size){

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2*erode_size + 1, 2*erode_size + 1),
                                                cv::Point(erode_size, erode_size)
                                                );

    cv::erode(processed_img_.image, processed_img_.image, element);
}

void ImageProcessor::dilateImage(int dilate_size){

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2*dilate_size+1, 2*dilate_size +1),
                                                cv::Point(dilate_size, dilate_size)
                                                );
    cv::dilate(processed_img_.image, processed_img_.image, element);
}

void ImageProcessor::thresholdImage(int threshold_value){

    int threshold_type = cv::ThresholdTypes::THRESH_BINARY_INV;
    double maxval = 255;

    cv::threshold(original_img_.image, processed_img_.image, threshold_value, maxval, threshold_type);

}

void ImageProcessor::extractContours(){
    cv::findContours(processed_img_.image, all_contours_.contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

void ImageProcessor::splitContours(){

    double area;
    for(size_t k = 0; k < all_contours_.contours_.size(); ++k){
        area = fabs(cv::contourArea(all_contours_.contours_[k]));

        if (area >= 1000){
            large_contours_.contours_.push_back(all_contours_.contours_[k]);
        } else if (area >=200 && area <= 1000){
            med_contours_.contours_.push_back(all_contours_.contours_[k]);
        }
    }
}

void ImageProcessor::mergeContours(){
    all_contours_.contours_.clear();
    all_contours_.contours_rect_.clear();

    for (size_t v = 0; v < large_contours_.contours_.size(); ++v){
    all_contours_.contours_.push_back(large_contours_.contours_[v]);
    }
    for (size_t w = 0; w < med_contours_.contours_.size(); ++w){
    all_contours_.contours_.push_back(med_contours_.contours_[w]);
    }

    all_contours_.contours_rect_ = med_contours_.contours_rect_;
}

void ImageProcessor::drawContoursIP(contours contour, std::string process, int n){
    cv_bridge::CvImage temp;
    temp.image = cv::Mat::zeros(original_img_.image.size(), CV_8U);

    for(size_t i = 0; i < contour.contours_.size(); ++i){
        cv::drawContours(temp.image, contour.contours_, i, cv::Scalar(255), cv::FILLED);
    }

//    processed_img_ = temp;
    displayImage(process, temp.image, n);
//    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
//    cv::imshow(process, temp.image);
}

void ImageProcessor::histogram(cv_bridge::CvImage img){
    int bins = 30;
    int histSize = bins;
    float range[] = {0, 1};
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    cv::Mat hist, mask;
    mask = createMask(img);
    std::cout << "Exited createMask()" << std::endl;
//    displayImage("mask", mask, 1);
    std::cout << "Exited displayImage()" << std::endl;
//    convertImgType(mask, CV_8UC1);
    cv::calcHist(&img.image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/bins );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0) );
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    for (int i = 1; i < histSize; i++){
        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1))), cv::Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    cv::imshow("original_img_ histogram", histImage);
    cv::waitKey(0);
}

void ImageProcessor::reset(){
    original_img_.image = cv::Mat();
    processed_img_.image = cv::Mat();
    final_img_.image = cv::Mat();
    all_contours_.contours_.clear();
    large_contours_.contours_.clear();
    med_contours_.contours_.clear();
}


cv::Mat ImageProcessor::createMask(cv_bridge::CvImage img){
//    std::cout<< img.image << std::endl;
    std::cout << "Entered createMask()" << std::endl;
    cv::Mat mask_;
    mask_ = cv::Mat::zeros(img.image.size(), CV_8UC1);

//    std::cout << "Image size: " << img.image.size() << std::endl;
//    std::cout << "Mask size: " << mask.size() << std::endl;
    for(int r = 0; r < img.image.rows; r++)
    {
        for(int c = 0; c < img.image.cols; c++)
        {
            if (img.image.at<float>(r,c) > 0.64085822 && img.image.at<float>(r,c) < 0.64085824)
            {
                mask_.at<int>(r,c) = 255;
            }
        }
    }

    std::cout << "Image type: " << img.image.type() << std::endl;
    std::cout << "Mask size: " << mask_.type() << std::endl;

//    displayImage("mask", mask, 8);
//    cv::waitKey(0);

    return mask_;
}

} //namespace planeseg
