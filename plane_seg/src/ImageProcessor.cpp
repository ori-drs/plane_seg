#include "plane_seg/ImageProcessor.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <boost/filesystem/operations.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

namespace planeseg {

void contours::filterMinConvexity(int min_convexity){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_.clear();
    convexities_.clear();

    for (size_t p = 0; p < temp.size(); ++p){
        std::vector<cv::Point> hull;
        double hull_perimeter;
        double contour_perimeter;
        double convexity;

        cv::convexHull(temp[p], hull);
        hull_perimeter = cv::arcLength(hull, true);
        contour_perimeter = cv::arcLength(temp[p], true);
        convexity = hull_perimeter / contour_perimeter;
        convexities_.push_back(convexity);

        if (convexity >= min_convexity){
            contours_.push_back(temp[p]);
        }
    }

//    print(convexities_);
}

void contours::filterMinElongation(int min_elongation){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_.clear();
    elongations_.clear();

    for (size_t r = 0; r < temp.size(); ++r){
        double elongation;
        cv::RotatedRect minarea_rect;
        minarea_rect = cv::minAreaRect(temp[r]);
        double major_axis = std::max(minarea_rect.size.height, minarea_rect.size.width);
        double minor_axis = std::min(minarea_rect.size.height, minarea_rect.size.width);
        elongation = major_axis / minor_axis;

        elongations_.push_back(elongation);

        if (elongation >= min_elongation){
            contours_.push_back(temp[r]);
        }
    }

//    print(elongations_);
}

void contours::filterMinRectangularity(int min_rectangularity){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_.clear();
    rectangularities_.clear();

    for (size_t r = 0; r < temp.size(); ++r){
        double rectangularity, contour_area;
        cv::RotatedRect minarea_rect;
        minarea_rect = cv::minAreaRect(temp[r]);
        contour_area = cv::contourArea(temp[r]);
        rectangularity = contour_area / minarea_rect.size.area();

//        rectangularities_.push_back(rectangularity);

        if (rectangularity >= min_rectangularity){
            contours_.push_back(temp[r]);
        }
    }
//    print(rectangularities_);
}

void contours::approxAsPoly(){

    std::vector<std::vector<cv::Point>> temp;
    temp = contours_;
    contours_rect_.clear();

    for(size_t l = 0; l < temp.size(); ++l){
        double epsilon;
        epsilon = 3;
        std::vector<cv::Point> approx;
        cv::approxPolyDP(temp[l], approx, epsilon, true);
        contours_rect_.push_back(approx);
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

bool contours::isSquare(std::vector<cv::Point> contour_){
    double elongation, rectangularity, contour_area;
    cv::RotatedRect minarea_rect;
    minarea_rect = cv::minAreaRect(contour_);
    double major_axis = std::max(minarea_rect.size.height, minarea_rect.size.width);
    double minor_axis = std::min(minarea_rect.size.height, minarea_rect.size.width);
    elongation = major_axis / minor_axis;

    contour_area = cv::contourArea(contour_);
    rectangularity = contour_area / minarea_rect.size.area();

    if(elongation > 0.5 && elongation < 1.3 && rectangularity > 0.8){
        return true;
    } else {
        return false;
    }
}

void contours::fitSquare(){

    contours_rect_.clear();
    for(size_t i = 0; i < contours_.size(); ++i){
        if (isSquare(contours_[i])){
            fitMinAreaRect();
        } else {
            contours_rect_.push_back(contours_[i]);
        }
    }
}


void contours::print(std::vector<double> property){

    for(size_t i = 0; i < property.size(); ++i){
        std::cout << property[i] << ", ";
    }
    std::cout << std::endl;
}


ImageProcessor::ImageProcessor(){
}

ImageProcessor::~ImageProcessor(){}

void ImageProcessor::process(){


    ip_convexities_.clear();
    ip_elongations_.clear();
    ip_rectangularities_.clear();

//    displayImage("original", original_img_.image, 0);
//    saveImage(original_img_.image);
//    histogram(original_img_);
    thresholdImage(0.3);

    processed_img_.image.convertTo(processed_img_.image, CV_8U);
//    displayImage("threshold", processed_img_.image, 1);

    erodeImage(1);
    dilateImage(2);
    erodeImage(1);
//    displayImage("morphological", processed_img_.image, 2);

    extractContours();
    splitContours();

//    drawContoursIP(med_contours_, "med_contours", 3);
//    drawContoursIP(large_contours_, "large_contours", 4);
    std::cout << "Convexities: " << std::endl;
    med_contours_.filterMinConvexity(0.9); // have another look at the threshold for convexity
//    ip_convexities_ = med_contours_.convexities_;
//    drawContoursIP(med_contours_, "filtered by convexity", 6);
    std::cout << "Elongations: " << std::endl;
    med_contours_.filterMinElongation(3);
//    ip_elongations_ = med_contours_.elongations_;
//    drawContoursIP(med_contours_, "filtered by elongation", 7);
    std::cout << "Rectangularities: " << std::endl;
    med_contours_.filterMinRectangularity(0.6);
//    ip_rectangularities_ = med_contours_.rectangularities_;
//    drawContoursIP(med_contours_, "filtered by rectangularity", 8);
    med_contours_.fitMinAreaRect();
    large_contours_.approxAsPoly();
//    large_contours_.filterMinRectangularity(0.6);
//    large_contours_.fitSquare();
//    drawContoursIP(med_contours_, "med_filtered", 5);
//    drawContoursIP(large_contours_, "large_approx", 6);
    mergeContours();
//    all_contours_.assignIDs();
//    all_contours_.assignColors();

    displayResult();


//    int p = cv::waitKey(0);
//    if (p == 's'){
//        saveImage(original_img_.image);
//    }

}

void ImageProcessor::copyOrigToProc(){
    processed_img_ = original_img_;
}

void ImageProcessor::displayImage(std::string process, cv::Mat img, int n) {

    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(process, 100 + img.cols * n, 50);
    cv::imshow(process, img);
//    saveImage(img);
}

void ImageProcessor::displayResult(){

    colour_img_.image = cv::Mat::zeros(original_img_.image.size(), CV_8UC3);
    rect_img_.image = cv::Mat::zeros(original_img_.image.size(), CV_8UC1);

    for(size_t i = 0; i < all_contours_.contours_.size(); ++i){
        cv::RNG rng;
        cv::Scalar color(rand()&255, rand()&255, rand()&255);
        cv::drawContours(colour_img_.image, all_contours_.contours_, i, color, cv::FILLED);
//        cv::drawContours(processed_img_.image, all_contours_.contours_, i, cv::Scalar(255), cv::FILLED);
    }
    for(size_t j = 0; j < all_contours_.contours_rect_.size(); ++j){
//        cv::drawContours(colour_img_.image, all_contours_.contours_rect_, j, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::drawContours(rect_img_.image, all_contours_.contours_rect_, j, cv::Scalar(255), cv::FILLED);
    }

    final_img_ = rect_img_;
//    displayImage("final", rect_img_.image, 7);
}

void ImageProcessor::saveImage(cv::Mat image_){
    cv::Mat CV_8U_img;
    image_.convertTo(CV_8U_img, CV_8U);

    std::string imagename;
    std::cout << "Enter filename to save image (don't forget .png!): " << std::endl;
    std::cin >> imagename;
    std::string homedir = getenv("HOME");
    if(!homedir.empty()){
      boost::filesystem::path output_path(homedir + "/git/4YP/Figures/");
      if(!boost::filesystem::is_directory(output_path)){
        if(boost::filesystem::create_directory(output_path)){
          std::cout << "Creating directory " << output_path.string() << " ... " << std::endl;
        } else {
          std::cerr << "ERROR: directory " << output_path.string() << " does not exists and couldn't be created." << std::endl;
          return;
        }
      }
      cv::imwrite(output_path.string() + imagename, CV_8U_img);
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

void ImageProcessor::thresholdImage(float threshold_value){

    int threshold_type = cv::ThresholdTypes::THRESH_BINARY_INV;
    double maxval = 255;

    cv::threshold(original_img_.image, processed_img_.image, threshold_value, maxval, threshold_type);

}

void ImageProcessor::fillContours(){
    drawContoursIP(all_contours_, "fill", 3);
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

    // populate all_contours.contours_
    for (size_t v = 0; v < large_contours_.contours_.size(); ++v){
    all_contours_.contours_.push_back(large_contours_.contours_[v]);
    }
    for (size_t w = 0; w < med_contours_.contours_.size(); ++w){
    all_contours_.contours_.push_back(med_contours_.contours_[w]);
    }

    // populate all_contours.contours_rect_
    for (size_t v = 0; v < large_contours_.contours_rect_.size(); ++v){
    all_contours_.contours_rect_.push_back(large_contours_.contours_rect_[v]);
    }
    for (size_t w = 0; w < med_contours_.contours_rect_.size(); ++w){
    all_contours_.contours_rect_.push_back(med_contours_.contours_rect_[w]);
    }
}

void ImageProcessor::drawContoursIP(contours contour, std::string process, int n){
    cv_bridge::CvImage temp;
    temp.image = cv::Mat::zeros(original_img_.image.size(), CV_8U);

    for(size_t i = 0; i < contour.contours_.size(); ++i){
        cv::drawContours(temp.image, contour.contours_, i, cv::Scalar(255), cv::FILLED);
    }

    displayImage(process, temp.image, n);
}

void ImageProcessor::histogram(cv_bridge::CvImage img){
    int bins = 30;
    int histSize = bins;
    float range[] = {0, 2};
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    cv::Mat hist, mask;
    mask = createMask(img);
    mask.convertTo(mask, CV_8U);
    std::cout << "Mask type: " << mask.type() << std::endl;
    cv::calcHist(&img.image, 1, 0, mask, hist, 1, &histSize, &histRange, uniform, accumulate);

    std::cout << "Start Histogram: " << std::endl;
    for (int m = 0; m < histSize ; ++m){
        std::cout << hist.at<float>(m) << ", ";
    }
    std::cout << "END Histogram" << std::endl;

    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/bins );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0) );
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    for (int i = 1; i < histSize; i++){
        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1))), cv::Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
    }

//    cv::imshow("original_img_ histogram", histImage);
//    cv::waitKey(0);
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

    cv::Mat mask_;
    mask_ = cv::Mat::ones(img.image.size(), CV_32F);

    for(int r = 0; r < img.image.rows; r++)
    {
        for(int c = 0; c < img.image.cols; c++)
        {
            if (img.image.at<float>(r,c) == 1)
            {
                mask_.at<float>(r,c) = 0;
            }
        }
    }

//    displayImage("mask", mask_, 8);
    std::cout << "Exited displayImage()" << std::endl;

    return mask_;
}

} //namespace planeseg
