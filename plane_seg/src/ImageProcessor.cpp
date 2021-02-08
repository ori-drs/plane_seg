#include "plane_seg/ImageProcessor.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem/operations.hpp>

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

ImageProcessor::ImageProcessor(){
}

ImageProcessor::~ImageProcessor(){}

void ImageProcessor::process(){

    copyOrigToProc();
    displayImage("original");

    thresholdImage(70);
    displayImage("threshold");
    erodeImage(1);
    dilateImage(1);
    dilateImage(2);
    erodeImage(2);
    displayImage("erode/dilate");

    extractContours();
    splitContours();
    drawContours(med_contours_, "med_contours");
    drawContours(large_contours_, "large contours");
    med_contours_.filterMinConvexity(0.8);
    drawContours(med_contours_, "filtered by convexity");
    med_contours_.filterMinElongation(2.5);
    drawContours(med_contours_, "filtered by elongation");
    mergeContours();

    displayResult();
    final_img_ = processed_img_;

    int p = cv::waitKey(0);
    if (p == 's'){
        saveImage(original_img_);
    }
}

void ImageProcessor::copyOrigToProc(){
    processed_img_ = original_img_;
}

void ImageProcessor::displayImage(std::string process){

    cv_bridge::CvImage temp;
    temp = processed_img_;
    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    cv::imshow(process, temp.image);
}

void ImageProcessor::displayResult(){

    final_img_.image = cv::Mat::zeros(original_img_.image.size(), CV_8U);

    for(size_t i = 0; i < all_contours_.contours_.size(); ++i){
        cv::drawContours(final_img_.image, all_contours_.contours_, i, cv::Scalar(255), cv::FILLED);
    }

    cv::namedWindow("final", cv::WINDOW_AUTOSIZE);
    cv::imshow("final", final_img_.image);
}

void ImageProcessor::saveImage(cv_bridge::CvImage image_){
    std::string imagename;
    std::cout << "Enter filename to save image (don't forget .png!): " << std::endl;
    std::cin >> imagename;
    std::string homedir = getenv("HOME");
    if(!homedir.empty()){
      boost::filesystem::path output_path(homedir + "/rosbags/image_processing");
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

    cv::threshold(processed_img_.image, processed_img_.image, threshold_value, maxval, threshold_type);

}

/*
void ImageProcessor::blobDetector(cv_bridge::CvImage image){

    cv::SimpleBlobDetector::Params params;

    params.minThreshold = 10;
    params.maxThreshold = 100;
    params.thresholdStep = 10;

    params.filterByColor = true;
    params.blobColor = 255;

    params.filterByArea = true;
    params.minArea = 1;
    params.maxArea = 200;

    params.filterByCircularity = false;
    params.maxCircularity = 0.1;

    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    params.filterByInertia = false;
    params.maxInertiaRatio = 0.1;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(image.image, keypoints);

    cv::drawKeypoints(image.image, keypoints, detect_img_.image, cv::Scalar(0,0,255), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}
*/

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
    for (size_t v = 0; v < large_contours_.contours_.size(); ++v){
    all_contours_.contours_.push_back(large_contours_.contours_[v]);
    }
    for (size_t w = 0; w < med_contours_.contours_.size(); ++w){
    all_contours_.contours_.push_back(med_contours_.contours_[w]);
    }
}

void ImageProcessor::drawContours(contours contour, std::string process){
    cv_bridge::CvImage temp;
    temp.image = cv::Mat::zeros(original_img_.image.size(), CV_8U);

    for(size_t i = 0; i < contour.contours_.size(); ++i){
        cv::drawContours(temp.image, contour.contours_, i, cv::Scalar(255), cv::FILLED);
    }

    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    cv::imshow(process, temp.image);
}

void ImageProcessor::fourierTransform(cv_bridge::CvImage image){
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(image.image.rows);
    int n = cv::getOptimalDFTSize(image.image.cols);
    cv::copyMakeBorder(image.image, padded, m - image.image.rows, 0, n - image.image.cols, 0, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);
    cv::dft(complexI, complexI);

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    cv::split(complexI, planes);
    cv::magnitude(planes[0], planes[1], planes[0]);
    magI.image = planes[0];

    magI.image += cv::Scalar::all(1);
    cv::log(magI.image, magI.image);

    // crop the spectrum, if it has an odd number of rows or columns
    magI.image = magI.image(cv::Rect(0, 0, magI.image.cols & -2, magI.image.rows & -2));

    // rearrange the quadrants of Fourier image so that the origin is at the image centre
    int cx = magI.image.cols/2;
    int cy = magI.image.cols/2;

    cv::Mat q0(magI.image, cv::Rect(0, 0, cx, cy)); // Top-left
    cv::Mat q1(magI.image, cv::Rect(cx, 0, cx, cy));  // Top-Right
    cv::Mat q2(magI.image, cv::Rect(0, cy, cx, cy));  // Bottom-Left
    cv::Mat q3(magI.image, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

    cv::Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);

    cv::normalize(magI.image, magI.image, 0, 1, CV_MINMAX);

}

} //namespace planeseg
