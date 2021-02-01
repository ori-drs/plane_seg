#include "plane_seg/ImageProcessor.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem/operations.hpp>

namespace planeseg {

ImageProcessor::ImageProcessor(){
}

ImageProcessor::~ImageProcessor(){}

void ImageProcessor::process(){
    displayImage(original_img_, "original");
/*
    std::cout << "Press 's' to save, 't' to threshold, anything else to close" << std::endl;
//    displayImage(original_img_, "white NaN");
    int l = cv::waitKey(0);
    if (l == 's'){
        saveImage(original_img_);
    }
    else if (l == 't'){
        thresholdImage(original_img_);
        displayImage(threshold_img_, "threshold");
        std::cout << "Press 's' to save both images, original then thresholded, or 'd' to dilate!" << std::endl;
        int k = cv::waitKey(0);
        if (k == 's'){
            saveImage(original_img_);
            saveImage(threshold_img_);
        } else if (k == 'd'){
            dilateImage(threshold_img_);
            displayImage(dilate_img_, "dilated");
            std::cout <<"Press 's' to save all three images, original, threshold, then dilated; 'e' to erode" << std::endl;
            int j = cv::waitKey(0);
            if (j == 's'){
                saveImage(original_img_);
                saveImage(threshold_img_);
                saveImage(dilate_img_);
            } else if (j == 'e'){
                erodeImage(dilate_img_);
                displayImage(erode_img_, "eroded");
                std::cout <<"Press 's' to save all four images, original, threshold, dilated then eroded" << std::endl;
                int h = cv::waitKey(0);
                if (h == 's'){
                    saveImage(original_img_);
                    saveImage(threshold_img_);
                    saveImage(dilate_img_);
                    saveImage(erode_img_);
                }
            }
        }
    }
    */
    thresholdImage(original_img_);
    erodeImage(threshold_img_, 1);
    dilateImage(erode_img_, 1);
    dilateImage(dilate_img_, 2);
    erodeImage(dilate_img_, 2);
    removeBlobs(erode_img_);

//    blobDetector(erode_img_);
//    erodeImage(erode_img_);
//    dilateImage(erode_img_);

    displayImage(threshold_img_, "threshold");
    displayImage(dilate_img_, "dilated");
    displayImage(erode_img_, "eroded");
    displayImage(large_contours_, "large contours");
    displayImage(small_contours_, "small contours");


    int p = cv::waitKey(0);
    if (p == 's'){
        saveImage(dilate_img_);
    }
}


void ImageProcessor::displayImage(cv_bridge::CvImage image, std::string process){
    cv::namedWindow(process, cv::WINDOW_AUTOSIZE);
    cv::imshow(process, image.image);
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

void ImageProcessor::erodeImage(cv_bridge::CvImage originalImage, int erode_size){

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2*erode_size + 1, 2*erode_size + 1),
                                                cv::Point(erode_size, erode_size)
                                                );
    cv::erode(originalImage.image, erode_img_.image, element);
}

void ImageProcessor::dilateImage(cv_bridge::CvImage originalImage, int dilate_size){

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2*dilate_size+1, 2*dilate_size +1),
                                                cv::Point(dilate_size, dilate_size)
                                                );
    cv::dilate(originalImage.image, dilate_img_.image, element);
}

void ImageProcessor::thresholdImage(cv_bridge::CvImage image){

    int threshold_value = 70;
    int threshold_type = cv::ThresholdTypes::THRESH_BINARY_INV;
    double maxval = 255;

    cv::threshold(image.image, threshold_img_.image, threshold_value, maxval, threshold_type);

}


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


void ImageProcessor::removeBlobs(cv_bridge::CvImage image){

    large_contours_.image = cv::Mat::zeros(image.image.size(), CV_8U);
    small_contours_.image = cv::Mat::zeros(image.image.size(), CV_8U);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> large_contours;
    std::vector<std::vector<cv::Point>> small_contours;

    cv::findContours(image.image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    for(size_t k = 0; k < contours.size(); ++k){
        double area = cv::contourArea(contours[k]);

        if (area >= 200){
            large_contours.push_back(contours[k]);
        }
        else{
            small_contours.push_back(contours[k]);
        }

     for(size_t i = 0; i < large_contours.size(); ++i){
        cv::drawContours(large_contours_.image, large_contours, i, cv::Scalar(255), cv::FILLED);
     }

     for(size_t j = 0; j < small_contours.size(); ++j){
        cv::drawContours(small_contours_.image, small_contours, j, cv::Scalar(255), cv::FILLED);
     }
    }
}


} //namespace planeseg
