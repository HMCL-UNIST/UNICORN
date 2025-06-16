#include "grid_filter.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Constructor: Set default kernel size and map parameters
GridFilter::GridFilter() 
    : kernelSize(3, 3), resolution(1.0), origin(0, 0), debug(false) {
    kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);
}

// Destructor
GridFilter::~GridFilter() {
}

// Load map info from YAML file and load the image using the image file name
bool GridFilter::loadMapFromYAML(const std::string& yamlPath, const std::string& imagePath) {
    std::cout << "Try to open YAML file: " << yamlPath << std::endl;
    YAML::Node config = YAML::LoadFile(yamlPath);
    if (!config) {
        std::cerr << "Error: Could not load YAML file: " << yamlPath << std::endl;
        return false;
    }

    originVec = config["origin"].as<std::vector<double>>();
    resolution = config["resolution"].as<double>();

    std::cout << "origin: ";
    for (double v : originVec)
        std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "resolution: " << resolution << std::endl;

    // Load the image specified in the YAML file (grayscale)
    image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    cv::flip(image, image, 0);
    if (image.empty()) {
        std::cerr << "Error: Could not load image: " << imagePath << std::endl;
        return false;
    }
    if(debug){
        cv::imshow("Loaded Image", image);
        cv::waitKey(0);  // Wait for key press (or set a desired time in ms)
    }
    // Image processing (erosion and contour extraction)
    updateImage();

    return true;
}

// Set erosion kernel size and recompute contours
void GridFilter::setErosionKernelSize(int pix) {
    kernelSize = cv::Size(pix, pix);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);
    updateImage();
}

// Apply erosion to the image and extract contours
void GridFilter::updateImage() {
    if (image.empty()) {
        std::cerr << "Error: Image not initialized." << std::endl;
        return;
    }
    cv::erode(image, erodedImage, kernel);
}

// Convert pixel coordinates to global coordinates
cv::Point2d GridFilter::pixelToGlobal(const cv::Point& pixelPoint) {
    return cv::Point2d(origin.x + pixelPoint.x * resolution,
                       origin.y + pixelPoint.y * resolution);
}


bool GridFilter::isPointInside(const float x, const float y) {
    cv::Point pixelPoint(static_cast<int>((x - originVec[0]) / resolution),
                         static_cast<int>((y - originVec[1]) / resolution));

    if(debug){
        cv::circle(erodedImage, pixelPoint, 3, cv::Scalar(128), -1);
        cv::imshow("Loaded Image", erodedImage);
        cv::waitKey(0); 
    }

    if (pixelPoint.x < 0 || pixelPoint.y < 0 ||
        pixelPoint.x >= erodedImage.cols || pixelPoint.y >= erodedImage.rows) {
        return false;
    }

    uchar pixelValue = erodedImage.at<uchar>(pixelPoint.y, pixelPoint.x);

    return (pixelValue == 255);

    return false;
}

bool GridFilter::isPointInside(const double x, const double y) {
    cv::Point pixelPoint(static_cast<int>((x - originVec[0]) / resolution),
                         static_cast<int>((y - originVec[1]) / resolution));

    if(debug){
        cv::circle(erodedImage, pixelPoint, 3, cv::Scalar(128), -1);
        cv::imshow("Loaded Image", erodedImage);
        cv::waitKey(0);  // Wait for key press (or set a desired time in ms)
    }

    // Check if the pixel coordinates are within the image bounds
    if (pixelPoint.x < 0 || pixelPoint.y < 0 ||
        pixelPoint.x >= erodedImage.cols || pixelPoint.y >= erodedImage.rows) {
        return false;
    }

    uchar pixelValue = erodedImage.at<uchar>(pixelPoint.y, pixelPoint.x);

    return (pixelValue == 255);

    return false;
}
