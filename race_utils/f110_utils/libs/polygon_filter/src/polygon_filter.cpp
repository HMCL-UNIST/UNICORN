#include "polygon_filter.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// constructor: set default kernel size and initial map parameters
PolygonFilter::PolygonFilter() 
    : kernelSize(3, 3), resolution(1.0), origin(0, 0), debug(true) {
    kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);
}

// destructor
PolygonFilter::~PolygonFilter() {
}

// load map info from the YAML file and read the image using the image filename
bool PolygonFilter::loadMapFromYAML(const std::string& yamlPath, const std::string& imagePath) {
    std::cout << "Try to open YAML file: " << yamlPath << std::endl;
    YAML::Node config = YAML::LoadFile(yamlPath);
    if (!config) {
        std::cerr << "Error: Could not load YAML file: " << yamlPath << std::endl;
        return false;
    }

    std::vector<double> originVec = config["origin"].as<std::vector<double>>();
    double resolution = config["resolution"].as<double>();

    std::cout << "origin: ";
    for (double v : originVec)
        std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "resolution: " << resolution << std::endl;

    // load the image file specified in the YAML (grayscale)
    image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Error: Could not load image: " << imagePath << std::endl;
        return false;
    }
    if(debug){
        cv::imshow("Loaded Image", image);
        cv::waitKey(0);  // wait until a key is pressed (or set a desired time in ms)
    }
    // image processing (erosion and contour extraction)
    updateContours();

    return true;
}

// set the erosion kernel size and recompute the contours
void PolygonFilter::setErosionKernelSize(int pix) {
    if (pix < 1) pix = 1;  // guard: 0/negative -> empty kernel -> cv::erode aborts (normalizeAnchor)
    kernelSize = cv::Size(pix, pix);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);
    updateContours();
}

// apply erosion to the image, then extract contours
void PolygonFilter::updateContours() {
    if (image.empty()) {
        std::cerr << "Error: Image not initialized." << std::endl;
        return;
    }
    cv::erode(image, erodedImage, kernel);

    std::vector<std::vector<cv::Point>> allContours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(erodedImage, allContours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    externalContours.clear();
    internalContours.clear();

    for (size_t i = 0; i < allContours.size(); i++) {
        if (hierarchy[i][3] == -1)
            externalContours.push_back(allContours[i]);
        else
            internalContours.push_back(allContours[i]);
    }

    if(debug){
        visualizeContours();
    }

    
}

// convert pixel coordinates to global coordinates
cv::Point2d PolygonFilter::pixelToGlobal(const cv::Point& pixelPoint) {
    return cv::Point2d(origin.x + pixelPoint.x * resolution,
                       origin.y + pixelPoint.y * resolution);
}

// convert global coordinates to pixel coordinates, then test if inside the external contour
bool PolygonFilter::isPointInside(const cv::Point2d& globalPoint) {
    cv::Point pixelPoint(static_cast<int>((globalPoint.x - origin.x) / resolution),
                         static_cast<int>((globalPoint.y - origin.y) / resolution));
    for (const auto& contour : externalContours) {
        double result = cv::pointPolygonTest(contour, pixelPoint, false);
        if (result >= 0) { // inside or on the boundary
            return true;
        }
    }
    return false;
}

// visualize the contours to inspect the result (for debugging)
void PolygonFilter::visualizeContours() {
    if (erodedImage.empty()) return;
    cv::Mat result;
    cv::cvtColor(erodedImage, result, cv::COLOR_GRAY2BGR);
    // external contours: red, internal contours: blue
    cv::drawContours(result, externalContours, -1, cv::Scalar(0, 0, 255), 2);
    cv::drawContours(result, internalContours, -1, cv::Scalar(255, 0, 0), 2);
    cv::imshow("Contours (Red: External, Blue: Internal)", result);
    cv::waitKey(0);
}
