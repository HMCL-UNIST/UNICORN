#include "polygon_filter.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// 생성자: 기본 커널 크기, 맵 파라미터 초기값 설정
PolygonFilter::PolygonFilter() 
    : kernelSize(3, 3), resolution(1.0), origin(0, 0), debug(true) {
    kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);
}

// 소멸자
PolygonFilter::~PolygonFilter() {
}

// YAML 파일에서 맵 정보를 로드하고, 이미지 파일 이름을 이용해 이미지를 불러옴
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

    // YAML 파일에 명시된 이미지 파일을 로드 (그레이스케일)
    image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Error: Could not load image: " << imagePath << std::endl;
        return false;
    }
    if(debug){
        cv::imshow("Loaded Image", image);
        cv::waitKey(0);  // 사용자가 키를 누를 때까지 대기 (또는 원하는 시간(ms) 설정)
    }
    // 이미지 처리 (침식 및 컨투어 추출)
    updateContours();

    return true;
}

// 침식 커널 크기를 설정하고, 컨투어 재계산
void PolygonFilter::setErosionKernelSize(int pix) {
    kernelSize = cv::Size(pix, pix);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);
    updateContours();
}

// 이미지에 침식 연산을 적용한 후 컨투어를 추출
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

// 픽셀 좌표를 글로벌 좌표로 변환
cv::Point2d PolygonFilter::pixelToGlobal(const cv::Point& pixelPoint) {
    return cv::Point2d(origin.x + pixelPoint.x * resolution,
                       origin.y + pixelPoint.y * resolution);
}

// 글로벌 좌표를 픽셀 좌표로 변환한 후, 외부 컨투어 내부에 있는지 판단
bool PolygonFilter::isPointInside(const cv::Point2d& globalPoint) {
    cv::Point pixelPoint(static_cast<int>((globalPoint.x - origin.x) / resolution),
                         static_cast<int>((globalPoint.y - origin.y) / resolution));
    for (const auto& contour : externalContours) {
        double result = cv::pointPolygonTest(contour, pixelPoint, false);
        if (result >= 0) { // 내부 또는 경계 상에 존재하면
            return true;
        }
    }
    return false;
}

// 컨투어를 시각화하여 결과 확인 (디버깅용)
void PolygonFilter::visualizeContours() {
    if (erodedImage.empty()) return;
    cv::Mat result;
    cv::cvtColor(erodedImage, result, cv::COLOR_GRAY2BGR);
    // 외부 컨투어: 빨간색, 내부 컨투어: 파란색
    cv::drawContours(result, externalContours, -1, cv::Scalar(0, 0, 255), 2);
    cv::drawContours(result, internalContours, -1, cv::Scalar(255, 0, 0), 2);
    cv::imshow("Contours (Red: External, Blue: Internal)", result);
    cv::waitKey(0);
}
