#ifndef POLYGON_FILTER_H
#define POLYGON_FILTER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class PolygonFilter {
public:
    PolygonFilter();
    ~PolygonFilter();

    // YAML 파일 경로를 받아 맵 정보를 로드 (이미지 파일 이름, 해상도, 원점, 임계치 등)
    bool loadMapFromYAML(const std::string& yamlPath, const std::string& imagePath);

    // 침식 커널 크기를 설정하고 컨투어를 갱신
    void setErosionKernelSize(int pix);

    // 픽셀 좌표를 글로벌 좌표로 변환 (global = origin + pixel * resolution)
    cv::Point2d pixelToGlobal(const cv::Point& pixelPoint);

    // 전달된 글로벌 좌표가 외부 컨투어 내부에 포함되는지 판단
    bool isPointInside(const cv::Point2d& globalPoint);

    // 컨투어 결과를 시각화 (디버깅용)
    void visualizeContours();

private:
    // 이미지 처리 후 침식 및 컨투어 업데이트
    void updateContours();

    cv::Mat image;             // YAML에 명시된 이미지 (그레이스케일)
    cv::Mat erodedImage;       // 침식 연산 후의 이미지
    cv::Mat kernel;            // 침식 연산에 사용되는 커널
    cv::Size kernelSize;       // 커널 크기

    // 컨투어 정보
    std::vector<std::vector<cv::Point>> externalContours; // 외부 컨투어
    std::vector<std::vector<cv::Point>> internalContours; // 내부 컨투어

    // 맵 파라미터 (YAML에서 읽음)
    double resolution;         // 해상도 (픽셀당 실제 길이)
    cv::Point2d origin;        // 픽셀 (0,0)에 대응하는 글로벌 원점
    double free_thresh;
    double occupied_thresh;
    int negate;                // Occupancy map의 반전 여부 (0 또는 1)
    bool debug;
};

#endif // POLYGON_FILTER_H
