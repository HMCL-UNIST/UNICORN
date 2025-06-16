#ifndef GRID_FILTER_H
#define GRID_FILTER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class GridFilter {
public:
    GridFilter();
    ~GridFilter();

    // Load map information from YAML file (image file name, resolution, origin, thresholds, etc.)
    bool loadMapFromYAML(const std::string& yamlPath, const std::string& imagePath);

    // Set erosion kernel size and update contours
    void setErosionKernelSize(int pix);

    // Convert pixel coordinates to global coordinates (global = origin + pixel * resolution)
    cv::Point2d pixelToGlobal(const cv::Point& pixelPoint);

    // Check if the given global coordinates are inside the external contour
    bool isPointInside(const float x, const float y);
    bool isPointInside(const double x, const double y);

private:
    // Perform erosion and update contours after image processing
    void updateImage();

    cv::Mat image;             // Grayscale image specified in YAML
    cv::Mat erodedImage;       // Image after erosion operation
    cv::Mat kernel;            // Kernel used for erosion
    cv::Size kernelSize;       // Size of the kernel

    // Contour information
    std::vector<std::vector<cv::Point>> externalContours; // External contours
    std::vector<std::vector<cv::Point>> internalContours; // Internal contours

    // Map parameters (read from YAML)
    double resolution;         // Resolution (real-world length per pixel)
    cv::Point2d origin;        // Global origin corresponding to pixel (0,0)
    std::vector<double> originVec;
    double free_thresh;
    double occupied_thresh;
    int negate;                // Whether to invert occupancy map (0 or 1)
    bool debug;
};

#endif // GRID_FILTER_H
