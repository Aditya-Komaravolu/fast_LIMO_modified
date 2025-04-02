#ifndef SCAN_CONTEXT_HPP
#define SCAN_CONTEXT_HPP

#include <Eigen/Dense>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "fast_limo/Common.hpp"
#include <opencv2/opencv.hpp>

namespace fast_limo {

class ScanContext {
public:
    ScanContext(int rings = 20, int sectors = 60, 
                double max_radius = 80.0, double max_height = 2.0) 
        : rings_(rings), sectors_(sectors), 
          max_radius_(max_radius), max_height_(max_height) {}

    // Generate scan context descriptor from point cloud
    cv::Mat generateContext(const pcl::PointCloud<PointType>::Ptr& cloud);
    
    // Find best match from database
    std::pair<int, double> findBestMatch(const cv::Mat& query, double threshold = 0.15);
    
    // Add descriptor to database
    void addDescriptor(const cv::Mat& descriptor);
    
    // Get all descriptors
    const std::vector<cv::Mat>& getDescriptors() const { return descriptors_; }

private:
    int rings_;
    int sectors_;
    double max_radius_;
    double max_height_;
    std::vector<cv::Mat> descriptors_;

    // Calculate distance between two descriptors
    double distance(const cv::Mat& desc1, const cv::Mat& desc2);
};

} // namespace fast_limo

#endif // SCAN_CONTEXT_HPP 