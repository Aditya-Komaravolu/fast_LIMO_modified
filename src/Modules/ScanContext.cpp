#include "fast_limo/Modules/ScanContext.hpp"
#include <cmath>
#include <limits>

namespace fast_limo {

cv::Mat ScanContext::generateContext(const pcl::PointCloud<PointType>::Ptr& cloud) {
    // Initialize empty descriptor
    cv::Mat descriptor = cv::Mat::zeros(rings_, sectors_, CV_32F);
    std::vector<std::vector<float>> height_per_bin(rings_, std::vector<float>(sectors_, -std::numeric_limits<float>::max()));
    
    // Process each point
    for (const auto& point : cloud->points) {
        // Convert to polar coordinates
        double x = point.x;
        double y = point.y;
        double z = point.z;
        double radius = std::sqrt(x*x + y*y);
        double angle = std::atan2(y, x);
        
        // Skip points outside max radius
        if (radius > max_radius_) continue;
        
        // Normalize height
        double height = std::min(std::max(z, 0.0), max_height_);
        
        // Calculate bin indices
        int ring_idx = std::min(static_cast<int>(radius / max_radius_ * rings_), rings_ - 1);
        int sector_idx = std::min(static_cast<int>((angle + M_PI) / (2 * M_PI) * sectors_), sectors_ - 1);
        
        // Update maximum height in this bin
        height_per_bin[ring_idx][sector_idx] = std::max(height_per_bin[ring_idx][sector_idx], static_cast<float>(height));
    }
    
    // Fill descriptor with normalized heights
    for (int r = 0; r < rings_; ++r) {
        for (int s = 0; s < sectors_; ++s) {
            if (height_per_bin[r][s] > -std::numeric_limits<float>::max()) {
                descriptor.at<float>(r, s) = height_per_bin[r][s] / max_height_;
            }
        }
    }
    
    return descriptor;
}

std::pair<int, double> ScanContext::findBestMatch(const cv::Mat& query, double threshold) {
    if (descriptors_.empty()) {
        return {-1, std::numeric_limits<double>::max()};
    }
    
    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::max();
    
    // Search for the best match
    for (size_t i = 0; i < descriptors_.size(); ++i) {
        // Skip very recent descriptors to avoid false positives
        if (i >= descriptors_.size() - 30) continue;
        
        double dist = distance(query, descriptors_[i]);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }
    
    // No match if distance is above threshold
    if (best_dist > threshold) {
        return {-1, best_dist};
    }
    
    return {best_idx, best_dist};
}

void ScanContext::addDescriptor(const cv::Mat& descriptor) {
    descriptors_.push_back(descriptor.clone());
}

double ScanContext::distance(const cv::Mat& desc1, const cv::Mat& desc2) {
    // Implement cosine distance for rotation invariance
    cv::Mat flat1 = desc1.reshape(1, 1);
    cv::Mat flat2 = desc2.reshape(1, 1);
    
    double dot = flat1.dot(flat2);
    double norm1 = cv::norm(flat1);
    double norm2 = cv::norm(flat2);
    
    if (norm1 < 1e-5 || norm2 < 1e-5) return 2.0; // Maximum distance
    
    double cosine_sim = dot / (norm1 * norm2);
    return 1.0 - cosine_sim; // Convert similarity to distance
}

} // namespace fast_limo 