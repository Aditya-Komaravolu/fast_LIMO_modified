#pragma once

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include "fast_limo/Common.hpp"

namespace fast_limo {

class FrameDumper {
public:
    FrameDumper(const std::string& output_dir);
    
    void saveRawFrame(const pcl::PointCloud<fast_limo::Point>::Ptr& raw_cloud, 
                      double timestamp);
                      
    void saveProcessedFrame(const pcl::PointCloud<fast_limo::Point>::Ptr& processed_cloud, 
                            double timestamp);
                            
    void saveRawToGlobalFrame(const pcl::PointCloud<fast_limo::Point>::Ptr& raw_cloud,
                             const Eigen::Matrix4f& transform,
                             double timestamp);
                             
    void saveTransformMatrix(const Eigen::Matrix4f& transform,
                            double timestamp);
private:
    std::string output_dir_;
    std::string ensureDirectoryExists(const std::string& dir);
};

} // namespace fast_limo 