#include "fast_limo/Utils/FrameDumper.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

namespace fast_limo {

FrameDumper::FrameDumper(const std::string& output_dir) 
    : output_dir_(output_dir) {
    ensureDirectoryExists(output_dir_);
    ensureDirectoryExists(output_dir_ + "/raw");
    ensureDirectoryExists(output_dir_ + "/processed");
    ensureDirectoryExists(output_dir_ + "/raw_global");
    ensureDirectoryExists(output_dir_ + "/transforms");
}

std::string FrameDumper::ensureDirectoryExists(const std::string& dir) {
    struct stat info;
    if (stat(dir.c_str(), &info) != 0) {
        // Directory doesn't exist, create it
        int ret = mkdir(dir.c_str(), 0755);
        if (ret != 0) {
            std::cerr << "Error creating directory: " << dir << std::endl;
        }
    }
    return dir;
}

void FrameDumper::saveRawFrame(const pcl::PointCloud<fast_limo::Point>::Ptr& raw_cloud, 
                              double timestamp) {
    std::stringstream ss;
    ss << output_dir_ << "/raw/frame_" << std::fixed << std::setprecision(6) << timestamp << ".pcd";
    pcl::io::savePCDFileBinary(ss.str(), *raw_cloud);
}

void FrameDumper::saveProcessedFrame(const pcl::PointCloud<fast_limo::Point>::Ptr& processed_cloud, 
                                    double timestamp) {
    std::stringstream ss;
    ss << output_dir_ << "/processed/frame_" << std::fixed << std::setprecision(6) << timestamp << ".pcd";
    pcl::io::savePCDFileBinary(ss.str(), *processed_cloud);
}

void FrameDumper::saveRawToGlobalFrame(const pcl::PointCloud<fast_limo::Point>::Ptr& raw_cloud,
                                     const Eigen::Matrix4f& transform,
                                     double timestamp) {
    pcl::PointCloud<fast_limo::Point>::Ptr transformed_cloud(new pcl::PointCloud<fast_limo::Point>());
    pcl::transformPointCloud(*raw_cloud, *transformed_cloud, transform);
    
    std::stringstream ss;
    ss << output_dir_ << "/raw_global/frame_" << std::fixed << std::setprecision(6) << timestamp << ".pcd";
    pcl::io::savePCDFileBinary(ss.str(), *transformed_cloud);
}

void FrameDumper::saveTransformMatrix(const Eigen::Matrix4f& transform,
                                    double timestamp) {
    std::stringstream ss;
    ss << output_dir_ << "/transforms/transform_" << std::fixed << std::setprecision(6) << timestamp << ".txt";
    
    std::ofstream file(ss.str());
    if (file.is_open()) {
        file << transform;
        file.close();
    }
}

} // namespace fast_limo 