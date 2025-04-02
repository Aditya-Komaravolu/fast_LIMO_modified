/**
 * Loop Closure Integration for Fast-LIMO
 * Implementation file
 */

#include "fast_limo/Utils/loop_closure_integration.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

namespace loop_closure {

// Publishers
static ros::Publisher cloud_pub;
static ros::Publisher odom_pub;
static std::string world_frame_id = "map";  // Default value

// Initialize publishers
void init(ros::NodeHandle& nh, const std::string& world_frame) {
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("loop_corrected_cloud", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("loop_corrected_odom", 1);
    world_frame_id = world_frame;
}

// Publish loop closure results
void publish(fast_limo::Localizer& localizer, const fast_limo::Config& config) {
    if (config.loop_closure.active) {
        // Publish loop-corrected point cloud
        pcl::PointCloud<PointType>::Ptr loop_corrected_cloud = localizer.get_loop_corrected_pointcloud();
        if (loop_corrected_cloud && !loop_corrected_cloud->empty()) {
            sensor_msgs::PointCloud2 loop_cloud_msg;
            pcl::toROSMsg(*loop_corrected_cloud, loop_cloud_msg);
            loop_cloud_msg.header.frame_id = world_frame_id;
            loop_cloud_msg.header.stamp = ros::Time::now();
            cloud_pub.publish(loop_cloud_msg);
        }
        
        // Publish loop-corrected odometry
        nav_msgs::Odometry loop_odom = localizer.get_loop_corrected_odometry();
        if (loop_odom.header.stamp.toSec() > 0) {
            odom_pub.publish(loop_odom);
        }
    }
}

} // namespace loop_closure 