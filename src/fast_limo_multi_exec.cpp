// Add additional publishers
#include <ros/ros.h>

// ros::Publisher loop_corrected_cloud_pub;
// ros::Publisher loop_corrected_odom_pub;

// In the main function, after other publishers are created:
ros::Publisher loop_corrected_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("loop_corrected_cloud", 1);
ros::Publisher loop_corrected_odom_pub = nh.advertise<nav_msgs::Odometry>("loop_corrected_odom", 1);

// In the main loop, after publishing other data:
if (config.loop_closure.active) {
    // Publish loop-corrected point cloud
    pcl::PointCloud<PointType>::Ptr loop_corrected_cloud = localizer.get_loop_corrected_pointcloud();
    if (loop_corrected_cloud && !loop_corrected_cloud->empty()) {
        sensor_msgs::PointCloud2 loop_cloud_msg;
        pcl::toROSMsg(*loop_corrected_cloud, loop_cloud_msg);
        loop_cloud_msg.header.frame_id = config.frames.world;
        loop_cloud_msg.header.stamp = ros::Time::now();
        loop_corrected_cloud_pub.publish(loop_cloud_msg);
    }
    
    // Publish loop-corrected odometry
    nav_msgs::Odometry loop_odom = localizer.get_loop_corrected_odometry();
    if (loop_odom.header.stamp.toSec() > 0) {
        loop_corrected_odom_pub.publish(loop_odom);
    }
} 