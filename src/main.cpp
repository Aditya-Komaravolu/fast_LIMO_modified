#include "ROSutils.hpp"
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <fast_limo/manualTrigger.h> 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <condition_variable>


// output publishers
ros::Publisher pc_pub;
ros::Publisher state_pub;

// debugging publishers
ros::Publisher orig_pub, desk_pub, match_pub, finalraw_pub, body_pub, map_bb_pub, match_points_pub;

// output frames
std::string world_frame, body_frame;

std::string base_path;

double latest_lidar_timestamp;

condition_variable sig_buffer;

bool save_pcd_by_parts = false;

bool flg_exit = false;
// Service callback for setting voxel leaf size
bool set_leaf_size_callback(fast_limo::manualTrigger::Request &req, fast_limo::manualTrigger::Response &res) {
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    // Append the latest lidar timestamp to a text file
    std::ofstream timestamp_file;
    timestamp_file.open(base_path + "/timestamps.txt", std::ios_base::app);
    timestamp_file << std::fixed << std::setprecision(9) << latest_lidar_timestamp << std::endl;
    timestamp_file.close();
    

    loc.set_voxel_leaf_size(req.leafSize);
    res.success = true;
    res.message = "Changed leaf size to " + std::to_string(req.leafSize);
    return true;
}

void env_callback(const std_msgs::String::ConstPtr& msg){

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Config& config = loc.get_config();
    
    if (config.button_trigger){
        loc.append_msg_to_env_buffer(msg);
    }
    // else {std::cout << "Button trigger not enabled!" << endl;}
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *pc_);

    latest_lidar_timestamp = msg->header.stamp.toSec();

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    loc.updatePointCloud(pc_, msg->header.stamp.toSec());

    // Publish output pointcloud
    sensor_msgs::PointCloud2 pc_ros;
    pcl::toROSMsg(*loc.get_pointcloud(), pc_ros);
    pc_ros.header.stamp = ros::Time::now();
    pc_ros.header.frame_id = world_frame;
    pc_pub.publish(pc_ros);

    // Publish debugging pointclouds
    sensor_msgs::PointCloud2 orig_msg;
    pcl::toROSMsg(*loc.get_orig_pointcloud(), orig_msg);
    orig_msg.header.stamp = ros::Time::now();
    orig_msg.header.frame_id = body_frame;
    orig_pub.publish(orig_msg);

    sensor_msgs::PointCloud2 deskewed_msg;
    pcl::toROSMsg(*loc.get_deskewed_pointcloud(), deskewed_msg);
    deskewed_msg.header.stamp = ros::Time::now();
    deskewed_msg.header.frame_id = world_frame;
    desk_pub.publish(deskewed_msg);

    sensor_msgs::PointCloud2 match_msg;
    pcl::toROSMsg(*loc.get_pc2match_pointcloud(), match_msg);
    match_msg.header.stamp = ros::Time::now();
    match_msg.header.frame_id = body_frame;
    match_pub.publish(match_msg);

    sensor_msgs::PointCloud2 finalraw_msg;
    pcl::toROSMsg(*loc.get_finalraw_pointcloud(), finalraw_msg);
    finalraw_msg.header.stamp = ros::Time::now();
    finalraw_msg.header.frame_id = world_frame;
    finalraw_pub.publish(finalraw_msg);

    // Visualize current map size
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
    visualization_msgs::Marker bb_marker = visualize_limo::getLocalMapMarker(map.get_local_map());
    bb_marker.header.frame_id = world_frame;
    map_bb_pub.publish(bb_marker);

    // Visualize current matches
    visualization_msgs::MarkerArray match_markers = visualize_limo::getMatchesMarker(loc.get_matches(), 
                                                                                    world_frame
                                                                                    );
    match_points_pub.publish(match_markers);

}

// void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

//     // pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
//     // pcl::fromROSMsg(*msg, *pc_);
//     fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();

//     if (msg->header.stamp.toSec() < loc.last_timestamp_lidar) {
//         ROS_ERROR("lidar loop back, clear buffer");
//         lidar_buffer.clear();
//     }


//     pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());
//     loc.preprocess_lidar_msg(msg, ptr);
//     latest_lidar_timestamp = msg->header.stamp.toSec();


//     fast_limo::Config& lidar_buffer = loc.get_lidar_buffer();

//     lidar_buffer.push_back(ptr);
//     // time_buffer.push_back(msg->header.stamp.toSec() + time_offset);
//     loc.last_timestamp_lidar = msg->header.stamp.toSec();
//     // mtx_buffer.unlock();
//     // sig_buffer.notify_all();

// }

// void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg){
//     sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*imu_msg));

//     // msg->header.stamp = ros::Time().fromSec(imu_msg->header.stamp.toSec() - time_diff_lidar_to_imu);

//     // if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
//     //     msg->header.stamp =
//     //         ros::Time().fromSec(timediff_lidar_wrt_imu + imu_msg->header.stamp.toSec());
//     // }

//     double timestamp = msg->header.stamp.toSec();

//     if (timestamp < last_timestamp_imu) {
//         ROS_WARN("imu loop back, ignoring!!!");
//         ROS_WARN("current T: %f, last T: %f", timestamp, last_timestamp_imu);
//         return;
//     }

//     last_timestamp_imu = timestamp;
//     last_imu_processed_time = ros::Time::now().toSec();

//     mtx_buffer.lock();

//     imu_buffer.push_back(msg);
//     // mtx_buffer.unlock();
//     // sig_buffer.notify_all();
// }

// void env_callback(const sensor_msgs::String::ConstPtr &msg) {
//     // mtx_buffer.lock();
//     std::cout << "current env location: " << msg->data << std::endl;
//     std::string data = msg->data;
    
//     // Extract room type
//     std::string room_type = data.substr(data.find("room:")+6, data.find(",")-6);
//     // Remove any whitespace
//     room_type.erase(std::remove_if(room_type.begin(), room_type.end(), ::isspace), room_type.end());
    
//     // Extract seconds and nanoseconds
//     std::string secsSubstring = data.substr(data.find("secs:")+5, data.find(",", data.find("secs:"))-data.find("secs:")-5);
//     std::string nsecsSubstring = data.substr(data.find("nsecs:")+6);
    
//     std::cout << "Room type: " << room_type << std::endl;
//     std::cout << "Msg secs: " << secsSubstring << std::endl;
//     std::cout << "Msg nsecs: " << nsecsSubstring << std::endl;
    
//     long secs, nsecs;
//     std::istringstream(secsSubstring) >> secs;
//     std::istringstream(nsecsSubstring) >> nsecs;
//     double current_timestamp = static_cast<double>(secs) + static_cast<double>(nsecs) * 1e-9;
//     ros::Time ros_timestamp;
//     ros_timestamp.fromSec(current_timestamp);

//     std::string small_str = "small";
//     std::string medium_str = "medium";
//     std::string large_str = "large";

//     if (room_type == small_str) {
//         std::cout << "Entered small room. Setting LEAF SIZE: ";
//         for (float size : this->config.filters.small_room_leafSize) {
//             std::cout << size << " ";
//         }
//         std::cout << std::endl;

//         // // Set threshold parameters
//         thres_ptr.header.stamp = ros_timestamp;
//         thres_ptr.leafSize = this->config.filters.small_room_leafSize;
//         thres_ptr.env = "small";

//         thresholds::mapping_tweak_values::ConstPtr push_thres = boost::make_shared<thresholds::mapping_tweak_values>(thres_ptr);
//         this->env_buffer.push_back(push_thres);

//     } 
//     else if (room_type == medium_str) {
//         std::cout << "Entered Medium Room. Setting LEAF SIZE: " ;
//         for (float size : this->config.filters.medium_room_leafSize) {
//             std::cout << size << " ";
//         }
//         std::cout << std::endl;
        
//         thres_ptr.header.stamp = ros_timestamp;
//         thres_ptr.leafSize = this->config.filters.medium_room_leafSize;
//         thres_ptr.env = "medium";
        
//         thresholds::mapping_tweak_values::ConstPtr push_thres = boost::make_shared<thresholds::mapping_tweak_values>(thres_ptr);
//         this->env_buffer.push_back(push_thres);
//     }
//     else if (room_type == large_str) {
//         std::cout << "Entered Large Room. Setting LEAF SIZE: ";
//         for (float size : this->config.filters.leafSize) {
//             std::cout << size << " ";
//         }
//         std::cout << std::endl;
//         thres_ptr.header.stamp = ros_timestamp;
//         thres_ptr.leafSize = this->config.filters.leafSize;
//         thres_ptr.env = "large";


//         thresholds::mapping_tweak_values::ConstPtr push_thres = boost::make_shared<thresholds::mapping_tweak_values>(thres_ptr);

//         this->env_buffer.push_back(push_thres);
//         }
//     else {
//         // ROS_WARN("Room not specified! Room type not recognized: %s", room_type.c_str());
//         std::cout << "Room not specified! Room type not recognized: " << room_type << std::endl;
//     }

//     // mtx_buffer.unlock();
//     // sig_buffer.notify_all();

// }

// void publish_pointcloud_aftermapped(fast_limo::Localizer& loc) {
//     // Publish output pointcloud
//     sensor_msgs::PointCloud2 pc_ros;
//     pcl::toROSMsg(*loc.get_pointcloud(), pc_ros);
//     pc_ros.header.stamp = ros::Time::now();
//     pc_ros.header.frame_id = world_frame;
//     pc_pub.publish(pc_ros);

//     // Publish debugging pointclouds
//     sensor_msgs::PointCloud2 orig_msg;
//     pcl::toROSMsg(*loc.get_orig_pointcloud(), orig_msg);
//     orig_msg.header.stamp = ros::Time::now();
//     orig_msg.header.frame_id = body_frame;
//     orig_pub.publish(orig_msg);

//     sensor_msgs::PointCloud2 deskewed_msg;
//     pcl::toROSMsg(*loc.get_deskewed_pointcloud(), deskewed_msg);
//     deskewed_msg.header.stamp = ros::Time::now();
//     deskewed_msg.header.frame_id = world_frame;
//     desk_pub.publish(deskewed_msg);

//     sensor_msgs::PointCloud2 match_msg;
//     pcl::toROSMsg(*loc.get_pc2match_pointcloud(), match_msg);
//     match_msg.header.stamp = ros::Time::now();
//     match_msg.header.frame_id = body_frame;
//     match_pub.publish(match_msg);

//     sensor_msgs::PointCloud2 finalraw_msg;
//     pcl::toROSMsg(*loc.get_finalraw_pointcloud(), finalraw_msg);
//     finalraw_msg.header.stamp = ros::Time::now();
//     finalraw_msg.header.frame_id = world_frame;
//     finalraw_pub.publish(finalraw_msg);

//     // Visualize current map size
//     fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
//     visualization_msgs::Marker bb_marker = visualize_limo::getLocalMapMarker(map.get_local_map());
//     bb_marker.header.frame_id = world_frame;
//     map_bb_pub.publish(bb_marker);

//     // Visualize current matches
//     visualization_msgs::MarkerArray match_markers = visualize_limo::getMatchesMarker(loc.get_matches(), 
//                                                                                     world_frame
//                                                                                     );
//     match_points_pub.publish(match_markers);
// }


// auto process = [&]() {
    
// }

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();

    fast_limo::IMUmeas imu;
    tf_limo::fromROStoLimo(msg, imu);

    // Propagate IMU measurement
    loc.updateIMU(imu);

    loc.last_timestamp_imu = msg->header.stamp.toSec();
    loc.last_imu_processed_time = ros::Time::now().toSec();


    // State publishing
    nav_msgs::Odometry state_msg, body_msg;
    tf_limo::fromLimoToROS(loc.getWorldState(), loc.getPoseCovariance(), loc.getTwistCovariance(), state_msg);
    tf_limo::fromLimoToROS(loc.getBodyState(), loc.getPoseCovariance(), loc.getTwistCovariance(), body_msg);

    // Fill frame id's
    state_msg.header.frame_id = world_frame;
    state_msg.child_frame_id  = body_frame;
    body_msg.header.frame_id  = world_frame;
    body_msg.child_frame_id   = body_frame;

    state_pub.publish(state_msg);
    body_pub.publish(body_msg);

    // TF broadcasting
    tf_limo::broadcastTF(loc.getWorldState(), world_frame, body_frame, true);

}



void save_pcd(){
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Config& config = loc.get_config();

    if (config.save_dense_pcd){
        pcl::PointCloud<PointType>::Ptr cloud = loc.get_accumulated_pointcloud();

        std::string pcd_path = base_path + "/PCD/scans.pcd";
        std::cout << "Saving point cloud to " << pcd_path << std::endl;

        std::cout << "Point cloud size: " << cloud->size() << std::endl;

        // Save the point cloud to a file
        pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(pcd_path, *cloud);
            std::cout << "Saved the final point cloud to " << pcd_path << std::endl;
    }
    
    if (config.save_skewed_pcd){
        pcl::PointCloud<PointType>::Ptr cloud = loc.get_accumulated_downsampled_pointcloud();
        std::string pcd_path = base_path + "/PCD/scans_downsampled.pcd";
        std::cout << "Saving point cloud to " << pcd_path << std::endl;
        std::cout << "Point cloud size: " << cloud->size() << std::endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(pcd_path, *cloud);
        std::cout << "Saved the final point cloud to " << pcd_path << std::endl;
    }

    // if (cloud) {
    //     // Save the point cloud to a file
    //     pcl::PCDWriter pcd_writer;
    //     pcd_writer.writeBinary(std::string(base_path + "/PCD/scans.pcd"), *cloud);
    //     std::cout << "Saved the final point cloud to " << base_path + "/PCD/scans.pcd" << std::endl;
    // }


}

int part_count = 0;
pcl::PointCloud<PointType>::Ptr diff_dense_cloud;
pcl::PointCloud<PointType>::Ptr diff_downsampled_cloud;
bool save_pcd_by_parts_callback(){
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Config& config = loc.get_config();

    pcl::PointCloud<PointType>::Ptr dense_pcl = loc.get_accumulated_pointcloud();
    pcl::PointCloud<PointType>::Ptr downsampled_pcl = loc.get_accumulated_downsampled_pointcloud();

    if (config.save_dense_pcd && config.save_pcd_by_parts && !dense_pcl->empty()){


        std::string pcd_path = base_path + "/PCD/part_" + std::to_string(part_count) + ".pcd";
        std::cout << "Saving point cloud to " << pcd_path << std::endl;

        std::cout << "Point cloud size: " << dense_pcl->size() << std::endl;

        

        // Save the point cloud to a file
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(pcd_path, *dense_pcl);
        dense_pcl->clear();
        std::cout << "Saved the part point cloud to " << pcd_path << std::endl;
        // *diff_dense_cloud += *cloud;
    }

    
    // if (config.save_skewed_pcd && config.save_pcd_by_parts && !downsampled_pcl->empty()){
    //     // pcl::PointCloud<PointType>::Ptr cloud = loc.get_accumulated_downsampled_pointcloud();
    //     std::string pcd_path = base_path + "/PCD/part_downsampled_" + std::to_string(part_count) + ".pcd";
    //     std::cout << "Saving point cloud to " << pcd_path << std::endl;
    //     std::cout << "Point cloud size: " << downsampled_pcl->size() << std::endl;
    //     pcl::PCDWriter pcd_writer;
    //     pcd_writer.writeBinary(pcd_path, *downsampled_pcl);
    //     std::cout << "Saved the final point cloud to " << pcd_path << std::endl;
    //     downsampled_pcl->clear();
    //     // *diff_downsampled_cloud += *cloud;
    // }
    // part_count++;
    // res.success = true;
    // res.message = "Saved pcd part" ;
    // return true;

}

bool break_pcd_on_service_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Config& config = loc.get_config();

    pcl::PointCloud<PointType>::Ptr dense_pcl = loc.get_accumulated_pointcloud();

    if (config.save_dense_pcd && config.save_pcd_by_parts && !dense_pcl->empty()){


        std::string pcd_path = base_path + "/PCD/part_" + std::to_string(part_count) + ".pcd";
        std::cout << "Saving point cloud to " << pcd_path << std::endl;

        std::cout << "Point cloud size: " << dense_pcl->size() << std::endl;

        

        // Save the point cloud to a file
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(pcd_path, *dense_pcl);
        dense_pcl->clear();
        std::cout << "Saved the part point cloud to " << pcd_path << std::endl;
        // *diff_dense_cloud += *cloud;
        part_count++;
    }
    
    save_pcd_by_parts = true;
    res.success = true;
    res.message = "Break pcd on service call";
    return true;
}

void mySIGhandler(int sig){
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
    save_pcd();
    // ros::shutdown();
   
}

void load_config(ros::NodeHandle* nh_ptr, fast_limo::Config* config){

    nh_ptr->param<std::string>("base_path", base_path, ROOT_DIR);

    nh_ptr->param<std::string>("topics/input/lidar", config->topics.lidar,  "/velodyne_points");
    nh_ptr->param<std::string>("topics/input/imu",   config->topics.imu,    "/EL/Sensors/vectornav/IMU");
    nh_ptr->param<std::string>("topics/input/env", config->topics.env_topic,  "/env");

    std::cout << "env topic : " << config->topics.env_topic << endl;

    nh_ptr->param<int>("num_threads", config->num_threads, 10);
    nh_ptr->param<int>("sensor_type", config->sensor_type, 1);

    nh_ptr->param<bool>("debug",    config->debug,      true);
    nh_ptr->param<bool>("verbose",  config->verbose,    true);

    nh_ptr->param<bool>("estimate_extrinsics",  config->ikfom.estimate_extrinsics,  true);
    nh_ptr->param<bool>("time_offset",          config->time_offset,                true);
    nh_ptr->param<bool>("end_of_sweep",         config->end_of_sweep,               false);

    nh_ptr->param<bool>("calibration/gravity_align", config->gravity_align,     true);
    nh_ptr->param<bool>("calibration/accel",         config->calibrate_accel,   true);
    nh_ptr->param<bool>("calibration/gyro",          config->calibrate_gyro,    true);
    nh_ptr->param<double>("calibration/time",        config->imu_calib_time,    3.0);

    nh_ptr->param<std::vector<float>>("extrinsics/imu/t",   config->extrinsics.imu2baselink_t,      {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("extrinsics/imu/R",   config->extrinsics.imu2baselink_R,      std::vector<float> (9, 0.0));
    nh_ptr->param<std::vector<float>>("extrinsics/lidar/t", config->extrinsics.lidar2baselink_t,    {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("extrinsics/lidar/R", config->extrinsics.lidar2baselink_R,    std::vector<float> (9, 0.0));

    nh_ptr->param<std::vector<float>>("intrinsics/accel/bias",  config->intrinsics.accel_bias,  {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("intrinsics/gyro/bias",   config->intrinsics.gyro_bias,   {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("intrinsics/accel/sm",    config->intrinsics.imu_sm,      std::vector<float> (9, 0.0));

    nh_ptr->param<bool>("filters/cropBox/active",                config->filters.crop_active,   true);
    nh_ptr->param<std::vector<float>>("filters/cropBox/box/min", config->filters.cropBoxMin,    {-1.0, -1.0, -1.0});
    nh_ptr->param<std::vector<float>>("filters/cropBox/box/max", config->filters.cropBoxMax,    {1.0, 1.0, 1.0});

    nh_ptr->param<bool>("filters/voxelGrid/active",                 config->filters.voxel_active,   true);
    nh_ptr->param<std::vector<float>>("filters/voxelGrid/leafSize", config->filters.leafSize,       {0.25, 0.25, 0.25});
    nh_ptr->param<std::vector<float>>("filters/voxelGrid/small_room_leafSize", config->filters.small_room_leafSize,       {0.1, 0.1, 0.1});
    nh_ptr->param<std::vector<float>>("filters/voxelGrid/medium_room_leafSize", config->filters.medium_room_leafSize,       {0.3, 0.3, 0.3});

    //print the filter values
    std::cout << "large area leafSize : " << config->filters.leafSize[0] << endl;
    std::cout << "medium area leafSize : " << config->filters.medium_room_leafSize[0] << endl;
    std::cout << "small area leafSize : " << config->filters.small_room_leafSize[0] << endl;
    
    //button trigger
    nh_ptr->param<bool>("filters/voxelGrid/button_trigger",  config->button_trigger,   true);
    std::cout << "button trigger enabled: " <<  config->button_trigger << endl;

    nh_ptr->param<bool>("filters/minDistance/active",   config->filters.dist_active,    false);
    nh_ptr->param<double>("filters/minDistance/value",  config->filters.min_dist,       4.0);

    nh_ptr->param<bool>("filters/rateSampling/active",  config->filters.rate_active,    false);
    nh_ptr->param<int>("filters/rateSampling/value",    config->filters.rate_value,     4);

    float fov_deg;
    nh_ptr->param<bool>("filters/FoV/active",  config->filters.fov_active,  false);
    nh_ptr->param<float>("filters/FoV/value",  fov_deg,                     360.0f);
    config->filters.fov_angle = fov_deg *M_PI/360.0; // half of FoV (bc. is divided by the x-axis)

    nh_ptr->param<int>("iKFoM/Mapping/NUM_MATCH_POINTS",    config->ikfom.mapping.NUM_MATCH_POINTS, 5);
    nh_ptr->param<int>("iKFoM/MAX_NUM_MATCHES",             config->ikfom.mapping.MAX_NUM_MATCHES,  2000);
    nh_ptr->param<int>("iKFoM/MAX_NUM_PC2MATCH",            config->ikfom.mapping.MAX_NUM_PC2MATCH, 1.e+4);
    nh_ptr->param<double>("iKFoM/Mapping/MAX_DIST_PLANE",   config->ikfom.mapping.MAX_DIST_PLANE,   2.0);
    nh_ptr->param<double>("iKFoM/Mapping/PLANES_THRESHOLD", config->ikfom.mapping.PLANE_THRESHOLD,  5.e-2);
    nh_ptr->param<bool>("iKFoM/Mapping/LocalMapping",       config->ikfom.mapping.local_mapping,    false);

    nh_ptr->param<bool>("iKFoM/Mapping/change_planar_threshold", config->ikfom.mapping.change_planar_threshold, false);
    nh_ptr->param<double>("iKFoM/Mapping/small_room_planar_threshold", config->ikfom.mapping.small_room_planar_threshold, 1.0e-2);
    nh_ptr->param<double>("iKFoM/Mapping/medium_room_planar_threshold", config->ikfom.mapping.medium_room_planar_threshold, 5.0e-2);
    nh_ptr->param<bool>("iKFoM/Mapping/dynamic_mapping", config->ikfom.mapping.dynamic_mapping, false);

    nh_ptr->param<float>("iKFoM/iKDTree/balance",   config->ikfom.mapping.ikdtree.balance_param,    0.6f);
    nh_ptr->param<float>("iKFoM/iKDTree/delete",    config->ikfom.mapping.ikdtree.delete_param,     0.3f);
    nh_ptr->param<float>("iKFoM/iKDTree/voxel",     config->ikfom.mapping.ikdtree.voxel_size,       0.2f);
    nh_ptr->param<double>("iKFoM/iKDTree/bb_size",  config->ikfom.mapping.ikdtree.cube_size,        300.0);
    nh_ptr->param<double>("iKFoM/iKDTree/bb_range", config->ikfom.mapping.ikdtree.rm_range,         200.0);

    nh_ptr->param<bool>("iKFoM/iKDTree/dynamic_bb", config->ikfom.mapping.ikdtree.dynamic_bb, false);
    nh_ptr->param<double>("iKFoM/iKDTree/small/bb_size", config->ikfom.mapping.ikdtree.small.bb_size, 100.0);
    nh_ptr->param<double>("iKFoM/iKDTree/small/bb_range", config->ikfom.mapping.ikdtree.small.bb_range, 20.0);
    nh_ptr->param<double>("iKFoM/iKDTree/medium/bb_size", config->ikfom.mapping.ikdtree.medium.bb_size, 200.0);
    nh_ptr->param<double>("iKFoM/iKDTree/medium/bb_range", config->ikfom.mapping.ikdtree.medium.bb_range, 40.0);



    nh_ptr->param<int>("iKFoM/MAX_NUM_ITERS",            config->ikfom.MAX_NUM_ITERS,   3);
    nh_ptr->param<double>("iKFoM/covariance/gyro",       config->ikfom.cov_gyro,        6.e-4);
    nh_ptr->param<double>("iKFoM/covariance/accel",      config->ikfom.cov_acc,         1.e-2);
    nh_ptr->param<double>("iKFoM/covariance/bias_gyro",  config->ikfom.cov_bias_gyro,   1.e-5);
    nh_ptr->param<double>("iKFoM/covariance/bias_accel", config->ikfom.cov_bias_acc,    3.e-4);

    nh_ptr->param<bool>("iKFoM/covariance/change_according_to_env", config->ikfom.change_according_to_env, false);
    nh_ptr->param<double>("iKFoM/covariance/small_room_covariance/gyro", config->ikfom.small_room_cov_gyro, 0.0001);
    nh_ptr->param<double>("iKFoM/covariance/small_room_covariance/accel", config->ikfom.small_room_cov_acc, 0.01);
    nh_ptr->param<double>("iKFoM/covariance/small_room_covariance/bias_gyro", config->ikfom.small_room_cov_bias_gyro, 0.000001);
    nh_ptr->param<double>("iKFoM/covariance/small_room_covariance/bias_accel", config->ikfom.small_room_cov_bias_acc, 0.0001);

    nh_ptr->param<double>("iKFoM/covariance/medium_room_covariance/gyro", config->ikfom.medium_room_cov_gyro, 0.0001);
    nh_ptr->param<double>("iKFoM/covariance/medium_room_covariance/accel", config->ikfom.medium_room_cov_acc, 0.01);
    nh_ptr->param<double>("iKFoM/covariance/medium_room_covariance/bias_gyro", config->ikfom.medium_room_cov_bias_gyro, 0.000001);
    nh_ptr->param<double>("iKFoM/covariance/medium_room_covariance/bias_accel", config->ikfom.medium_room_cov_bias_acc, 0.0001);

    nh_ptr->param<bool>("iKFoM/esekf/active", config->esekf.active, true);
    nh_ptr->param<bool>("iKFoM/esekf/change_according_to_env", config->esekf.change_according_to_env, false);
    nh_ptr->param<double>("iKFoM/esekf/default/measurement_noise", config->esekf.measurement_noise, 0.001);  
    nh_ptr->param<double>("iKFoM/esekf/default/degeneracy_threshold", config->esekf.degeneracy_threshold, 5.0);  
    nh_ptr->param<double>("iKFoM/esekf/small_room/measurement_noise", config->esekf.small_room_measurement_noise, 0.0001);  
    nh_ptr->param<double>("iKFoM/esekf/small_room/degeneracy_threshold", config->esekf.small_room_degeneracy_threshold, 0.0001);  
    nh_ptr->param<double>("iKFoM/esekf/medium_room/measurement_noise", config->esekf.medium_room_measurement_noise, 0.0001);  
    nh_ptr->param<double>("iKFoM/esekf/medium_room/degeneracy_threshold", config->esekf.medium_room_degeneracy_threshold, 0.0001); 
    nh_ptr->param<bool>("iKFoM/esekf/print_degeneracy_values", config->esekf.print_degeneracy_values, false);


    nh_ptr->param<bool>("iKFoM/skf/active", config->skf.active, false);
    nh_ptr->param<double>("iKFoM/skf/measurement_noise", config->skf.measurement_noise, 0.0001);
    nh_ptr->param<double>("iKFoM/skf/degeneracy_threshold", config->skf.degeneracy_threshold, 0.0001);


    nh_ptr->param<bool>("save/dense_pcd", config->save_dense_pcd, false);
    nh_ptr->param<bool>("save/skewed_pcd", config->save_skewed_pcd, false);
    nh_ptr->param<bool>("save/break_pcd_on_service_call", config->save_pcd_by_parts, false);

    nh_ptr->param<bool>("offline_mode", config->offline_mode, false);

    nh_ptr->param<std::string>("scan_path", config->data_path, "");

    double ikfom_limits;
    nh_ptr->param<double>("iKFoM/LIMITS", ikfom_limits, 1.e-3);
    config->ikfom.LIMITS = std::vector<double> (23, ikfom_limits);

}

void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Localizer and Mapper objects
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    // Setup config parameters
    fast_limo::Config config;
    load_config(&nh, &config);

    // Read frames names
    nh.param<std::string>("frames/world", world_frame, "map");
    nh.param<std::string>("frames/body", body_frame, "base_link");


    // Define services
    ros::ServiceServer set_leaf_size_service = nh.advertiseService("set_leaf_size", set_leaf_size_callback);

    ros::ServiceServer save_pcd_service = nh.advertiseService("break_pcd", break_pcd_on_service_callback);

    pc_pub      = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    state_pub   = nh.advertise<nav_msgs::Odometry>("state", 1);

        // debug
    orig_pub     = nh.advertise<sensor_msgs::PointCloud2>("original", 1);
    desk_pub     = nh.advertise<sensor_msgs::PointCloud2>("deskewed", 1);
    match_pub    = nh.advertise<sensor_msgs::PointCloud2>("match", 1);
    finalraw_pub = nh.advertise<sensor_msgs::PointCloud2>("final_raw", 1);
    body_pub     = nh.advertise<nav_msgs::Odometry>("body_state", 1);
    map_bb_pub   = nh.advertise<visualization_msgs::Marker>("map/bb", 1);
    match_points_pub = nh.advertise<visualization_msgs::MarkerArray>("match_points", 1);
    

    // Set up fast_limo config
    loc.init(config);

    if (config.data_path == "") {
        ROS_ERROR("bag files location wrong");
        ros::shutdown;
        return EXIT_SUCCESS;
    }

    if (config.offline_mode){

        std::vector<std::string> topics_;
        topics_.push_back(config.topics.lidar);
        topics_.push_back(config.topics.imu);
        topics_.push_back(config.topics.env_topic);


        rosbag::TopicQuery topics(topics_);

        std::vector<std::shared_ptr<rosbag::Bag>> bags;

        if (boost::filesystem::exists(config.data_path) && boost::filesystem::is_directory(config.data_path)) {
            std::vector<boost::filesystem::path> bag_files;

            // Collect all bag files
            for (const auto& file : boost::filesystem::directory_iterator(config.data_path)) {
                if (file.path().extension() == ".bag") {
                    bag_files.push_back(file.path());
                }
            }

            // Sort the files alphabetically
            std::sort(bag_files.begin(), bag_files.end());

            // Open the bags in sorted order
            for (const auto& file_path : bag_files) {
                std::cout << "Reading bag " << file_path.string() << std::endl;
                std::shared_ptr<rosbag::Bag> bag = std::make_shared<rosbag::Bag>();
                bag->open(file_path.string());
                bags.push_back(bag);
            }
        } else {
            ROS_ERROR("data path does not exist or is not a directory");
            ros::shutdown();
            return EXIT_SUCCESS;
        }

        std::cout << "read " << bags.size() << " bags" << std::endl;

        rosbag::View full_view;
        BOOST_FOREACH (std::shared_ptr<rosbag::Bag> bag, bags) {
            full_view.addQuery(*bag);
        }
        // for (const auto& bag : bags) {
        //     full_view.addQuery(*bag);
        // }

        ros::Time initial_time = full_view.getBeginTime();

        std::cout << "initial_time: " << initial_time.toSec() << std::endl;

        rosbag::View view;
        BOOST_FOREACH (std::shared_ptr<rosbag::Bag> bag, bags) {
            view.addQuery(*bag, topics, initial_time, ros::TIME_MAX);

        }
        // for (const auto& bag : bags) {
        //     view.addQuery(*bag, topics, initial_time, ros::TIME_MAX);

        // }
        // view.sort();
        // view.sortByTimestamp();
        std::cout << "view size: " << view.size() << std::endl;
        std::cout << "topics: ";
        for (const auto& topic : topics_) {
            std::cout << topic << " ";
        }
        std::cout << std::endl;

        // // Start spinning (async)
        // ros::AsyncSpinner spinner(0);
        // spinner.start();

        ros::Publisher imu_pub, pc2_pub, env_pub;

        // Inside main function, initialize these publishers
        imu_pub = nh.advertise<sensor_msgs::Imu>(config.topics.imu, 1);
        pc2_pub = nh.advertise<sensor_msgs::PointCloud2>(config.topics.lidar, 1);
        env_pub = nh.advertise<std_msgs::String>(config.topics.env_topic, 1);

        // signal(SIGINT, SigHandle);

        

        ros::Rate rate(10);

        std::cout << "starting reading messages from bag..." << std::endl;

        try {
            BOOST_FOREACH (rosbag::MessageInstance const m,  view) {
                if (flg_exit || loc.scan_finished) {
                    break;
                }

                if (save_pcd_by_parts){
                    save_pcd_by_parts_callback();
                }
                
                //print the message type
                // std::cout << "Message type: " << m.getTopic() << std::endl;
                // std::cout << "Message timestamp: " << m.getTime() << std::endl;
                // std::cout << "Message size: " << m.size() << std::endl;
                auto imu_msg = m.instantiate<sensor_msgs::Imu>();
                if (imu_msg) {
                    // std::cout << "Processing IMU message." << std::endl;
                    imu_callback(imu_msg);
                    imu_pub.publish(imu_msg);
                    if(!loc.get_sync_status()){
                        break;
                    }
                    continue;
                }

                auto pc2_msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (pc2_msg) {
                    // std::cout << "Processing PointCloud2 message." << std::endl;
                    lidar_callback(pc2_msg);
                    pc2_pub.publish(pc2_msg);
                    if(!loc.get_sync_status()){
                        break;
                    }
                    continue;
                }

                auto env_msg = m.instantiate<std_msgs::String>();
                if (env_msg) {
                    // std::cout << "Processing Environment message." << std::endl;
                    env_callback(env_msg);
                    env_pub.publish(env_msg);
                    if(!loc.get_sync_status()){
                        break;
                    }
                    continue;
                }
                rate.sleep();
                ros::spinOnce(); 
            }
            // ros::spinOnce(); 

        } catch (const std::exception& e) {
            std::cerr << "Exception caught during message processing: " << e.what() << std::endl;
            ros::shutdown();
            return EXIT_FAILURE;
        }
        std::cout << "Scan finished successfully! Cleanup started :)" << endl;


        BOOST_FOREACH (std::shared_ptr<rosbag::Bag> bag, bags) {
            bag->close();
        }
    
        // save_pcd();
        //call the signal handler
        mySIGhandler(0);
    }
    else{
        // Define subscribers & publishers
        ros::Subscriber lidar_sub = nh.subscribe(config.topics.lidar, 1, &lidar_callback, ros::TransportHints().tcpNoDelay());
        ros::Subscriber imu_sub   = nh.subscribe(config.topics.imu, 1000, &imu_callback, ros::TransportHints().tcpNoDelay());
        ros::Subscriber env_sub   = nh.subscribe(config.topics.env_topic, 1000, &env_callback, ros::TransportHints().tcpNoDelay());

        // Start spinning (async)
        ros::AsyncSpinner spinner(0);
        spinner.start();

        ros::waitForShutdown();
    }
    // ros::shutdown();

    // ros::waitForShutdown();

    return 0;

}