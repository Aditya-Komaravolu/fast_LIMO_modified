/*
 Copyright (c) 2024 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __FASTLIMO_CONFIG_HPP__
#define __FASTLIMO_CONFIG_HPP__

#include "fast_limo/Common.hpp"

struct fast_limo::Config{

    struct Topics{
        std::string lidar;
        std::string imu;
        std::string env_topic;
    } topics;

    struct Extrinsics{
        std::vector<float> imu2baselink_t;
        std::vector<float> imu2baselink_R;
        std::vector<float> lidar2baselink_t;
        std::vector<float> lidar2baselink_R;
    } extrinsics;

    struct Intrinsics{
        std::vector<float> accel_bias;
        std::vector<float> gyro_bias;
        std::vector<float> imu_sm;
    } intrinsics;

    struct Filters{
        std::vector<float> cropBoxMin;  // crop filter
        std::vector<float> cropBoxMax;  // crop filter
        bool crop_active;               // crop filter
        std::vector<float> leafSize;    // voxel grid filter
        std::vector<float> small_room_leafSize;
        std::vector<float> medium_room_leafSize;
        bool voxel_active;              // voxel grid filter
        double min_dist;                // norm/dist filter
        bool dist_active;               // norm/dist filter
        int rate_value;                 // time rate filter
        bool rate_active;               // time rate filter
        float fov_angle;                // FoV filter
        bool fov_active;                // FoV filter
        struct filter_points{
            bool active;
            float max_filter_distance;
        } filter_points;
    } filters;

    struct iKFoM{
        struct Mapping{
            int NUM_MATCH_POINTS;   // num of points that constitute a match
            int MAX_NUM_MATCHES;    // max num of matches (helps to reduce comp. load)
            int MAX_NUM_PC2MATCH;   // max num of points to match (helps to reduce comp. load)
            int *k_found_matches;   // max num of points found to match (helps to reduce comp. load)
            double MAX_DIST_PLANE;  // max distance between points to be considered a plane
            double PLANE_THRESHOLD; // threshold to consider an estimated plane is actually a plane (also used for deciding if point belongs to plane )
            bool change_planar_threshold;
            double small_room_planar_threshold;
            double medium_room_planar_threshold;
            bool local_mapping;     // whether to move the map with the robot's pose (fixed size map) or not (increasing size, limitless map) 
            bool dynamic_mapping;   // FLAG:: whether to change the local mapping based on the leaf size
            struct iKDTree{
                float delete_param;
                float balance_param;
                float voxel_size;
                double cube_size;
                double rm_range;
                bool dynamic_bb;
                struct small{
                    double bb_size;
                    double bb_range;
                } small;
                struct medium{
                    double bb_size;
                    double bb_range;
                } medium;
            } ikdtree;
        } mapping;

        int MAX_NUM_ITERS;          // max num of iterations of the extended KF
        std::vector<double> LIMITS;
        bool estimate_extrinsics;   // whether to estimate extrinsics or assume fixed
        double cov_gyro;            // covariance ang. velocity
        double cov_acc;             // covariance lin. accel.
        double cov_bias_gyro;       // covariance bias ang. vel.
        double cov_bias_acc;        // covariance bias lin. accel.
        bool change_according_to_env;
        double small_room_cov_gyro;
        double small_room_cov_acc;
        double small_room_cov_bias_gyro;
        double small_room_cov_bias_acc;
        double medium_room_cov_gyro;
        double medium_room_cov_acc;
        double medium_room_cov_bias_gyro;
        double medium_room_cov_bias_acc;
    } ikfom;

    struct SelectiveKF{
        bool active;
        double measurement_noise;
        double degeneracy_threshold;
    } skf;

    struct ESEKFOM{
        bool active;
        double measurement_noise;
        double degeneracy_threshold;
        bool print_degeneracy_values;
        bool change_according_to_env;
        double small_room_measurement_noise;
        double small_room_degeneracy_threshold;
        double medium_room_measurement_noise;
        double medium_room_degeneracy_threshold;
        bool original_method;
        bool selective_method;
    } esekf;

    // Flags
    bool gravity_align;         // whether to estimate gravity vector
    bool calibrate_accel;       // whether to estimate linear accel. bias
    bool calibrate_gyro;        // whether to estimate ang. velocity bias
    bool time_offset;           // whether to take into account the time offset
    bool end_of_sweep;          // whether the sweep reference time is w.r.t. the start or the end of the scan (only applies to VELODYNE/OUSTER)

    bool debug;         // whether to copy intermediate point clouds into aux variables (for visualization)
    bool verbose;       // whether to print debugging/performance board

    bool button_trigger;      // whether to trigger the button

    bool save_dense_pcd;
    bool save_skewed_pcd;
    bool save_pcd_by_parts;
    bool new_esekf;
    std::string data_path;
    bool offline_mode;
    // Other
    int sensor_type;        // LiDAR type
    int num_threads;        // num of threads to be used by OpenMP
    double imu_calib_time;  // time to be estimating IMU biases

    struct {
        bool active = false;
        float distance_threshold = 10.0;
        float scan_match_threshold = 0.3;
        int keyframe_interval = 20;
        int nearby_frames_to_skip = 100;
        float loop_weight = 100.0;
        float odometry_weight = 1.0;
        bool use_floor_constraints = true;
        bool use_imu_constraints = true;
        float floor_plane_weight = 100.0;
        float imu_orientation_weight = 100.0;
        // Floor detection parameters
        float floor_max_angle = 0.1;        // ~5.7 degrees from horizontal
        float floor_max_height = 0.2;       // 20cm max height difference
        int floor_min_points = 100;         // Minimum points to detect a floor
        float floor_normal_thresh = 0.8;    // cos(angle) threshold for normal
        // Scan context parameters
        float scan_context_threshold = 0.15;  // Threshold for descriptor matching
        int scan_context_rings = 20;          // Number of rings in scan context
        int scan_context_sectors = 60;        // Number of sectors in scan context
        float scan_context_max_radius = 80.0; // Maximum radius for scan context (meters)
        float scan_context_max_height = 2.0;  // Maximum height for scan context (meters)
        // ICP parameters
        int icp_max_iterations = 100;
        float icp_transformation_epsilon = 1e-5;
        float icp_euclidean_fitness_epsilon = 1e-4;
        float icp_max_correspondence_distance = 2.0;
        // SCIA parameters
        bool use_coarse_alignment = true;
        float coarse_alignment_max_correspondence_distance = 5.0;
        float coarse_alignment_min_sample_distance = 1.0;
        int coarse_alignment_max_iterations = 50;
        int coarse_alignment_correspondence_randomness = 5;
        int coarse_alignment_number_of_samples = 3;
    } loop_closure;
};

#endif