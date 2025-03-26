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

#ifndef __FASTLIMO_LOCALIZER_HPP__
#define __FASTLIMO_LOCALIZER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Mapper.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/Plane.hpp"
#include "fast_limo/Utils/Config.hpp"
#include "fast_limo/Utils/Algorithms.hpp"
#include "fast_limo/Utils/FrameDumper.hpp"
#include <std_msgs/Header.h>  
#include <std_msgs/String.h>  
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace fast_limo;

namespace thresholds{
    template <class ContainerAllocator>
    struct mapping_tweak_values_ { 
    typedef mapping_tweak_values_<ContainerAllocator> Type;


    mapping_tweak_values_()
    : header()
    , env("")
    , leafSize({0.0,0.0,0.0})
    , localmapping(true)
    , ikdtree_bb_size(0.0)
    , ikdtree_bb_range(0.0)
    , planar_threshold(0.0)
    , cov_gyro(0.0)
    , cov_acc(0.0)
    , cov_bias_gyro(0.0)
    , cov_bias_acc(0.0)
    , esekf_degeneracy_threshold(0.0)
    , esekf_measurement_noise(0.0)
    {
    
    }

    mapping_tweak_values_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , env("")
    , leafSize({0.0,0.0,0.0})
    , localmapping(true)
    , ikdtree_bb_size(0.0)
    , ikdtree_bb_range(0.0)
    , planar_threshold(0.0)
    , cov_gyro(0.0)
    , cov_acc(0.0)
    , cov_bias_gyro(0.0)
    , cov_bias_acc(0.0)
    , esekf_degeneracy_threshold(0.0)
    , esekf_measurement_noise(0.0)
    {
    (void)_alloc;
    }


    typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
    _header_type header;


    typedef std::vector<float> leaf_size;
    leaf_size leafSize;

    typedef bool local_mapping;
    local_mapping localmapping;


    typedef double bb_size;
    bb_size ikdtree_bb_size;

    typedef double bb_range;
    bb_range ikdtree_bb_range;

    typedef double planarThreshold;
    planarThreshold planar_threshold;

    typedef double covGyro;
    covGyro cov_gyro;

    typedef double covAcc;
    covAcc cov_acc;

    typedef double covBiasGyro;
    covBiasGyro cov_bias_gyro;

    typedef double covBiasAcc;
    covBiasAcc cov_bias_acc;

    typedef double degeneracy_threshold;
    degeneracy_threshold esekf_degeneracy_threshold;

    typedef double measurement_noise;
    measurement_noise esekf_measurement_noise;

    typedef std::string env_name;
    env_name env;



    typedef boost::shared_ptr<thresholds::mapping_tweak_values_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<thresholds::mapping_tweak_values_<ContainerAllocator> const> ConstPtr;

    };

    typedef thresholds::mapping_tweak_values_<std::allocator<void> > mapping_tweak_values;

    typedef boost::shared_ptr<thresholds::mapping_tweak_values > mapping_tweak_valuesPtr;
    typedef boost::shared_ptr<thresholds::mapping_tweak_values const> mapping_tweak_valuesConstPtr;


}


class fast_limo::Localizer {

    // VARIABLES

    public:
        pcl::PointCloud<PointType>::ConstPtr pc2match; // pointcloud to match in Xt2 (last_state) frame

        // Config struct
        Config config;
        double last_timestamp_imu;
        double last_timestamp_lidar;
        double last_imu_processed_time;
        bool scan_finished;

        std::vector<float> default_leaf_size;
        double default_bb_size;
        double default_bb_range;
        double default_planar_threshold;

        double cov_gyro_default;
        double cov_acc_default;
        double cov_bias_gyro_default;
        double cov_bias_acc_default;

        double esekf_measurement_noise;
        double esekf_degeneracy_threshold;
        bool print_degeneracy_values;

        double default_esekf_measurement_noise;
        double default_esekf_degeneracy_threshold;
        Eigen::Matrix<double, 6, 1> curr_state_cov_eign_values;
        // bool raw_pc_empty;
        // bool imu_buffer_empty;
        // bool imu_msg_empty;
        
    private:
        // Iterated Kalman Filter on Manifolds (FASTLIOv2)
        esekfom::esekf<state_ikfom, 12, input_ikfom> _iKFoM;
        std::mutex mtx_ikfom;

        State state, last_state;
        Extrinsics extr;
        SensorType sensor;
        IMUmeas last_imu;

        bool sync_status;



        // Matches (debug aux var.)
        Matches matches;

        // PCL Filters
        pcl::CropBox<PointType> crop_filter;
        pcl::VoxelGrid<PointType> voxel_filter;

        // Point Clouds
        pcl::PointCloud<PointType>::ConstPtr original_scan; // in base_link/body frame
        pcl::PointCloud<PointType>::ConstPtr deskewed_scan; // in global/world frame
        pcl::PointCloud<PointType>::Ptr final_raw_scan;     // in global/world frame
        pcl::PointCloud<PointType>::Ptr final_scan;         // in global/world frame
        pcl::PointCloud<PointType>::Ptr accumulated_cloud;
        pcl::PointCloud<PointType>::Ptr accumulated_downsampled_cloud;

        // Time related var.
        double scan_stamp;
        double prev_scan_stamp;
        double scan_dt;

        double imu_stamp;
        double prev_imu_stamp;
        double imu_dt;
        double first_imu_stamp;
        double last_propagate_time_;
        double imu_calib_time_;

        // Gravity
        double gravity_;

        // Flags
        bool imu_calibrated_ = false;

        // OpenMP max threads
        int num_threads_;

        // IMU buffer
        // boost::circular_buffer<IMUmeas> imu_buffer;
        std::deque<IMUmeas> imu_buffer;

        // boost::circular_buffer<std::string> env_buffer;

        std::deque<thresholds::mapping_tweak_values::ConstPtr> env_buffer;
        thresholds::mapping_tweak_values thres_ptr;

        bool button_trigger;
        std::string global_env_state;

        // std::deque<sensor_msgs::PointCloud2> lidar_buffer;
        // std::deque<sensor_msgs::Imu> imu_buffer;
        // std::deque<std_msgs::String> env_buffer;

        // Propagated states buffer
        // boost::circular_buffer<State> propagated_buffer;
        std::deque<State> propagated_buffer;
        std::mutex mtx_prop; // mutex for avoiding multiple thread access to the buffer
        std::condition_variable cv_prop_stamp;


        // IMU axis matrix 
        Eigen::Matrix3f imu_accel_sm_;
        /*(if your IMU doesn't comply with axis system ISO-8855, 
        this matrix is meant to map its current orientation with respect to the standard axis system)
            Y-pitch
            ^   
            |  
            | 
            |
      Z-yaw o-----------> X-roll
        */

        // Debugging
        unsigned char calibrating = 0;

            // Threads
        std::thread debug_thread;

            // Buffers
        // boost::circular_buffer<double> cpu_times;
        // boost::circular_buffer<double> imu_rates;
        // boost::circular_buffer<double> lidar_rates;
        // boost::circular_buffer<double> cpu_percents;

        std::deque<double> cpu_times;
        std::deque<double> imu_rates;
        std::deque<double> lidar_rates;
        std::deque<double> cpu_percents;

            // CPU specs
        std::string cpu_type;
        clock_t lastCPU, lastSysCPU, lastUserCPU;
        int numProcessors;

            // Other
        chrono::duration<double> elapsed_time;  // pointcloud callback elapsed time
        int deskew_size;                        // steps taken to deskew (FoV discretization)
        int propagated_size;                    // number of integrated states

        std::shared_ptr<FrameDumper> frame_dumper_;
        bool save_frames_ = false; // Set to false if you don't want to save frames by default

        Eigen::Vector3f initial_position_;
        Eigen::Matrix3f initial_rotation_;
        visualization_msgs::Marker ground_plane_marker_;

    // FUNCTIONS

    public:
        Localizer();
        void init(Config& cfg);

        // Callbacks 
        void updateIMU(IMUmeas& raw_imu);
        void updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp);

        // Get output
        pcl::PointCloud<PointType>::Ptr get_pointcloud();
        pcl::PointCloud<PointType>::Ptr get_finalraw_pointcloud();
        pcl::PointCloud<PointType>::Ptr get_accumulated_pointcloud();
        pcl::PointCloud<PointType>::Ptr get_accumulated_downsampled_pointcloud();
        void append_msg_to_env_buffer(const std_msgs::String::ConstPtr& msg);
        bool get_sync_status();
        pcl::PointCloud<PointType>::ConstPtr get_orig_pointcloud();
        pcl::PointCloud<PointType>::ConstPtr get_deskewed_pointcloud();
        pcl::PointCloud<PointType>::ConstPtr get_pc2match_pointcloud();

        Matches& get_matches();

        State getWorldState();  // get state in body/base_link frame
        State getBodyState();   // get state in LiDAR frame

        std::vector<double> getPoseCovariance(); // get eKF covariances
        std::vector<double> getTwistCovariance();// get eKF covariances
        
        double get_propagate_time();

        // Status info
        bool is_calibrated();

        // Config
        void set_sensor_type(uint8_t type);

        // iKFoM measurement model
        void calculate_H(const state_ikfom&, const Matches&, Eigen::MatrixXd& H, Eigen::VectorXd& h);

        // Backpropagation
        void propagateImu(const IMUmeas& imu);
        void propagateImu(double t1, double t2);
        void set_voxel_leaf_size(float leaf_size);
        fast_limo::Config& get_config();

        Eigen::Vector3f getInitialPosition() const { return initial_position_; }
        Eigen::Matrix3f getInitialRotation() const { return initial_rotation_; }

    private:
        void init_iKFoM();
        void init_iKFoM_state();

        IMUmeas imu2baselink(IMUmeas& imu);

        pcl::PointCloud<PointType>::Ptr deskewPointCloud(pcl::PointCloud<PointType>::Ptr& pc, double& start_time);

        States integrateImu(double start_time, double end_time, State& state);

        // bool propagatedFromTimeRange(double start_time, double end_time,
        //                           boost::circular_buffer<State>::reverse_iterator& begin_prop_it,
        //                           boost::circular_buffer<State>::reverse_iterator& end_prop_it);
        // bool imuMeasFromTimeRange(double start_time, double end_time,
        //                           boost::circular_buffer<IMUmeas>::reverse_iterator& begin_imu_it,
        //                           boost::circular_buffer<IMUmeas>::reverse_iterator& end_imu_it);

        bool propagatedFromTimeRange(double start_time, double end_time,
                                  std::deque<State>::reverse_iterator& begin_prop_it,
                                  std::deque<State>::reverse_iterator& end_prop_it);
        bool imuMeasFromTimeRange(double start_time, double end_time,
                                  std::deque<IMUmeas>::reverse_iterator& begin_imu_it,
                                  std::deque<IMUmeas>::reverse_iterator& end_imu_it);
        bool isInRange(PointType& p);


        // Method to update the accumulated point cloud
        // void update_accumulated_pointcloud(pcl::PointCloud<PointType>::Ptr new_cloud);
        void update_accumulated_pointcloud(pcl::PointCloud<PointType>::Ptr new_cloud, pcl::PointCloud<PointType>::Ptr new_downsampled_cloud);


        void getCPUinfo();
        void debugVerbose();

    // SINGLETON 

    public:
        static Localizer& getInstance(){
            static Localizer* loc = new Localizer();
            return *loc;
        }

    private:
        // Disable copy/move functionality
        Localizer(const Localizer&) = delete;
        Localizer& operator=(const Localizer&) = delete;
        Localizer(Localizer&&) = delete;
        Localizer& operator=(Localizer&&) = delete;

};

#endif