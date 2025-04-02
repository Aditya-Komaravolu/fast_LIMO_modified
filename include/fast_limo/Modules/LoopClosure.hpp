#ifndef LOOP_CLOSURE_HPP
#define LOOP_CLOSURE_HPP

#include <fast_limo/Common.hpp>
#include <fast_limo/Objects/State.hpp>
#include <fast_limo/Utils/Config.hpp>
#include <fast_limo/Modules/ScanContext.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fast_limo {

struct KeyFrame {
    int id;
    double timestamp;
    State pose;
    pcl::PointCloud<PointType>::Ptr cloud;
    
    // Add floor plane info
    bool has_floor;
    Eigen::Vector4f floor_plane;
    
    // Add scan context descriptor
    cv::Mat scan_context;
    bool has_descriptor = false;
    
    KeyFrame(int _id, double _timestamp, const State& _pose, pcl::PointCloud<PointType>::Ptr _cloud)
        : id(_id), timestamp(_timestamp), pose(_pose), cloud(_cloud), has_floor(false) {}
};

// Floor plane constraint
struct FloorPlaneConstraint {
    int keyframe_id;
    Eigen::Vector4f floor_coeffs;  // ax + by + cz + d = 0
    
    FloorPlaneConstraint(int id, const Eigen::Vector4f& coeffs)
        : keyframe_id(id), floor_coeffs(coeffs) {}
};

// IMU constraint
struct IMUConstraint {
    int keyframe_id;
    Eigen::Vector3f gravity_dir;  // Direction of gravity in world frame
    
    IMUConstraint(int id, const Eigen::Vector3f& gravity)
        : keyframe_id(id), gravity_dir(gravity) {}
};

class LoopClosure {
public:
    LoopClosure();
    ~LoopClosure();
    
    void init(Config& cfg);
    
    // Add a new keyframe for loop closure detection
    void addKeyFrame(const State& pose, pcl::PointCloud<PointType>::Ptr cloud);
    
    // Detect loop closure and optimize trajectory
    bool detectAndOptimize();
    
    // Get the corrected poses and transformations
    std::vector<State> getCorrectedPoses();
    Eigen::Matrix4f getCorrection(double timestamp) const;
    
    // Apply the correction to a point cloud
    pcl::PointCloud<PointType>::Ptr correctPointCloud(pcl::PointCloud<PointType>::Ptr cloud, double timestamp);
    
    // Get the current optimized map
    pcl::PointCloud<PointType>::Ptr getOptimizedMap();
    
    // Getters for debug info
    bool wasLoopDetected() const { return loop_detected_; }
    int getTotalLoopClosures() const { return total_loop_closures_; }
    double getLatestLoopError() const { return latest_loop_error_; }
    double getLatestCorrectionMagnitude() const { return latest_correction_magnitude_; }
    int getKeyframesCount() const { return keyframes_.size(); }
    int getFloorPlanesCount() const { return floor_constraints_.size(); }
    
    // Add IMU constraint
    void addIMUConstraint(const KeyFrame& keyframe);
    
    // Add this method to get floor constraints
    std::vector<FloorPlaneConstraint> getFloorConstraints() const {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        return floor_constraints_;
    }
    
private:
    // Loop detection parameters
    float distance_threshold_;      // Minimum distance for loop closure
    float scan_match_threshold_;    // ICP fitness score threshold
    int keyframe_interval_;         // Number of frames to skip between keyframes
    int nearby_frames_to_skip_;     // Number of recent frames to skip in loop detection
    float scan_context_threshold_;  // Threshold for scan context matching
    
    // Pose graph optimization parameters
    float loop_closure_weight_;
    float odometry_weight_;
    float floor_plane_weight_;
    float imu_orientation_weight_;
    
    // Floor plane detection parameters
    float floor_max_angle_;         // Maximum angle in radians between floor and XY plane
    float floor_max_height_;        // Maximum height difference for a floor plane
    int floor_min_points_;          // Minimum points to detect a floor plane
    float floor_normal_thresh_;     // Normal threshold for floor plane
    bool use_floor_constraints_;    // Whether to use floor plane constraints
    
    // IMU constraints parameters
    bool use_imu_constraints_;      // Whether to use IMU gravity constraints
    
    // Data storage
    std::vector<KeyFrame> keyframes_;
    std::vector<State> original_poses_;
    std::vector<State> corrected_poses_;
    std::vector<FloorPlaneConstraint> floor_constraints_;
    std::vector<IMUConstraint> imu_constraints_;
    mutable std::mutex keyframes_mutex_;
    
    // Scan context for loop detection
    std::shared_ptr<ScanContext> scan_context_;
    
    // Latest point cloud
    pcl::PointCloud<PointType>::Ptr optimized_map_;
    
    // ICP for scan matching
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;
    
    // Helper methods
    bool detectLoopClosure(const KeyFrame& current_frame, KeyFrame& matched_frame);
    bool optimizePoseGraph(const std::vector<std::pair<int, int>>& loop_closures,
                         const std::vector<Eigen::Matrix4f>& loop_constraints);
    Eigen::Matrix4f poseToMatrix(const State& pose) const;
    State matrixToPose(const Eigen::Matrix4f& matrix, double timestamp);
    pcl::PointCloud<PointType>::Ptr downsampleCloud(pcl::PointCloud<PointType>::Ptr cloud, float leaf_size);
    
    // Floor plane detection
    bool detectFloorPlane(KeyFrame& keyframe);
    
    // Pose graph
    g2o::SparseOptimizer optimizer_;
    
    // Config
    Config config_;
    int frame_count_;
    bool loop_detected_;
    int total_loop_closures_;
    double latest_loop_error_;
    double latest_correction_magnitude_;
    
    pcl::PointCloud<PointType>::Ptr preprocessCloud(pcl::PointCloud<PointType>::Ptr cloud);
    void visualizeICPAlignment(const pcl::PointCloud<PointType>::Ptr& source,
                              const pcl::PointCloud<PointType>::Ptr& target,
                              const pcl::PointCloud<PointType>::Ptr& aligned,
                              const std::string& frame_id);
};

} // namespace fast_limo

#endif // LOOP_CLOSURE_HPP 