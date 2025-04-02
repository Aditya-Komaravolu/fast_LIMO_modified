#include <fast_limo/Modules/LoopClosure.hpp>
#include <fast_limo/Modules/g2o_custom_edges.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <ros/ros.h>
#include <omp.h>
#include <thread>
#include <atomic>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <sensor_msgs/PointCloud2.h>

namespace fast_limo {

LoopClosure::LoopClosure() 
    : distance_threshold_(10.0),
      scan_match_threshold_(0.3),
      keyframe_interval_(20),
      nearby_frames_to_skip_(100),
      scan_context_threshold_(0.15),
      loop_closure_weight_(100.0),
      odometry_weight_(1.0),
      floor_plane_weight_(100.0),
      imu_orientation_weight_(100.0),
      floor_max_angle_(0.1),         // ~5.7 degrees from horizontal
      floor_max_height_(0.2),        // 20cm max height difference
      floor_min_points_(100),        // Minimum points to consider a valid floor
      floor_normal_thresh_(0.8),     // cos(angle) threshold for normal
      use_floor_constraints_(true),
      use_imu_constraints_(true),
      frame_count_(0),
      loop_detected_(false),
      total_loop_closures_(0),
      latest_loop_error_(0.0),
      latest_correction_magnitude_(0.0) {
    
    optimized_map_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    
    // Configure ICP
    gicp_.setMaxCorrespondenceDistance(2.0);
    gicp_.setMaximumIterations(100);
    gicp_.setTransformationEpsilon(1e-5);
    gicp_.setEuclideanFitnessEpsilon(1e-4);
    
    // Initialize scan context with default parameters
    scan_context_ = std::make_shared<ScanContext>(20, 60, 80.0, 2.0);
    
    // Configure G2O optimizer
    auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto blockSolver = std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    
    optimizer_.setAlgorithm(algorithm);
    optimizer_.setVerbose(false);
}

LoopClosure::~LoopClosure() {
}

void LoopClosure::init(Config& cfg) {
    config_ = cfg;
    
    // Load parameters from config
    if (cfg.loop_closure.active) {
        distance_threshold_ = cfg.loop_closure.distance_threshold;
        scan_match_threshold_ = cfg.loop_closure.scan_match_threshold;
        keyframe_interval_ = cfg.loop_closure.keyframe_interval;
        nearby_frames_to_skip_ = cfg.loop_closure.nearby_frames_to_skip;
        loop_closure_weight_ = cfg.loop_closure.loop_weight;
        odometry_weight_ = cfg.loop_closure.odometry_weight;
        
        // Add parameters for floor and IMU constraints
        use_floor_constraints_ = cfg.loop_closure.use_floor_constraints;
        use_imu_constraints_ = cfg.loop_closure.use_imu_constraints;
        floor_plane_weight_ = cfg.loop_closure.floor_plane_weight;
        imu_orientation_weight_ = cfg.loop_closure.imu_orientation_weight;
        
        // Floor detection parameters
        floor_max_angle_ = cfg.loop_closure.floor_max_angle;
        floor_max_height_ = cfg.loop_closure.floor_max_height;
        floor_min_points_ = cfg.loop_closure.floor_min_points;
        floor_normal_thresh_ = cfg.loop_closure.floor_normal_thresh;
        
        // Scan context parameters
        scan_context_threshold_ = cfg.loop_closure.scan_context_threshold;
        
        // Initialize scan context with config parameters
        scan_context_ = std::make_shared<ScanContext>(
            cfg.loop_closure.scan_context_rings,
            cfg.loop_closure.scan_context_sectors,
            cfg.loop_closure.scan_context_max_radius,
            cfg.loop_closure.scan_context_max_height
        );
        
        // ICP parameters
        gicp_.setMaxCorrespondenceDistance(cfg.loop_closure.icp_max_correspondence_distance);
        gicp_.setMaximumIterations(cfg.loop_closure.icp_max_iterations);
        gicp_.setTransformationEpsilon(cfg.loop_closure.icp_transformation_epsilon);
        gicp_.setEuclideanFitnessEpsilon(cfg.loop_closure.icp_euclidean_fitness_epsilon);
    }
}

void LoopClosure::addKeyFrame(const State& pose, pcl::PointCloud<PointType>::Ptr cloud) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    
    // Only add keyframes at specified interval
    if (frame_count_++ % keyframe_interval_ != 0) {
        return;
    }
    
    // Limit maximum number of keyframes to prevent memory issues
    const size_t MAX_KEYFRAMES = 1000;
    if (keyframes_.size() >= MAX_KEYFRAMES) {
        // Evict oldest keyframe
        keyframes_.erase(keyframes_.begin());
        original_poses_.erase(original_poses_.begin());
        corrected_poses_.erase(corrected_poses_.begin());
        
        // Also remove any constraints for the evicted keyframe
        floor_constraints_.erase(
            std::remove_if(floor_constraints_.begin(), floor_constraints_.end(),
                [](const FloorPlaneConstraint& c) { return c.keyframe_id == 0; }),
            floor_constraints_.end());
            
        imu_constraints_.erase(
            std::remove_if(imu_constraints_.begin(), imu_constraints_.end(),
                [](const IMUConstraint& c) { return c.keyframe_id == 0; }),
            imu_constraints_.end());
        
        // Remap IDs to maintain consistency
        for (size_t i = 0; i < keyframes_.size(); i++) {
            keyframes_[i].id = i;
        }
        
        // Update IDs in constraints
        for (auto& c : floor_constraints_) {
            c.keyframe_id--;
        }
        for (auto& c : imu_constraints_) {
            c.keyframe_id--;
        }
    }
    
    // Downsample cloud for storage efficiency
    pcl::PointCloud<PointType>::Ptr downsampled_cloud = downsampleCloud(cloud, 0.2);
    
    // Create a new keyframe
    int id = keyframes_.size();
    KeyFrame keyframe(id, pose.time, pose, downsampled_cloud);
    
    // Detect floor plane
    detectFloorPlane(keyframe);
    
    // Add IMU constraint if enabled
    if (use_imu_constraints_) {
        addIMUConstraint(keyframe);
    }
    
    // Generate scan context descriptor
    if (scan_context_) {
        keyframe.scan_context = scan_context_->generateContext(downsampled_cloud);
        keyframe.has_descriptor = true;
        scan_context_->addDescriptor(keyframe.scan_context);
    }
    
    keyframes_.push_back(keyframe);
    original_poses_.push_back(pose);
    
    // If we don't have corrected poses yet (no loop closure detected),
    // just copy the original pose
    if (corrected_poses_.size() < original_poses_.size()) {
        corrected_poses_.push_back(pose);
    }
    
    // Try to detect loop closure
    if (keyframes_.size() > nearby_frames_to_skip_) {
        // Run detection in a separate thread to prevent blocking
        std::thread detection_thread([this]() {
            try {
                this->detectAndOptimize();
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in loop closure thread: %s", e.what());
            }
        });
        detection_thread.detach();
    }
}

bool LoopClosure::detectAndOptimize() {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    
    if (keyframes_.size() < 2) {
        return false;
    }
    
    // Add timeout mechanism
    auto start_time = std::chrono::steady_clock::now();
    const std::chrono::seconds timeout(10); // 10 second timeout
    
    // Get the current (latest) keyframe
    KeyFrame& current_frame = keyframes_.back();
    
    // Find potential loop closures
    KeyFrame matched_frame(0, 0.0, State(), nullptr);
    bool loop_found = detectLoopClosure(current_frame, matched_frame);
    
    if (!loop_found) {
        return false;
    }
    
    // Preprocess clouds for feature extraction
    pcl::PointCloud<PointType>::Ptr clean_source = preprocessCloud(current_frame.cloud);
    pcl::PointCloud<PointType>::Ptr clean_target = preprocessCloud(matched_frame.cloud);
    
    // Downsample for faster processing
    pcl::PointCloud<PointType>::Ptr source = downsampleCloud(clean_source, 0.1);
    pcl::PointCloud<PointType>::Ptr target = downsampleCloud(clean_target, 0.1);
    
    // Estimate normals
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    
    // Source normals
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(source);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*source_normals);
    
    // Target normals
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(target);
    ne.compute(*target_normals);
    
    // Compute FPFH features
    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(config_.num_threads);
    
    // Source features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setInputCloud(source);
    fpfh.setInputNormals(source_normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.1);
    fpfh.compute(*source_features);
    
    // Target features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setInputCloud(target);
    fpfh.setInputNormals(target_normals);
    fpfh.compute(*target_features);
    
    // Configure SCIA
    pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> scia;
    
    // Set parameters from config
    scia.setMaximumIterations(50);
    scia.setNumberOfSamples(3);
    scia.setCorrespondenceRandomness(5);
    scia.setMinSampleDistance(config_.loop_closure.coarse_alignment_min_sample_distance);
    scia.setMaxCorrespondenceDistance(config_.loop_closure.coarse_alignment_max_correspondence_distance);
    
    // Set inputs
    scia.setInputSource(source);
    scia.setSourceFeatures(source_features);
    scia.setInputTarget(target);
    scia.setTargetFeatures(target_features);
    
    // Perform alignment
    pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);
    scia.align(*aligned);
    
    if (scia.hasConverged()) {
        Eigen::Matrix4f scia_transform = scia.getFinalTransformation();
        // Use this transform as initial guess for ICP
        gicp_.setInputSource(clean_source);
        gicp_.setInputTarget(clean_target);
        pcl::PointCloud<PointType>::Ptr final_aligned(new pcl::PointCloud<PointType>);
        gicp_.align(*final_aligned, scia_transform);
        
        // Visualize intermediate results
        visualizeICPAlignment(source, target, aligned, "map");
        
        // Get the final transformation for loop closure
        Eigen::Matrix4f loop_constraint = gicp_.getFinalTransformation();
        
        // Check if the transformation is valid (not too large)
        Eigen::Vector3f translation = loop_constraint.block<3,1>(0,3);
        if (translation.norm() > 5.0) {
            ROS_WARN("Loop closure transformation too large, rejecting");
            return false;
        }
        
        ROS_INFO("Loop closure registration successful: fitness score = %.4f", gicp_.getFitnessScore());
        
        // Store the loop closure constraints
        std::vector<std::pair<int, int>> loop_closures;
        std::vector<Eigen::Matrix4f> loop_constraints;
        
        loop_closures.push_back(std::make_pair(current_frame.id, matched_frame.id));
        loop_constraints.push_back(loop_constraint);
        
        // Optimize the pose graph with the new constraints
        bool success = optimizePoseGraph(loop_closures, loop_constraints);
        
        if (success) {
            loop_detected_ = true;
            total_loop_closures_++;
            
            // Calculate and store the loop error (distance between corrected poses)
            const State& loop_start = corrected_poses_[matched_frame.id];
            const State& loop_end = corrected_poses_[current_frame.id];
            latest_loop_error_ = (loop_start.p - loop_end.p).norm();
            
            // Calculate correction magnitude (average correction)
            latest_correction_magnitude_ = 0.0;
            int count = 0;
            for (size_t i = 0; i < original_poses_.size(); i++) {
                latest_correction_magnitude_ += (original_poses_[i].p - corrected_poses_[i].p).norm();
                count++;
            }
            if (count > 0) {
                latest_correction_magnitude_ /= count;
            }
            
            ROS_INFO("Loop closure detected and pose graph optimized: error = %.2f m, avg correction = %.2f m", 
                     latest_loop_error_, latest_correction_magnitude_);
            
            // Rebuild the optimized map
            optimized_map_->clear();
            for (const auto& keyframe : keyframes_) {
                if (keyframe.id < corrected_poses_.size()) {
                    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
                    pcl::transformPointCloud(*keyframe.cloud, *transformed_cloud, 
                                           poseToMatrix(corrected_poses_[keyframe.id]));
                    *optimized_map_ += *transformed_cloud;
                }
            }
            
            // Downsample the final map
            optimized_map_ = downsampleCloud(optimized_map_, 0.1);
        }
        
        return success;
    } else {
        ROS_WARN("Initial alignment failed to converge");
        return false;
    }
}

bool LoopClosure::detectLoopClosure(const KeyFrame& current_frame, KeyFrame& matched_frame) {
    // First try to find matches using scan context
    int best_match_idx = -1;
    double sc_distance = std::numeric_limits<double>::max();
    
    if (current_frame.has_descriptor && scan_context_) {
        auto match_result = scan_context_->findBestMatch(current_frame.scan_context, scan_context_threshold_);
        best_match_idx = match_result.first;
        sc_distance = match_result.second;
        
        if (best_match_idx >= 0) {
            matched_frame = keyframes_[best_match_idx];
            ROS_INFO("Loop closure candidate found via scan context: current frame %d with frame %d (distance: %.3f)",
                    current_frame.id, best_match_idx, sc_distance);
            return true;
        }
    }
    
    // If scan context didn't find a match, fall back to distance-based method
    // Skip the most recent frames
    int max_frame_to_check = std::max(0, static_cast<int>(keyframes_.size()) - nearby_frames_to_skip_);
    
    // Best match variables
    double best_score = std::numeric_limits<double>::max();
    best_match_idx = -1;
    
    // Add timeout mechanism
    auto start_time = std::chrono::steady_clock::now();
    const std::chrono::seconds timeout(5); // 5 second timeout
    
    // Check each candidate frame
    for (int i = 0; i < max_frame_to_check; ++i) {
        const KeyFrame& candidate = keyframes_[i];
        
        // Compute distance between current and candidate
        Eigen::Vector3f current_pos = current_frame.pose.p;
        Eigen::Vector3f candidate_pos = candidate.pose.p;
        float distance = (current_pos - candidate_pos).norm();
        
        // If within distance threshold, perform detailed matching
        if (distance < distance_threshold_) {
            // Simple feature matching could be performed here
            // For now, we'll use the distance as our score
            double score = distance;
            
            if (score < best_score) {
                best_score = score;
                best_match_idx = i;
            }
        }
        
        // Check for timeout
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            ROS_WARN("Loop closure detection timed out");
            return false;
        }
    }
    
    // If we found a good match
    if (best_match_idx >= 0 && best_score < scan_match_threshold_) {
        matched_frame = keyframes_[best_match_idx];
        ROS_INFO("Loop closure candidate found via distance: current frame %d with frame %d (distance: %.3f m)",
                current_frame.id, best_match_idx, best_score);
        return true;
    }
    
    return false;
}

bool LoopClosure::optimizePoseGraph(const std::vector<std::pair<int, int>>& loop_closures,
                                   const std::vector<Eigen::Matrix4f>& loop_constraints) {
    // Add timeout mechanism
    auto start_time = std::chrono::steady_clock::now();
    const std::chrono::seconds timeout(15); // 15 second timeout
    bool skipped_constraints = false;
    
    // Clear previous optimization data
    optimizer_.clear();
    
    // Parallelize vertex and edge creation using OpenMP tasks
    #pragma omp parallel
    {
        #pragma omp single
        {
            // Add vertices (robot poses)
            for (size_t i = 0; i < original_poses_.size(); ++i) {
                #pragma omp task
                {
                    const State& pose = original_poses_[i];
                    Eigen::Isometry3d pose_isometry = Eigen::Isometry3d::Identity();
                    
                    // Convert to Isometry3d
                    pose_isometry.translation() = pose.p.cast<double>();
                    pose_isometry.linear() = pose.q.toRotationMatrix().cast<double>();
                    
                    // Create vertex
                    g2o::VertexSE3* vertex = new g2o::VertexSE3();
                    vertex->setId(i);
                    vertex->setEstimate(pose_isometry);
                    
                    // Fix the first vertex to constrain the gauge freedom
                    if (i == 0) {
                        vertex->setFixed(true);
                    }
                    
                    #pragma omp critical
                    {
                        optimizer_.addVertex(vertex);
                    }
                }
            }
            
            #pragma omp taskwait
            
            // Add odometry edges
            for (size_t i = 1; i < original_poses_.size(); ++i) {
                #pragma omp task
                {
                    const State& prev_pose = original_poses_[i-1];
                    const State& curr_pose = original_poses_[i];
                    
                    // Compute relative transform
                    Eigen::Matrix4f prev_mat = poseToMatrix(prev_pose);
                    Eigen::Matrix4f curr_mat = poseToMatrix(curr_pose);
                    Eigen::Matrix4f rel_transform = prev_mat.inverse() * curr_mat;
                    
                    // Convert to Isometry3d
                    Eigen::Isometry3d rel_isometry = Eigen::Isometry3d::Identity();
                    rel_isometry.translation() = rel_transform.block<3,1>(0,3).cast<double>();
                    rel_isometry.linear() = rel_transform.block<3,3>(0,0).cast<double>();
                    
                    // Create edge
                    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                    edge->setVertex(0, optimizer_.vertex(i-1));
                    edge->setVertex(1, optimizer_.vertex(i));
                    edge->setMeasurement(rel_isometry);
                    
                    // Set the information matrix
                    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * odometry_weight_;
                    edge->setInformation(information);
                    
                    #pragma omp critical
                    {
                        optimizer_.addEdge(edge);
                    }
                }
            }
            
            #pragma omp taskwait
            
            // Pre-check loop closure constraints and filter out invalid ones
            std::vector<size_t> valid_loop_indices;
            for (size_t i = 0; i < loop_closures.size(); ++i) {
                int from_id = loop_closures[i].first;
                int to_id = loop_closures[i].second;
                
                if (from_id >= optimizer_.vertices().size() || to_id >= optimizer_.vertices().size()) {
                    skipped_constraints = true;
                    ROS_WARN("Skipping loop closure constraint with invalid vertex IDs: %d -> %d", from_id, to_id);
                } else {
                    valid_loop_indices.push_back(i);
                }
            }
            
            // Now process only the valid loops in parallel
            #pragma omp parallel for
            for (size_t idx = 0; idx < valid_loop_indices.size(); ++idx) {
                size_t i = valid_loop_indices[idx];
                int from_id = loop_closures[i].first;
                int to_id = loop_closures[i].second;
                
                // Convert constraint to Isometry3d
                Eigen::Matrix4f constraint = loop_constraints[i];
                Eigen::Isometry3d constraint_isometry = Eigen::Isometry3d::Identity();
                constraint_isometry.translation() = constraint.block<3,1>(0,3).cast<double>();
                constraint_isometry.linear() = constraint.block<3,3>(0,0).cast<double>();
                
                // Create edge
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->setVertex(0, optimizer_.vertex(to_id));
                edge->setVertex(1, optimizer_.vertex(from_id));
                edge->setMeasurement(constraint_isometry);
                
                // Set higher information for loop closures
                Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * loop_closure_weight_;
                edge->setInformation(information);
                
                #pragma omp critical
                {
                    optimizer_.addEdge(edge);
                }
            }
            
            // Add floor plane constraints
            if (use_floor_constraints_ && !floor_constraints_.empty()) {
                // Process constraints sequentially inside the single region
                for (size_t i = 0; i < floor_constraints_.size(); i++) {
                    const auto& constraint = floor_constraints_[i];
                    int id = constraint.keyframe_id;
                    
                    if (id >= optimizer_.vertices().size()) {
                        #pragma omp critical
                        {
                            ROS_WARN("Skipping floor constraint with invalid vertex ID: %d", id);
                        }
                        continue;
                    }
                    
                    // Create a plane parameter edge
                    g2o::EdgeSE3Plane* edge = new g2o::EdgeSE3Plane();
                    edge->setVertex(0, optimizer_.vertex(id));
                    
                    // Set the measurement (floor plane in world coordinates)
                    Eigen::Vector4d plane_coeffs = constraint.floor_coeffs.cast<double>();
                    edge->setMeasurement(plane_coeffs);
                    
                    // Set information matrix (weight)
                    Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * floor_plane_weight_;
                    edge->setInformation(information);
                    
                    // Add robust kernel
                    g2o::RobustKernelHuber* kernel = new g2o::RobustKernelHuber;
                    kernel->setDelta(0.1);
                    edge->setRobustKernel(kernel);
                    
                    #pragma omp critical
                    {
                        optimizer_.addEdge(edge);
                    }
                }
            }
            
            // Add IMU orientation constraints
            if (use_imu_constraints_ && !imu_constraints_.empty()) {
                // Process constraints sequentially inside the single region
                for (size_t i = 0; i < imu_constraints_.size(); i++) {
                    const auto& constraint = imu_constraints_[i];
                    int id = constraint.keyframe_id;
                    
                    if (id >= optimizer_.vertices().size()) {
                        #pragma omp critical
                        {
                            ROS_WARN("Skipping IMU constraint with invalid vertex ID: %d", id);
                        }
                        continue;
                    }
                    
                    // Create an IMU orientation edge
                    g2o::EdgeSE3GravityDirection* edge = new g2o::EdgeSE3GravityDirection();
                    edge->setVertex(0, optimizer_.vertex(id));
                    
                    // Set the measurement (gravity direction in world frame)
                    Eigen::Vector3d gravity_dir = constraint.gravity_dir.cast<double>();
                    gravity_dir.normalize();
                    edge->setMeasurement(gravity_dir);
                    
                    // Set information matrix (weight)
                    Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * imu_orientation_weight_;
                    edge->setInformation(information);
                    
                    // Add robust kernel
                    g2o::RobustKernelHuber* kernel = new g2o::RobustKernelHuber;
                    kernel->setDelta(0.1);
                    edge->setRobustKernel(kernel);
                    
                    #pragma omp critical
                    {
                        optimizer_.addEdge(edge);
                    }
                }
            }
        }
    }
    
    // Set up optimizer - Improved options for faster convergence
    optimizer_.setVerbose(false);
    optimizer_.initializeOptimization();
    
    // Run optimization with timeout monitoring
    bool optimized = false;
    std::thread optimization_thread([&]() {
        try {
            optimized = optimizer_.optimize(10);
        } catch (const std::exception& e) {
            ROS_ERROR("Optimization exception: %s", e.what());
        }
    });
    
    // Wait with timeout
    for (int i = 0; i < 150; i++) { // Check every 100ms (15s total)
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!optimization_thread.joinable()) {
            break; // Thread completed
        }
    }
    
    // If not complete, kill thread
    if (optimization_thread.joinable()) {
        ROS_WARN("Optimization timed out, terminating");
        optimization_thread.detach(); // Let it run but don't wait
        return false;
    }
    
    if (!optimized) {
        ROS_WARN("g2o optimization failed");
        return false;
    }
    
    // Update corrected poses in parallel
    corrected_poses_.resize(original_poses_.size());
    
    #pragma omp parallel for
    for (size_t i = 0; i < original_poses_.size(); ++i) {
        g2o::VertexSE3* vertex = static_cast<g2o::VertexSE3*>(optimizer_.vertex(i));
        Eigen::Isometry3d optimized_pose = vertex->estimate();
        
        // Convert back to our State format
        State corrected_pose = original_poses_[i]; // Copy timestamp and other fields
        corrected_pose.p = optimized_pose.translation().cast<float>();
        corrected_pose.q = Eigen::Quaternionf(optimized_pose.rotation().cast<float>());
        
        corrected_poses_[i] = corrected_pose;
    }
    
    if (skipped_constraints) {
        ROS_WARN("Some loop closure constraints were skipped due to invalid vertex IDs");
    }
    
    return true;
}

std::vector<State> LoopClosure::getCorrectedPoses() {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return corrected_poses_;
}

Eigen::Matrix4f LoopClosure::getCorrection(double timestamp) const {
    // Find the closest pose in time
    int closest_idx = -1;
    double min_time_diff = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < original_poses_.size(); ++i) {
        double time_diff = std::abs(original_poses_[i].time - timestamp);
        if (time_diff < min_time_diff) {
            min_time_diff = time_diff;
            closest_idx = i;
        }
    }
    
    if (closest_idx >= 0 && closest_idx < corrected_poses_.size()) {
        // Compute the correction transform: T_corrected * T_original^-1
        Eigen::Matrix4f original_matrix = poseToMatrix(original_poses_[closest_idx]);
        Eigen::Matrix4f corrected_matrix = poseToMatrix(corrected_poses_[closest_idx]);
        return corrected_matrix * original_matrix.inverse();
    }
    
    // If no correction found, return identity
    return Eigen::Matrix4f::Identity();
}

pcl::PointCloud<PointType>::Ptr LoopClosure::correctPointCloud(
    pcl::PointCloud<PointType>::Ptr cloud, double timestamp) {
    
    Eigen::Matrix4f correction = getCorrection(timestamp);
    
    pcl::PointCloud<PointType>::Ptr corrected_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cloud, *corrected_cloud, correction);
    
    return corrected_cloud;
}

pcl::PointCloud<PointType>::Ptr LoopClosure::getOptimizedMap() {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return optimized_map_;
}

Eigen::Matrix4f LoopClosure::poseToMatrix(const State& pose) const {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    // Set rotation
    matrix.block<3,3>(0,0) = pose.q.toRotationMatrix();
    
    // Set translation
    matrix.block<3,1>(0,3) = pose.p;
    
    return matrix;
}

State LoopClosure::matrixToPose(const Eigen::Matrix4f& matrix, double timestamp) {
    State pose;
    
    // Set timestamp
    pose.time = timestamp;
    
    // Set translation
    pose.p = matrix.block<3,1>(0,3);
    
    // Set rotation
    Eigen::Matrix3f rotation_matrix = matrix.block<3,3>(0,0);
    pose.q = Eigen::Quaternionf(rotation_matrix);
    
    return pose;
}

pcl::PointCloud<PointType>::Ptr LoopClosure::downsampleCloud(
    pcl::PointCloud<PointType>::Ptr cloud, float leaf_size) {
    
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
    
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*downsampled_cloud);
    
    return downsampled_cloud;
}

// Implement floor plane detection
bool LoopClosure::detectFloorPlane(KeyFrame& keyframe) {
    if (!use_floor_constraints_) {
        return false;
    }
    
    // Transform cloud to world frame for plane detection
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*keyframe.cloud, *transformed_cloud, poseToMatrix(keyframe.pose));
    
    // Downsample for faster processing
    pcl::PointCloud<PointType>::Ptr downsampled(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(transformed_cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_filter.filter(*downsampled);
    
    if (downsampled->size() < floor_min_points_) {
        return false;
    }
    
    // Use RANSAC to detect planes
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);  // 5cm threshold for inliers
    seg.setMaxIterations(100);
    
    seg.setInputCloud(downsampled);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() < floor_min_points_) {
        return false;
    }
    
    // Check if the plane is horizontal (normal close to z-axis)
    Eigen::Vector3f normal(coefficients->values[0], 
                          coefficients->values[1], 
                          coefficients->values[2]);
    normal.normalize();
    
    // Floor plane should have normal close to (0,0,1) in world frame
    float cos_angle = normal.dot(Eigen::Vector3f(0, 0, 1));
    
    // Check if plane is horizontal enough
    if (std::abs(cos_angle) < floor_normal_thresh_) {
        return false;
    }
    
    // Ensure normal points upward
    if (cos_angle < 0) {
        normal = -normal;
        coefficients->values[0] = -coefficients->values[0];
        coefficients->values[1] = -coefficients->values[1];
        coefficients->values[2] = -coefficients->values[2];
        coefficients->values[3] = -coefficients->values[3];
    }
    
    // Check height of the plane (d parameter)
    float height = -coefficients->values[3] / normal.z();
    if (std::abs(height) > floor_max_height_) {
        return false;
    }
    
    // Store the floor plane equation ax + by + cz + d = 0
    keyframe.has_floor = true;
    keyframe.floor_plane = Eigen::Vector4f(
        coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2],
        coefficients->values[3]
    );
    
    // Store as a constraint
    FloorPlaneConstraint constraint(keyframe.id, keyframe.floor_plane);
    floor_constraints_.push_back(constraint);
    
    ROS_INFO("Detected floor plane in keyframe %d with normal [%.2f, %.2f, %.2f] at height %.2f m",
             keyframe.id, normal.x(), normal.y(), normal.z(), height);
    
    return true;
}

// Add IMU constraint based on state gravity vector
void LoopClosure::addIMUConstraint(const KeyFrame& keyframe) {
    if (!use_imu_constraints_) {
        return;
    }
    
    // Get gravity direction from rotation matrix
    // Assuming gravity points along z-axis in world frame
    Eigen::Vector3f world_gravity(0, 0, 1);
    
    // Store IMU constraint
    IMUConstraint constraint(keyframe.id, world_gravity);
    imu_constraints_.push_back(constraint);
}

pcl::PointCloud<PointType>::Ptr LoopClosure::preprocessCloud(pcl::PointCloud<PointType>::Ptr cloud) {
    // Remove NaN points
    std::vector<int> indices;
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);
    
    // Statistical outlier removal
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    sor.filter(*output);
    
    return output;
}

void LoopClosure::visualizeICPAlignment(const pcl::PointCloud<PointType>::Ptr& source,
                                      const pcl::PointCloud<PointType>::Ptr& target,
                                      const pcl::PointCloud<PointType>::Ptr& aligned,
                                      const std::string& frame_id) {
    static ros::NodeHandle nh;
    static ros::Publisher source_pub = nh.advertise<sensor_msgs::PointCloud2>
                                     ("loop_closure/source_cloud", 1);
    static ros::Publisher target_pub = nh.advertise<sensor_msgs::PointCloud2>
                                     ("loop_closure/target_cloud", 1);
    static ros::Publisher aligned_pub = nh.advertise<sensor_msgs::PointCloud2>
                                     ("loop_closure/aligned_cloud", 1);
    
    // Publish the clouds
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*source, msg);
    msg.header.frame_id = frame_id;
    source_pub.publish(msg);
    
    pcl::toROSMsg(*target, msg);
    msg.header.frame_id = frame_id;
    target_pub.publish(msg);
    
    if (aligned) {
        pcl::toROSMsg(*aligned, msg);
        msg.header.frame_id = frame_id;
        aligned_pub.publish(msg);
    }
}

} // namespace fast_limo 