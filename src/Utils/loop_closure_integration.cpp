namespace loop_closure {
    // Publishers
    static ros::Publisher cloud_pub;
    static ros::Publisher odom_pub;
    static ros::Publisher floor_planes_pub;
    static std::string world_frame_id = "map";  // Default value

    // Initialize publishers
    void init(ros::NodeHandle& nh, const std::string& world_frame) {
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("loop_corrected_cloud", 1);
        odom_pub = nh.advertise<nav_msgs::Odometry>("loop_corrected_odom", 1);
        floor_planes_pub = nh.advertise<visualization_msgs::MarkerArray>("floor_planes", 1);
        world_frame_id = world_frame;
    }

    // Publish loop closure results
    void publish(fast_limo::Localizer& localizer, const fast_limo::Config& config) {
        if (config.loop_closure.active) {
            // Publish loop-corrected point cloud and odometry as before
            // ...
            
            // Publish detected floor planes if enabled
            if (config.loop_closure.use_floor_constraints) {
                std::vector<fast_limo::FloorPlaneConstraint> floor_constraints = 
                    localizer.get_loop_closure_floor_constraints();
                
                if (!floor_constraints.empty()) {
                    visualization_msgs::MarkerArray floor_markers = 
                        visualize_limo::getFloorPlaneMarkers(floor_constraints, world_frame_id);
                    floor_planes_pub.publish(floor_markers);
                }
            }
        }
    }
} 