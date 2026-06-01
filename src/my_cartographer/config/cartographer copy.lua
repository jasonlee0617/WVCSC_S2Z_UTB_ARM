include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_footprint", 	-- "imu_link",gyro_link


    published_frame = "odom_combined", 	-- 发布map到published_frame之间的

    odom_frame = "odom", 		-- "odom" 里程计的坐标系名字,仅在provide_odom_frame为true时使用
    -- provide_odom_frame = true,  -- 如果为true,则tf树为map->odom->footprint
    -- publish_frame_projected_to_2d = true,  --是否将坐标系投影到平面上
    publish_frame_projected_to_2d = false,  --true 发布一个odom ros2 run tf2_tools view_frame 可以看到map->odom->laser_link
    provide_odom_frame = false,      --如果enable, 则local, non-loop-closed, continuous pose 将作为 odom_frame发布在 map_frame. 
    -- use_odometry = false,   -- 如果启用，请在/odom上订阅nav_msgs/Odometry
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    pose_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 30e-3,
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 0.1,
    fixed_frame_pose_sampling_ratio = 1.,
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.2 
TRAJECTORY_BUILDER_2D.max_range = 50.0  
TRAJECTORY_BUILDER_2D.min_z = -5.0
TRAJECTORY_BUILDER_2D.max_z = 5.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 50.
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
--  TEST 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.03
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
MAP_BUILDER.num_background_threads = 4    

--  TEST
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001

-- POSE_GRAPH.optimize_every_n_nodes = 90    


return options
