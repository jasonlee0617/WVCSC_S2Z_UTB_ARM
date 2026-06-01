include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,                -- 引入地图构建器配置（来自map_builder.lua）
    trajectory_builder = TRAJECTORY_BUILDER,  -- 引入轨迹构建器配置（来自trajectory_builder.lua）
    map_frame = "map",                        -- 全局地图坐标系（最终建图的基准坐标系）
    tracking_frame = "base_footprint", 	      -- 跟踪坐标系（机器人底盘中心，激光/IMU等传感器的参考基座）
    published_frame = "odom", 	              -- 发布的位姿参考坐标系（对外输出的里程计坐标系）
    odom_frame = "odom",                      -- 里程计坐标系（通常是机器人自身里程计的坐标系）
    provide_odom_frame = false,               -- 是否由Cartographer生成odom坐标系（false表示使用外部里程计）
    publish_frame_projected_to_2d = true,     -- 将发布的坐标系投影到2D平面（忽略z轴，适合2D建图）
    use_odometry = true,                      -- 是否使用外部里程计数据（如轮式里程计，辅助位姿估计）
    use_nav_sat = false,                      -- 是否使用GPS/导航卫星数据（2D建图一般关闭）
    use_landmarks = false,                    -- 是否使用人工路标（如二维码，一般关闭）
    num_laser_scans = 1,                      -- 使用的激光雷达数量（1个单线激光）
    num_multi_echo_laser_scans = 0,           -- 多回波激光数量（0表示不用）
    num_subdivisions_per_laser_scan = 1,      -- 每个激光扫描的细分次数（1表示不细分）
    num_point_clouds = 0,                     -- 点云数量（0表示不用3D点云）
    lookup_transform_timeout_sec = 2.0,       -- 坐标系变换查找超时时间（2秒，防止等待卡死）
    submap_publish_period_sec = 0.3,          -- 子地图发布频率（0.3秒/次，控制rviz可视化刷新率）
     pose_publish_period_sec = 5e-3,          -- 位姿发布频率（5毫秒/次，即200Hz）
    trajectory_publish_period_sec = 30e-3,   -- 轨迹发布频率（30毫秒/次，即约33Hz）
    rangefinder_sampling_ratio = 1.,         -- 激光数据采样率（1表示全部使用，<1则降采样）
    odometry_sampling_ratio = 0.5,           -- 里程计数据采样率（0.5表示每2帧取1帧，降低计算量）
    fixed_frame_pose_sampling_ratio = 1.,    -- 固定坐标系位姿采样率（1表示全部使用）
    imu_sampling_ratio = 1.,                 -- IMU数据采样率（1表示全部使用，这里未启用IMU，无影响）
    landmarks_sampling_ratio = 1.,           -- 路标采样率（1表示全部使用，这里未启用，无影响）
}


MAP_BUILDER.use_trajectory_builder_2d = true  -- 启用2D轨迹构建器（核心：指定做2D SLAM）
MAP_BUILDER.num_background_threads = 4        -- 后台处理线程数（4线程，加快建图计算速度）

-- 激光数据范围限制
TRAJECTORY_BUILDER_2D.min_range = 0.50        -- 激光最小有效距离（过滤0.5米内的噪声点）
TRAJECTORY_BUILDER_2D.max_range = 30.0        -- 激光最大有效距离（只使用30米内的激光点）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 30.  -- 无数据区域的虚拟射线长度（30米，补全地图空白）

-- 传感器使用开关
TRAJECTORY_BUILDER_2D.use_imu_data = false    -- 不使用IMU数据（依赖轮式里程计+激光）


-- 实时相关扫描匹配（前端配准）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关扫描匹配（提升前端配准精度）

-- 运动滤波器（减少无效计算）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)  -- 角度变化阈值（0.3弧度≈17度，角度变化小于此值则不处理）
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2          -- 距离变化阈值（0.2米，距离变化小于此值则不处理，减小阈值可提升响应精度）

-- Ceres扫描匹配（前端优化）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 80       -- 平移权重（80，权重越高，平移配准越"顽固"，不易被旋转影响）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.0        -- 旋转权重（80，权重越高，旋转配准越稳定，降低此值可减少旋转误差）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30  -- 配准迭代次数（30次，次数越多配准越准，但耗时略增）

-- 数据累积与子地图
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 每帧激光数据累积次数（1表示不累积，实时处理）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30     -- 每个子地图包含的激光帧数（30帧，子地图越大，局部建图越稳定）

-- 约束构建器（回环检测）
POSE_GRAPH.constraint_builder.min_score = 0.60                -- 回环约束最小匹配分数（0.6，分数越高，回环匹配越严格，减少错误回环）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- 全局定位最小分数（0.65，全局重定位时的匹配阈值，更高则更精准）
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01           -- 局部回环采样率（0.01，1%的采样率，降低计算量）
POSE_GRAPH.constraint_builder.max_constraint_distance = 40.0  -- 最大回环检测距离（40米，默认20米，增大可检测更远的回环，适合大场景）

-- 全局采样与优化触发
POSE_GRAPH.global_sampling_ratio = 0.01                       -- 全局回环采样率（0.01，1%的采样率，减少全局回环计算量）
POSE_GRAPH.optimize_every_n_nodes = 20                        -- 每累积20个节点触发一次全局优化（节点数越少，优化越频繁，精度越高但耗时略增）

-- 优化器参数（后端求解）
POSE_GRAPH.optimization_problem.huber_scale = 1e2             -- Huber核函数尺度（100，增大可降低异常值（错误回环）的影响，提升优化稳定性）
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- 优化迭代次数（50次，次数越多优化越充分）
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true  -- 启用非单调步骤（允许优化过程中暂时变差，更容易找到全局最优解）


return options
