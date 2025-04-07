include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
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
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,  -- 保持完整的里程计数据
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 提高扫描累积，改善特征提取
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2  -- 增加累积的激光扫描数

-- 传感器范围优化
TRAJECTORY_BUILDER_2D.min_range = 0.2  -- 降低最小范围以获取更多近距离特征
TRAJECTORY_BUILDER_2D.max_range = 30.0  -- 根据图像中的环境尺寸调整
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 扫描匹配器优化
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0  -- 大幅提高平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0  -- 大幅提高旋转权重，减少转弯处漂移
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 增加占用空间的权重

-- 自适应体素滤波器参数优化
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5  -- 减小最大长度
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200  -- 增加最小点数
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 25.0  -- 调整最大范围

-- 子图参数优化
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80  -- 减少每个子图的激光扫描数，提高闭环检测频率
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 提高地图分辨率

-- 位姿图优化参数
POSE_GRAPH.optimization_problem.huber_scale = 5e1  -- 降低以增加对异常值的敏感度
POSE_GRAPH.optimize_every_n_nodes = 20  -- 减少优化间隔，更频繁地进行全局优化
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 降低最小分数以增加闭环检测机会
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  -- 设置全局定位的最小分数

-- 闭环检测增强
POSE_GRAPH.global_sampling_ratio = 0.01  -- 增加全局采样率，提高闭环检测概率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5  -- 增加约束构建器采样率
POSE_GRAPH.constraint_builder.max_constraint_distance = 30.0  -- 增加最大约束距离，适应环境大小
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.0e5  -- 大幅增加闭环约束的平移权重
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.0e6  -- 大幅增加闭环约束的旋转权重

-- 全局优化参数
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- 增加局部SLAM位姿的平移权重
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5  -- 增加局部SLAM位姿的旋转权重
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5  -- 增加里程计平移权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5  -- 增加里程计旋转权重

-- 增加优化迭代次数
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- 增加最大迭代次数

return options
