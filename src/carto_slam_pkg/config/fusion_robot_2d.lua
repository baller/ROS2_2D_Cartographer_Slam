include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",   -- 改为IMU框架，因为IMU提供最准确的方向信息
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,      -- 使用我们从编码器生成的里程计
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
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,  -- 保持1.0以使用所有IMU数据
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 100.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 修改为true以使用我们添加的IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0  -- 增加平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.0     -- 增加旋转权重

-- IMU相关设置
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- 位姿图优化参数
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3  -- 使用IMU加速度数据
POSE_GRAPH.optimization_problem.rotation_weight = 3e5     -- 使用IMU旋转数据
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.0
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.0e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.0e4


return options
