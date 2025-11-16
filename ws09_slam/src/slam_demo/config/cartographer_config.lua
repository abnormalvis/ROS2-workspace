-- 版权信息
-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "trajectory_builder.lua"
include "pose_graph.lua"

options = {
  trajectory_builder = trajectory_builder,
  pose_graph = pose_graph,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
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
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  scan_matcher_options = {
    occupied_space_cost_functor_option_2d = {

      translation_delta_cost_weight = 1e-1,
      rotation_delta_cost_weight = 1e-1,
    },
    ceres_scan_matcher_2d_options = {

      occupied_space_cost_functor_option_2d = {
        weight = 1.,
        huber_scale = 1.,
        translation_delta_cost_functor_option_2d = {
          weight = 1e-1,
          huber_scale = 1.,
        },
        rotation_delta_cost_functor_option_2d = {
          weight = 1e-1,
          huber_scale = 1.,
        },
      },
    },
    ceres_scan_matcher_translation_weight = 5e2,
    ceres_scan_matcher_rotation_weight = 4e2,
  },
}

trajectory_builder = {
  trajectory_builder_2d = {

    frames_add_min_range = 0.,
    frames_add_max_range = 15.,
    frames_add_empty_ray_weight = 0.,
    frames_add_scan_weight = 1.,
    frames_add_queue_count = 10,
    frames_add_queue_size = 10,
    frames_add_queue_timeout = 10.,


    min_range = 0.3,
    max_range = 12.,
    min_z = -0.8,
    max_z = 2.,
    missing_data_ray_length = 5.,
    num_accumulated_range_data = 1,
    voxel_filter_size = 0.05,


    adaptive_voxel_filter = {
      max_length = 0.5,
      min_num_points = 200,
      max_range = 50.,
    },


    loop_closure_adaptive_voxel_filter = {
      max_length = 0.9,
      min_num_points = 100,
      max_range = 50.,
    },


    use_online_correlative_scan_matching = true,
    real_time_correlative_scan_matcher = {
      linear_search_window = 0.1,
      angular_search_window = math.rad(20.),
      translation_delta_cost_weight = 1e-1,
      rotation_delta_cost_weight = 1e-1,
    },


    ceres_scan_matcher = {
      occupied_space_cost_functor = 1.,
      translation_weight = 10.,
      rotation_weight = 40.,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 20,
        num_threads = 1,
      },
    },


    motion_filter = {
      max_time_seconds = 5.,
      max_distance_meters = 0.2,
      max_angle_radians = math.rad(1.),
    },


    -- TODO(sheepslime94): Use `pose_extrapolator_2d` configuration.
    pose_extrapolator = {
      use_imu_based = false,
      constant_velocity = {
        imu_gravity_time_constant = 10,
        pose_extrapolator_options = {
          extrapolation_duration = .001,
          gravity_time_constant = 9.8,
        },
        imu_tracker = {
          gravity_time_constant = 9.8,
          imu_tracker_options = {
            num_odometry_states = 10,
            num_imu_states = 10,
          },
        },
      },
      imu_based = {
        pose_extrapolator_options = {
          imu_gravity_time_constant = 10,
          pose_extrapolator_options = {
            num_odometry_states = 10,
            num_imu_states = 10,
          },
        },
        imu_tracker = {
          gravity_time_constant = 9.8,
          imu_tracker_options = {
            num_odometry_states = 10,
            num_imu_states = 10,
          },
        },
      },
    },


    submaps = {
      resolution = 0.05,
      num_range_data = 90,
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
    },
  },
}

-- 位姿图配置
pose_graph = {
  -- 优化间隔
  optimize_every_n_nodes = 90,
  -- 约束构建器配置
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.0,
    min_score = 0.45,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,
    fast_correlative_scan_matcher = {
      linear_search_window = 0.15,
      angular_search_window = math.rad(30.0),
      branch_and_bound_depth = 8,
    },
    ceres_scan_matcher = {
      occupied_space_cost_functor = 1.0,
      ceres_scan_matcher_translation_weight = 10.0,
      ceres_scan_matcher_rotation_weight = 1.0,
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      linear_xy_search_window = 0.15,
      linear_z_search_window = 0.02,
      angular_search_window = math.rad(15.0),
    },
    ceres_scan_matcher_3d = {
      occupied_space_cost_functor = 1.0,
      ceres_scan_matcher_translation_weight = 10.0,
      ceres_scan_matcher_rotation_weight = 1.0,
    },
  },

  -- 优化问题配置
  optimizer_options = {
    huber_scale = 1e1,
    acceleration_threshold = 1.0,
    acceleration_order = 3,
    scan_translation_weight = 1e2,
    scan_rotation_weight = 4e2,
    odometry_translation_weight = 1e2,
    odometry_rotation_weight = 1e3,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },

  -- 最大终末位姿数量
  max_num_final_iterations = 200,
  -- 全局采样比例
  global_sampling_ratio = 0.003,
  -- 全局约束搜索方向（向前或逆向）
  global_constraint_search_after_n_seconds = 10,
  -- 是否对最后的优化进行全局搜索
  -- global_search_refinement = false,

  -- 迭代旋转
  iteration_rotation = {
    enabled = false,
    -- 归一化角度阈值
    angle_threshold = math.rad(10.0),
  },

  -- 重叠子图剔除搅拌器
  overlapping_submaps_trimmer_2d = {
    fresh_submaps_count = 1,
    min_covered_area = 2.0,
    min_added_submaps_count = 5,
  },
}

return options