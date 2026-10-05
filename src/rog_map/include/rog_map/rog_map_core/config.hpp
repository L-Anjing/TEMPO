/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>
#include <stdexcept>

#include <nav2_util/node_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rog_map/rog_map_core/common_lib.hpp>
#include <rog_map/terrain_types.hpp>

#ifndef ORIGIN_AT_CORNER
#ifndef ORIGIN_AT_CENTER
#error "Please define either ORIGIN_AT_CORNER or ORIGIN_AT_CENTER, but not both."
#endif
#endif

#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
#error "Cannot use both ORIGIN_AT_CENTER and ORIGIN_AT_CENTER at the same time. Please define only one."
#endif
#endif

namespace rog_map {
using color_text::BLUE;
using color_text::RED;
using color_text::RESET;
using std::string;
using std::vector;
typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;

class Config
{
  static std::string replaceCmakeRootDir(const std::string & input)
  {
#define CMAKE_ROOT_DIR(name) (string(string(ROOT_DIR) + name))
    std::string replaced;
    std::regex cmakeRootDirRegex(R"(\$\{CMAKE_ROOT_DIR\}/)");
    if (std::regex_search(input, cmakeRootDirRegex)) {
      replaced = std::regex_replace(input, cmakeRootDirRegex, "");
      replaced = CMAKE_ROOT_DIR(replaced);
      printf("\033[0;32m The pcd_name is remapped to %s\033[0;0m \n", replaced.c_str());
    } else {
      replaced = input;
    }
    return replaced;
  }

public:
  Config(){};

  template <typename NodeT> void loadFromRosNode(const NodeT & node, const string & prefix)
  {
    auto load = [&node, &prefix](const string & key, auto & value) {
      const string param_name = prefix + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(value));
      if (!node->get_parameter(param_name, value)) {
        RCLCPP_WARN(node->get_logger(),
          "[ROGMap Config] parameter '%s' not found after declaration, using default.",
          param_name.c_str());
      }
    };

    auto loadVec3 = [&node, &prefix](const string & key, const vector<double> & default_value) {
      vector<double> values = default_value;
      const string param_name = prefix + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(values));
      if (!node->get_parameter(param_name, values)) {
        RCLCPP_WARN(node->get_logger(),
          "[ROGMap Config] parameter '%s' not found after declaration, using default vector.",
          param_name.c_str());
      }
      if (values.size() != 3) {
        throw std::invalid_argument(param_name + " size is not 3!");
      }
      return Vec3f(values[0], values[1], values[2]);
    };

    auto loadVec2 = [&node, &prefix](const string & key, const vector<double> & default_value) {
      vector<double> values = default_value;
      const string param_name = prefix + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(values));
      if (!node->get_parameter(param_name, values)) {
        RCLCPP_WARN(node->get_logger(),
          "[ROGMap Config] parameter '%s' not found after declaration, using default vector.",
          param_name.c_str());
      }
      if (values.size() != 2) {
        throw std::invalid_argument(param_name + " size is not 2!");
      }
      return values;
    };

    esdf_resolution = 0.2;
    esdf_en = false;
    load("esdf.resolution", esdf_resolution);
    load("esdf.enable", esdf_en);
    esdf_local_update_box = loadVec3("esdf.local_update_box", vector<double>{10.0, 10.0, 2.0});

    load_pcd_en = false;
    pcd_name = "map.pcd";
    load("load_pcd_en", load_pcd_en);
    load("pcd_name", pcd_name);
    if (load_pcd_en) {
      pcd_name = replaceCmakeRootDir(pcd_name);
    }

    map_sliding_en = true;
    map_sliding_thresh = -1.0;
    map_center_offset_enable = false;
    map_center_offset = Vec3f::Zero();
    load("map_sliding.enable", map_sliding_en);
    load("map_sliding.threshold", map_sliding_thresh);
    load("map_sliding.center_offset_enable", map_center_offset_enable);
    vector<double> map_center_offset_values{0.0, 0.0, 0.0};
    load("map_sliding.center_offset", map_center_offset_values);
    if (map_center_offset_values.size() == 3) {
      map_center_offset = Vec3f(
        map_center_offset_values[0], map_center_offset_values[1], map_center_offset_values[2]);
    } else {
      RCLCPP_WARN(node->get_logger(),
        "[ROGMap Config] parameter '%s.map_sliding.center_offset' size is %zu, using zero offset.",
        prefix.c_str(), map_center_offset_values.size());
      map_center_offset = Vec3f::Zero();
    }
    fix_map_origin = loadVec3("fix_map_origin", vector<double>{0.0, 0.0, 0.0});
    frontier_extraction_en = false;
    load("frontier_extraction_en", frontier_extraction_en);

    ros_callback_en = false;
    cloud_topic = "/cloud_registered_full";
    odom_topic = "/lidar_slam/odom";
    odom_timeout = 0.05;
    update_period_ms = 1;
    load("ros_callback.enable", ros_callback_en);
    load("ros_callback.cloud_topic", cloud_topic);
    load("ros_callback.odom_topic", odom_topic);
    load("ros_callback.robot_frame", robot_frame);
    load("ros_callback.odom_timeout", odom_timeout);
    load("ros_callback.update_period_ms", update_period_ms);
    if (update_period_ms <= 0) {
      update_period_ms = 1;
    }

    sensor_in_robot = loadVec3("sensor_in_robot", vector<double>{0.0, 0.0, 0.0});
    vector<double> sensor_rotation{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    load("sensor_in_robot_R", sensor_rotation);
    if (sensor_rotation.size() != 9 || !sensor_in_robot.allFinite()) {
      throw std::invalid_argument("ROGMap sensor extrinsics require a finite translation and 9 rotation values");
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) { sensor_in_robot_R(row, col) = sensor_rotation[3 * row + col]; }
    }
    if (!sensor_in_robot_R.allFinite() ||
        (sensor_in_robot_R.transpose() * sensor_in_robot_R - Eigen::Matrix3d::Identity()).norm() > 1.e-3 ||
        std::abs(sensor_in_robot_R.determinant() - 1.) > 1.e-3) {
      throw std::invalid_argument("ROGMap sensor_in_robot_R must be a proper rotation matrix");
    }
    sensor_in_robot_q = Eigen::Quaterniond(sensor_in_robot_R).normalized();
    cloud_filter_en = false;
    cloud_filter_z_offset = 0.0;
    cloud_filter_position_frame = "map";
    cloud_filter_mode = "transform_center";
    cloud_filter_remove_inside = true;
    debug_clouds_enable = false;
    debug_clouds_half_extent_xy = 0.6;
    debug_clouds_min_z = -0.5;
    debug_clouds_max_z = 1.0;
    load("debug_clouds.enable", debug_clouds_enable);
    load("debug_pipeline.enable", debug_pipeline_enable);
    load("debug_clouds.half_extent_xy", debug_clouds_half_extent_xy);
    load("debug_clouds.min_z", debug_clouds_min_z);
    load("debug_clouds.max_z", debug_clouds_max_z);

    cloud_filter_log_stats = false;
    cloud_filter_stats_log_period_ms = 1000;
    cloud_filter_publish_visualization = false;
    cloud_filter_visualization_topic = "/rog_map/cloud_filter/markers";
    cloud_filter_visualization_period_ms = 200;
    cloud_filter_z_plane_size = 12.0;
    cloud_filter_z_plane_thickness = 0.03;
    cloud_filter_box_padding = 0.0;
    cloud_filter_center_x = 0.0;
    cloud_filter_center_y = 0.0;
    cloud_filter_center_z = 0.0;
    cloud_filter_size_x = 12.0;
    cloud_filter_size_y = 6.0;
    cloud_filter_size_z = 3.5;
    cloud_filter_centers_x.clear();
    cloud_filter_centers_y.clear();
    cloud_filter_centers_z.clear();
    cloud_filter_sizes_x.clear();
    cloud_filter_sizes_y.clear();
    cloud_filter_sizes_z.clear();
    load("cloud_filter.enable", cloud_filter_en);
    load("cloud_filter.z_filter_enable", cloud_filter_z_filter_enable);
    load("cloud_filter.z_offset", cloud_filter_z_offset);
    load("cloud_filter.position_frame", cloud_filter_position_frame);
    load("cloud_filter.filter_mode", cloud_filter_mode);
    load("cloud_filter.remove_inside", cloud_filter_remove_inside);
    load("cloud_filter.log_stats", cloud_filter_log_stats);
    load("cloud_filter.stats_log_period_ms", cloud_filter_stats_log_period_ms);
    load("cloud_filter.publish_visualization", cloud_filter_publish_visualization);
    load("cloud_filter.visualization_topic", cloud_filter_visualization_topic);
    load("cloud_filter.visualization_period_ms", cloud_filter_visualization_period_ms);
    load("cloud_filter.z_plane_size", cloud_filter_z_plane_size);
    load("cloud_filter.z_plane_thickness", cloud_filter_z_plane_thickness);
    load("cloud_filter.box_padding", cloud_filter_box_padding);
    load("cloud_filter.position.x", cloud_filter_center_x);
    load("cloud_filter.position.y", cloud_filter_center_y);
    load("cloud_filter.position.z", cloud_filter_center_z);
    load("cloud_filter.box_size.x", cloud_filter_size_x);
    load("cloud_filter.box_size.y", cloud_filter_size_y);
    load("cloud_filter.box_size.z", cloud_filter_size_z);
    load("cloud_filter.positions.x", cloud_filter_centers_x);
    load("cloud_filter.positions.y", cloud_filter_centers_y);
    load("cloud_filter.positions.z", cloud_filter_centers_z);
    load("cloud_filter.box_sizes.x", cloud_filter_sizes_x);
    load("cloud_filter.box_sizes.y", cloud_filter_sizes_y);
    load("cloud_filter.box_sizes.z", cloud_filter_sizes_z);
    if (cloud_filter_mode != "transform_cloud" && cloud_filter_mode != "transform_center") {
      RCLCPP_WARN(node->get_logger(),
        "[ROGMap Config] cloud_filter.filter_mode='%s' is invalid, using transform_center.",
        cloud_filter_mode.c_str());
      cloud_filter_mode = "transform_center";
    }

    visualization_en = false;
    frame_id = "world";
    load("frame_id", frame_id);
    visualization_frame_id = "";
    visualization_rate = 5.0;
    load("visualization.enable", visualization_en);
    load("visualization.frame_id", visualization_frame_id);
    load("visualization.rate", visualization_rate);
    if (visualization_frame_id.empty()) {
      visualization_frame_id = frame_id;
    }
    if (visualization_rate <= 0.0) {
      visualization_rate = 5.0;
    }
    visualization_range = loadVec3("visualization.range", vector<double>{10.0, 10.0, 2.0});
    if (visualization_range.minCoeff() <= 0) {
      visualization_en = false;
    }
    // 全局二维地形图可视化（/global_terrain/map）。点数随探索范围增长，单独限频。
    global_map_viz_en = true;
    global_map_viz_rate = 1.0;
    load("visualization.global_map_enable", global_map_viz_en);
    load("visualization.global_map_rate", global_map_viz_rate);
    if (!std::isfinite(global_map_viz_rate) || global_map_viz_rate <= 0.0) {
      global_map_viz_rate = 1.0;
    }

    resolution = 0.1;
    inflation_resolution = 0.1;
    unk_inflation_en = false;
    unk_inflation_step = 1;
    inflation_step = 1;
    intensity_thresh = -1;
    map_size_d = Vec3f(10.0, 10.0, 0.0);
    point_filt_num = 2;
    load("resolution", resolution);
    load("inflation_resolution", inflation_resolution);
    load("unk_inflation_en", unk_inflation_en);
    load("unk_inflation_step", unk_inflation_step);
    load("inflation_step", inflation_step);
    load("intensity_thresh", intensity_thresh);
    map_size_d = loadVec3("map_size", vector<double>{10.0, 10.0, 0.0});
    load("point_filt_num", point_filt_num);
    if (point_filt_num <= 0) {
      point_filt_num = 1;
    }

    raycasting_en = true;
    batch_update_size = 1;
    unk_thresh = 0.70;
    load("time_sync.odom_buffer_sec", odom_buffer_sec);
    load("time_sync.max_odom_gap_sec", max_odom_gap_sec);
    load("time_sync.cloud_wait_timeout_ms", cloud_wait_timeout_ms);
    load("time_sync.max_stamp_age_sec", max_stamp_age_sec);
    if (!std::isfinite(odom_buffer_sec) || odom_buffer_sec <= 0 ||
        !std::isfinite(max_odom_gap_sec) || max_odom_gap_sec <= 0 ||
        cloud_wait_timeout_ms <= 0 || !std::isfinite(max_stamp_age_sec) || max_stamp_age_sec <= 0) {
      throw std::invalid_argument("[ROGMap] invalid time_sync buffer/gap/wait/age configuration");
    }
    p_hit = 0.90f;
    p_miss = 0.49f;
    p_min = 0.12f;
    p_max = 0.98f;
    p_occ = 0.85f;
    p_free = 0.499f;
    parallel_raycast_en = true;
    raycast_num_threads = 4;
    load("raycasting.enable", raycasting_en);
    load("raycasting.batch_update_size", batch_update_size);
    load("raycasting.unk_thresh", unk_thresh);
    load("raycasting.p_hit", p_hit);
    load("raycasting.p_miss", p_miss);
    load("raycasting.p_min", p_min);
    load("raycasting.p_max", p_max);
    load("raycasting.p_occ", p_occ);
    load("raycasting.p_free", p_free);
    if (!std::isfinite(p_min) || !std::isfinite(p_free) ||
        !std::isfinite(p_miss) || !std::isfinite(p_hit) ||
        !std::isfinite(p_occ) || !std::isfinite(p_max) ||
        !(0.0f < p_min && p_min < p_free && p_free < 0.5f) ||
        !(0.0f < p_miss && p_miss < 0.5f) ||
        !(0.5f < p_hit && p_hit < 1.0f) ||
        !(0.5f < p_occ && p_occ < p_max && p_max < 1.0f)) {
      throw std::invalid_argument(
        "[ROGMap] invalid raycasting probabilities: require "
        "0 < p_min < p_free < 0.5, 0 < p_miss < 0.5, "
        "0.5 < p_hit < 1 and 0.5 < p_occ < p_max < 1.");
    }
    load("raycasting.parallel_enable", parallel_raycast_en);
    load("performance.parallel_raycast_enable", parallel_raycast_en);
    load("raycasting.num_threads", raycast_num_threads);
    load("performance.raycast_num_threads", raycast_num_threads);
    if (batch_update_size <= 0) {
      batch_update_size = 1;
    }
    if (raycast_num_threads <= 0) {
      raycast_num_threads = 1;
    }

    layer_en = true;
    scan_z_min_abs = -0.25;
    scan_z_max_abs = 1.50;
    min_observed_voxels = 2;
    unknown_as_occupied = true;
    surface_height_delta_max = 0.10;
    wall_height_delta_min = 0.20;
    wall_occupancy_ratio_min = 0.80;
    tunnel_height_delta_min = 0.24;
    tunnel_height_delta_max = 0.40;
    tunnel_occupancy_ratio_max = 0.55;
    passable_cost = 50;
    passable_as_free = true;
    layer_hysteresis_en = true;
    layer_hysteresis_count = 2;
    layer_obstacle_hold_time = 0.0;
    layer_mask_filter_en = true;
    layer_fill_occ_min = 5;
    layer_denoise_occ_max = 0;
    load("projection.enable", layer_en);
    load("projection.scan_z_min_abs", scan_z_min_abs);
    load("projection.scan_z_max_abs", scan_z_max_abs);
    load("projection.robot_relative_scan", robot_relative_scan);
    load("projection.scan_z_min_offset", scan_z_min_offset);
    load("projection.scan_z_max_offset", scan_z_max_offset);
    load("terrain.enable", terrain.enable);
    load("terrain.seed_current_footprint", terrain.seed_current_footprint);
    load("terrain.ground_seed_tolerance", terrain.ground_seed_tolerance);
    load("terrain.slope_min_deg", terrain.slope_min_deg);
    load("terrain.slope_max_deg", terrain.slope_max_deg);
    load("terrain.slope_fit_radius", terrain.slope_fit_radius);
    load("terrain.slope_max_residual", terrain.slope_max_residual);
    load("terrain.stair_rise", terrain.stair_rise);
    load("terrain.stair_tread", terrain.stair_tread);
    load("terrain.stair_rise_tolerance", terrain.stair_rise_tolerance);
    load("terrain.stair_half_width", terrain.stair_half_width);
    load("terrain.stair_min_width", terrain.stair_min_width);
    load("terrain.terrain_min_component_area", terrain.terrain_min_component_area);
    load("terrain.obstacle_min_height", terrain.obstacle_min_height);
    load("terrain.support_band_up", terrain.support_band_up);
    load("terrain.support_band_down", terrain.support_band_down);
    load("terrain.drop_probe_depth", terrain.drop_probe_depth);
    load("terrain.min_free_voxels", terrain.min_free_voxels);
    load("terrain.alignment_cosine", terrain.alignment_cosine);
    load("terrain.alignment_weight", terrain.alignment_weight);
    load("terrain.flat_clearance_radius", terrain.flat_clearance_radius);
    load("terrain.terrain_half_length", terrain.terrain_half_length);
    load("terrain.terrain_half_width", terrain.terrain_half_width);
    load("terrain.yaw_independent_footprint", terrain.yaw_independent_footprint);
    load("terrain.clearance_margin", terrain.clearance_margin);
    if (terrain.enable) {
      const double positive[] = {terrain.ground_seed_tolerance,
        terrain.slope_max_deg, terrain.slope_fit_radius, terrain.slope_max_residual,
        terrain.stair_rise, terrain.stair_tread, terrain.stair_rise_tolerance,
        terrain.stair_half_width, terrain.stair_min_width,
        terrain.terrain_min_component_area, terrain.obstacle_min_height,
        terrain.flat_clearance_radius, terrain.terrain_half_length, terrain.terrain_half_width,
        terrain.alignment_weight};
      for (const double value : positive) {
        if (!std::isfinite(value) || value <= 0.0) {
          throw std::invalid_argument("online terrain dimensions/tolerances must be finite and positive.");
        }
      }
      if (!std::isfinite(terrain.slope_min_deg) || terrain.slope_min_deg < 0.0 ||
          terrain.slope_min_deg >= terrain.slope_max_deg || terrain.slope_max_deg >= 90.0 ||
          !std::isfinite(terrain.alignment_cosine) || terrain.alignment_cosine <= 0.0 ||
          terrain.alignment_cosine > 1.0 || !layer_en || !robot_relative_scan) {
        throw std::invalid_argument("online terrain requires projection, robot-relative scan and valid slope/alignment limits.");
      }
      if (!std::isfinite(terrain.clearance_margin) || terrain.clearance_margin < 0.0) {
        throw std::invalid_argument("terrain.clearance_margin must be finite and non-negative.");
      }
    }
    if (robot_relative_scan &&
        (!std::isfinite(scan_z_min_offset) || !std::isfinite(scan_z_max_offset) ||
         scan_z_max_offset <= scan_z_min_offset)) {
      throw std::invalid_argument(
        "robot-relative projection requires finite ordered scan_z_min_offset/scan_z_max_offset.");
    }
    load("projection.min_observed_voxels", min_observed_voxels);
    load("projection.unknown_as_occupied", unknown_as_occupied);
    load("projection.surface_height_delta_max", surface_height_delta_max);
    load("projection.wall_height_delta_min", wall_height_delta_min);
    load("projection.wall_occupancy_ratio_min", wall_occupancy_ratio_min);
    load("projection.tunnel_height_delta_min", tunnel_height_delta_min);
    load("projection.tunnel_height_delta_max", tunnel_height_delta_max);
    load("projection.tunnel_occupancy_ratio_max", tunnel_occupancy_ratio_max);
    load("projection.passable_cost", passable_cost);
    load("projection.passable_as_free", passable_as_free);
    load("projection.hysteresis_enable", layer_hysteresis_en);
    load("projection.hysteresis_count", layer_hysteresis_count);
    load("projection.obstacle_hold_time", layer_obstacle_hold_time);
    layer_obstacle_hold_time = std::max(0.0, layer_obstacle_hold_time);
    load("projection.mask_filter_en", layer_mask_filter_en);
    load("projection.fill_occ_min", layer_fill_occ_min);
    load("projection.denoise_occ_max", layer_denoise_occ_max);
    prior_map_enable = false;
    prior_map_yaml_path.clear();
    prior_map_pgm_path.clear();
    prior_map_frame = "map";
    load("projection.prior_map.enable", prior_map_enable);
    load("projection.prior_map.yaml_path", prior_map_yaml_path);
    load("projection.prior_map.pgm_path", prior_map_pgm_path);
    load("projection.prior_map.frame_id", prior_map_frame);
    if (prior_map_enable && prior_map_yaml_path.empty()) {
      throw std::invalid_argument(
        "projection.prior_map.yaml_path must not be empty when prior map fusion is enabled.");
    }
    if (prior_map_enable && prior_map_frame.empty()) {
      throw std::invalid_argument(
        "projection.prior_map.frame_id must not be empty when prior map fusion is enabled.");
    }
    if (layer_fill_occ_min < 1 || layer_fill_occ_min > 8 ||
        layer_denoise_occ_max < 0 || layer_denoise_occ_max > 7 ||
        layer_denoise_occ_max >= layer_fill_occ_min) {
      throw std::invalid_argument(
        "projection mask filter requires fill_occ_min in [1, 8], "
        "denoise_occ_max in [0, 7], and denoise_occ_max < fill_occ_min.");
    }
    if (!std::isfinite(scan_z_min_abs) || !std::isfinite(scan_z_max_abs) ||
        scan_z_max_abs <= scan_z_min_abs) {
      throw std::invalid_argument(
        "projection.scan_z_min_abs and projection.scan_z_max_abs must be finite absolute Z "
        "coordinates in the ROGMap frame, and scan_z_max_abs must be greater than scan_z_min_abs.");
    }
    if (!std::isfinite(surface_height_delta_max) || surface_height_delta_max < 0.0) {
      throw std::invalid_argument(
        "projection.surface_height_delta_max must be a non-negative vertical height difference.");
    }
    if (!std::isfinite(wall_height_delta_min) || wall_height_delta_min <= surface_height_delta_max) {
      throw std::invalid_argument(
        "projection.wall_height_delta_min must be a vertical height difference greater than "
        "surface_height_delta_max.");
    }
    if (!std::isfinite(wall_occupancy_ratio_min) || wall_occupancy_ratio_min < 0.0 ||
        wall_occupancy_ratio_min > 1.0) {
      throw std::invalid_argument(
        "projection.wall_occupancy_ratio_min must be a dimensionless ratio in [0, 1].");
    }
    if (!std::isfinite(tunnel_occupancy_ratio_max) || tunnel_occupancy_ratio_max < 0.0 ||
        tunnel_occupancy_ratio_max > 1.0) {
      throw std::invalid_argument(
        "projection.tunnel_occupancy_ratio_max must be a dimensionless ratio in [0, 1].");
    }
    if (tunnel_occupancy_ratio_max >= wall_occupancy_ratio_min) {
      throw std::invalid_argument(
        "projection.tunnel_occupancy_ratio_max must be a dimensionless ratio smaller than "
        "wall_occupancy_ratio_min.");
    }
    if (!std::isfinite(tunnel_height_delta_min) || tunnel_height_delta_min <= surface_height_delta_max) {
      throw std::invalid_argument(
        "projection.tunnel_height_delta_min must be a vertical height difference greater than "
        "surface_height_delta_max.");
    }
    if (!std::isfinite(tunnel_height_delta_max) || tunnel_height_delta_max < tunnel_height_delta_min) {
      throw std::invalid_argument(
        "projection.tunnel_height_delta_max must be a vertical height difference greater than or equal "
        "to tunnel_height_delta_min.");
    }

    // 全局二维地形图：世界系固定，由局部观测增量更新，是 planner 实际查询的地图。
    // 缺省 40x40 m、0.05 m，世界原点在地图中心。
    global_map = GlobalTerrainConfig{};
    load("global_map.enable", global_map.enable);
    load("global_map.resolution", global_map.resolution);
    load("global_map.size_x", global_map.size_x);
    load("global_map.size_y", global_map.size_y);
    load("global_map.origin_x", global_map.origin_x);
    load("global_map.origin_y", global_map.origin_y);
    load("global_map.slope_max_deg", global_map.slope_max_deg);
    load("global_map.slope_min_deg", global_map.slope_min_deg);
    load("global_map.field_enable", global_map.field_enable);
    load("global_map.field_max_distance", global_map.field_max_distance);
    load("global_map.field_min_distance", global_map.field_min_distance);
    load("global_map.field_clamp_distance", global_map.field_clamp_distance);
    load("global_map.field_update_period_sec", global_map.field_update_period_sec);
    load("global_map.void_gain", global_map.void_gain);
    load("global_map.void_confirm", global_map.void_confirm);
    // support/drop band 参数落在 terrain.* 下（见 YAML），但由 GlobalTerrainGrid 使用。
    global_map.support_band_up = terrain.support_band_up;
    global_map.support_band_down = terrain.support_band_down;
    global_map.drop_probe_depth = terrain.drop_probe_depth;
    global_map.min_free_voxels = terrain.min_free_voxels;
    // 车身净空口径必须和局部地形层完全一致，否则 A*/MINCO/validator 会互相打架。
    global_map.flat_clearance_radius = terrain.flat_clearance_radius;
    global_map.terrain_half_length = terrain.terrain_half_length;
    global_map.terrain_half_width = terrain.terrain_half_width;
    global_map.yaw_independent_footprint = terrain.yaw_independent_footprint;
    global_map.clearance_margin = terrain.clearance_margin;
    global_map.alignment_cosine = terrain.alignment_cosine;
    global_map.alignment_weight = terrain.alignment_weight;
    if (global_map.enable &&
        (!std::isfinite(global_map.resolution) || global_map.resolution <= 0.0 ||
         !std::isfinite(global_map.size_x) || global_map.size_x <= 0.0 ||
         !std::isfinite(global_map.size_y) || global_map.size_y <= 0.0)) {
      throw std::invalid_argument(
        "rog_map.global_map requires finite positive resolution/size_x/size_y.");
    }
    if (global_map.enable && !terrain.enable) {
      RCLCPP_WARN(node->get_logger(),
        "[ROGMap Config] global_map.enable=true 但 terrain.enable=false：全局图拿不到支撑面，"
        "将没有坡道/边缘语义。");
    }

    field_en = true;
    field_inflation_radius = 0.33;
    field_max_distance = 3.0;
    field_min_distance = -1.0;
    field_clamp_distance_en = true;
    field_interpolation = "bilinear";
    field_update_rate = 20.0;
    load("field.enable", field_en);
    load("field.inflation_radius", field_inflation_radius);
    load("field.max_distance", field_max_distance);
    load("field.min_distance", field_min_distance);
    load("field.clamp_distance", field_clamp_distance_en);
    load("field.interpolation", field_interpolation);
    load("field.update_rate", field_update_rate);
    load("performance.field_update_rate", field_update_rate);
    if (field_max_distance <= 0.0) {
      field_max_distance = 3.0;
    }
    if (field_min_distance > 0.0) {
      field_min_distance = -1.0;
    }

    decay_en = true;
    keep_time = 0.4;
    decay_clear_time = 1.0;
    decay_active_list_en = true;
    dirty_column_en = false;
    dirty_full_ratio = 0.30;
    performance_enable = true;
    performance_detailed_csv_enable = false;
    performance_detailed_csv_path = "/tmp/rog_map_perf_detailed.csv";
    performance_summary_csv_enable = false;
    performance_summary_csv_path = "/tmp/rog_map_perf_summary.csv";
    performance_run_id = "";
    performance_scenario = "";
    performance_variant = "";
    performance_csv_flush_every_n = 30;
    performance_summary_rate = 1.0;
    debug_layer_pub_en = true;
    debug_field_pub_en = true;
    debug_pub_rate = 5.0;
    load("decay.enable", decay_en);
    load("decay.keep_time", keep_time);
    load("decay.clear_time", decay_clear_time);
    if (decay_en && (!std::isfinite(keep_time) || keep_time < 0.0 ||
        !std::isfinite(decay_clear_time) || decay_clear_time <= keep_time)) {
      throw std::runtime_error(
        "[ROGMap] enabled decay requires finite clear_time > keep_time >= 0");
    }
    load("decay.active_list_enable", decay_active_list_en);
    load("performance.dirty_column_enable", dirty_column_en);
    load("performance.dirty_full_ratio", dirty_full_ratio);
    load("performance.enable", performance_enable);
    load("performance.detailed_csv_enable", performance_detailed_csv_enable);
    load("performance.detailed_csv_path", performance_detailed_csv_path);
    load("performance.summary_csv_enable", performance_summary_csv_enable);
    load("performance.summary_csv_path", performance_summary_csv_path);
    load("performance.run_id", performance_run_id);
    load("performance.scenario", performance_scenario);
    load("performance.variant", performance_variant);
    load("performance.csv_flush_every_n", performance_csv_flush_every_n);
    load("performance.summary_rate", performance_summary_rate);
    load("debug.layer_pub_enable", debug_layer_pub_en);
    load("debug.field_pub_enable", debug_field_pub_en);
    load("debug.pub_rate", debug_pub_rate);

    auto ray_range = loadVec2("raycasting.ray_range", vector<double>{0.3, 10.0});
    raycast_range_min = ray_range[0];
    raycast_range_max = ray_range[1];
    sqr_raycast_range_max = raycast_range_max * raycast_range_max;
    sqr_raycast_range_min = raycast_range_min * raycast_range_min;
    local_update_box_d = loadVec3("raycasting.local_update_box", vector<double>{999.0, 999.0, 999.0});

    virtual_ground_height = -0.80;
    virtual_ceil_height = 1.80;
    load("virtual_ground_height", virtual_ground_height);
    load("virtual_ceil_height", virtual_ceil_height);
    if (virtual_ground_height >= virtual_ceil_height) {
      RCLCPP_WARN(node->get_logger(),
        "[ROGMap Config] invalid virtual height range [%.3f, %.3f], reset to [-0.80, 1.80].",
        virtual_ground_height,
        virtual_ceil_height);
      virtual_ground_height = -0.80;
      virtual_ceil_height = 1.80;
    }

    resetMapSize();

#define logit(x) (log((x) / (1 - (x))))
    l_hit = logit(p_hit);
    l_miss = logit(p_miss);
    l_min = logit(p_min);
    l_max = logit(p_max);
    l_occ = logit(p_occ);
    l_free = logit(p_free);
    inf_spherical_neighbor.clear();
    unk_inf_spherical_neighbor.clear();
    spherical_neighbor.clear();
    for (int dx = -inflation_step; dx <= inflation_step; dx++) {
      for (int dy = -inflation_step; dy <= inflation_step; dy++) {
        for (int dz = -inflation_step; dz <= inflation_step; dz++) {
          if (inflation_step == 1 || dx * dx + dy * dy + dz * dz <= inflation_step * inflation_step) {
            inf_spherical_neighbor.emplace_back(dx, dy, dz);
          }
        }
      }
    }
    std::sort(
      inf_spherical_neighbor.begin(), inf_spherical_neighbor.end(), [](const Vec3i & a, const Vec3i & b) {
        return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() <
               b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
      });

    if (unk_inflation_en) {
      for (int dx = -unk_inflation_step; dx <= unk_inflation_step; dx++) {
        for (int dy = -unk_inflation_step; dy <= unk_inflation_step; dy++) {
          for (int dz = -unk_inflation_step; dz <= unk_inflation_step; dz++) {
            if (unk_inflation_step == 1 ||
                dx * dx + dy * dy + dz * dz <= unk_inflation_step * unk_inflation_step) {
              unk_inf_spherical_neighbor.emplace_back(dx, dy, dz);
            }
          }
        }
      }
      std::sort(unk_inf_spherical_neighbor.begin(),
        unk_inf_spherical_neighbor.end(),
        [](const Vec3i & a, const Vec3i & b) {
          return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() <
                 b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
        });
    }

    constexpr double max_search_dis = 5.0;
    const int max_seach_step = ceil(max_search_dis / resolution);
    for (int dx = -max_seach_step; dx <= max_seach_step; dx++) {
      for (int dy = -max_seach_step; dy <= max_seach_step; dy++) {
        for (int dz = -max_seach_step; dz <= max_seach_step; dz++) {
          if (dx * dx + dy * dy + dz * dz <= max_seach_step * max_seach_step) {
            spherical_neighbor.emplace_back(dx, dy, dz);
          }
        }
      }
    }
    std::sort(spherical_neighbor.begin(), spherical_neighbor.end(), [](const Vec3i & a, const Vec3i & b) {
      return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() < b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
    });
    RCLCPP_INFO(node->get_logger(),
      "[ROGMap Config] loaded prefix='%s', frame_id='%s', cloud='%s', "
      "odom='%s', resolution=%.3f, "
      "map_size=[%.2f %.2f %.2f], projection=%s, field=%s/%s, performance=%s, visualization=%s",
      prefix.c_str(),
      frame_id.c_str(),
      cloud_topic.c_str(),
      odom_topic.c_str(),
      resolution,
      map_size_d.x(),
      map_size_d.y(),
      map_size_d.z(),
      layer_en ? "on" : "off",
      field_en ? "on" : "off",
      field_interpolation.c_str(),
      performance_enable ? "on" : "off",
      visualization_en ? "on" : "off");
  }

  // add 24.07.18 add esdf
  bool esdf_en{false};
  Vec3f esdf_local_update_box{};
  double esdf_resolution{};

  bool layer_en{true};
  double scan_z_min_abs{-0.25};
  double scan_z_max_abs{1.50};
  // Opt-in: offsets from the LIO/IMU origin, along gravity-aligned world Z.
  bool robot_relative_scan{false};
  double scan_z_min_offset{-1.0};
  double scan_z_max_offset{1.5};
  TerrainConfig terrain;
  GlobalTerrainConfig global_map;
  int min_observed_voxels{2};
  bool unknown_as_occupied{true};
  double surface_height_delta_max{0.10};
  double wall_height_delta_min{0.20};
  double wall_occupancy_ratio_min{0.80};
  double tunnel_height_delta_min{0.24};
  double tunnel_height_delta_max{0.40};
  double tunnel_occupancy_ratio_max{0.55};
  int passable_cost{50};
  bool layer_hysteresis_en{true};
  int layer_hysteresis_count{2};
  double layer_obstacle_hold_time{0.0};
  bool layer_mask_filter_en{true};
  int layer_fill_occ_min{5};
  int layer_denoise_occ_max{0};
  bool passable_as_free{true};
  bool prior_map_enable{false};
  string prior_map_yaml_path{};
  string prior_map_pgm_path{};
  string prior_map_frame{"map"};

  Vec3f sensor_in_robot{Vec3f::Zero()};
  Eigen::Matrix3d sensor_in_robot_R{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond sensor_in_robot_q{Eigen::Quaterniond::Identity()};
  bool cloud_filter_en{false};
  bool cloud_filter_z_filter_enable{true};
  double cloud_filter_z_offset{0.0};
  string cloud_filter_position_frame{"map"};
  string cloud_filter_mode{"transform_center"};
  bool cloud_filter_remove_inside{true};
  // 自车附近诊断点云：把进入地图前后的点与最终被判成障碍的格子分别发出来。
  bool debug_clouds_enable{false};
  bool debug_pipeline_enable{false};
  double debug_clouds_half_extent_xy{0.6};
  double debug_clouds_min_z{-0.5};
  double debug_clouds_max_z{1.0};
  bool cloud_filter_log_stats{false};
  int cloud_filter_stats_log_period_ms{1000};
  bool cloud_filter_publish_visualization{false};
  string cloud_filter_visualization_topic{"/rog_map/cloud_filter/markers"};
  int cloud_filter_visualization_period_ms{200};
  double cloud_filter_z_plane_size{12.0};
  double cloud_filter_z_plane_thickness{0.03};
  double cloud_filter_box_padding{0.0};
  double cloud_filter_center_x{0.0};
  double cloud_filter_center_y{0.0};
  double cloud_filter_center_z{0.0};
  double cloud_filter_size_x{12.0};
  double cloud_filter_size_y{6.0};
  double cloud_filter_size_z{3.5};
  vector<double> cloud_filter_centers_x{};
  vector<double> cloud_filter_centers_y{};
  vector<double> cloud_filter_centers_z{};
  vector<double> cloud_filter_sizes_x{};
  vector<double> cloud_filter_sizes_y{};
  vector<double> cloud_filter_sizes_z{};

  bool field_en{true};
  double field_inflation_radius{0.33};
  double field_max_distance{3.0};
  double field_min_distance{-1.0};
  bool field_clamp_distance_en{true};
  string field_interpolation{"bilinear"};

  bool decay_en{true};
  double keep_time{0.4};
  double decay_clear_time{1.0};
  bool decay_active_list_en{true};

  bool parallel_raycast_en{true};
  int raycast_num_threads{4};
  bool dirty_column_en{false};
  double dirty_full_ratio{0.30};
  double field_update_rate{20.0};
  bool performance_enable{true};
  bool performance_detailed_csv_enable{false};
  string performance_detailed_csv_path{"/tmp/rog_map_perf_detailed.csv"};
  bool performance_summary_csv_enable{false};
  string performance_summary_csv_path{"/tmp/rog_map_perf_summary.csv"};
  string performance_run_id{};
  string performance_scenario{};
  string performance_variant{};
  int performance_csv_flush_every_n{30};
  double performance_summary_rate{1.0};
  bool debug_layer_pub_en{true};
  bool debug_field_pub_en{true};
  double debug_pub_rate{5.0};

  bool load_pcd_en{false};
  string pcd_name{"map.pcd"};

  double resolution{}, inflation_resolution{};
  int inflation_step{};
  Vec3f local_update_box_d, half_local_update_box_d{};
  Vec3i local_update_box_i, half_local_update_box_i{};
  Vec3f map_size_d, half_map_size_d{};
  Vec3i inf_half_map_size_i{}, half_map_size_i{}, fro_half_map_size_i{};

  /* The inf map virtual ceil should consider the inflation step
   *  the float type ceil and ground height is remap to the lower and upper bound of
   *  the inflation layer respectively
   */
  double virtual_ceil_height{}, virtual_ground_height{};
  int inf_virtual_ceil_height_id_g{}, inf_virtual_ground_height_id_g{};

  bool visualization_en{false}, frontier_extraction_en{false}, raycasting_en{true}, ros_callback_en{false};

  /* Spherical neighbor for inflation*/
  std::vector<Vec3i> inf_spherical_neighbor{};
  std::vector<Vec3i> unk_inf_spherical_neighbor{};
  /* Spherical neighbor for nearest search within x m*/
  std::vector<Vec3i> spherical_neighbor{};

  /* intensity noise filter*/
  int intensity_thresh{};
  /* aster properties */
  string frame_id{};
  bool map_sliding_en{true};
  bool map_center_offset_enable{false};
  Vec3f map_center_offset{Vec3f::Zero()};
  Vec3f fix_map_origin{};
  string odom_topic{}, cloud_topic{};
  string robot_frame{"base_link"};
  int update_period_ms{1};
  /* probability update */
  double raycast_range_min{}, raycast_range_max{};
  double sqr_raycast_range_min{}, sqr_raycast_range_max{};
  int point_filt_num{}, batch_update_size{};
  float p_hit{}, p_miss{}, p_min{}, p_max{}, p_occ{}, p_free{};
  double odom_buffer_sec{2.0}, max_odom_gap_sec{0.05}, max_stamp_age_sec{2.0};
  int cloud_wait_timeout_ms{30};
  float l_hit{}, l_miss{}, l_min{}, l_max{}, l_occ{}, l_free{};

  /* for unknown inflation */
  bool unk_inflation_en{false};
  int unk_inflation_step{0};

  double odom_timeout{};
  Vec3f visualization_range{};
  string visualization_frame_id{};
  double visualization_rate{};
  bool global_map_viz_en{true};
  double global_map_viz_rate{1.0};

  double unk_thresh{};
  double map_sliding_thresh{};

  void resetMapSize()
  {
    int inflation_ratio = ceil(inflation_resolution / resolution);

#ifdef ORIGIN_AT_CENTER
    // When discretized in such a way that the origin is at cell center
    // the inflation ratio should be an odd number
    if (inflation_ratio % 2 == 0) {
      inflation_ratio += 1;
    }
    std::cout << YELLOW << " -- [RM-Config] inflation_ratio: " << inflation_ratio << std::endl;
#endif

    inflation_resolution = resolution * inflation_ratio;
    std::cout << color_text::GREEN << " -- [RM-Config] inflation_resolution: " << inflation_resolution
              << color_text::RESET << std::endl;

    half_map_size_d = map_size_d / 2;

    // Size_d is only for calculate index number.
    // 1) we calculate the index number of the inf map
    int max_step = 0;
    if (!unk_inflation_en) {
      max_step = inflation_step;
    } else {
      max_step = std::max(inflation_step, unk_inflation_step);
    }
    inf_half_map_size_i =
      (half_map_size_d / inflation_resolution).cast<int>() + (max_step + 1) * Vec3i::Ones();

    // 2) we calculate the index number of the prob map, which should be smaller than infmap
    half_map_size_i = (inf_half_map_size_i - (max_step + 1) * Vec3i::Ones()) * inflation_ratio;

    // 3) compute the frontier counter map size
    if (frontier_extraction_en) {
      fro_half_map_size_i = half_map_size_i + Vec3i::Constant(1);
    }

    // 4) re-compute the map_size_d, the map_size_d is not that important.
    map_size_d = (half_map_size_i * 2 + Vec3i::Constant(1)).cast<double>() * resolution;
    half_map_size_d = map_size_d / 2;

    // 5) compute the index of raycasting update box
    half_local_update_box_d = local_update_box_d / 2;
    half_local_update_box_i = (half_local_update_box_d / resolution).cast<int>();
    local_update_box_i = half_local_update_box_i * 2 + Vec3i::Constant(1);
    local_update_box_d = local_update_box_i.cast<double>() * resolution;

    // 6) reset the virtual ground and ceil size
#ifdef ORIGIN_AT_CENTER
    inf_virtual_ceil_height_id_g =
      static_cast<int>(virtual_ceil_height / inflation_resolution + SIGN(virtual_ceil_height) * 0.5);
    inf_virtual_ground_height_id_g =
      static_cast<int>(virtual_ground_height / inflation_resolution + SIGN(virtual_ground_height) * 0.5);
    fmt::print(" -- [ROG-Map] Init, resetMapSize: ORIGIN_AT_CENTER.\n");
#endif

#ifdef ORIGIN_AT_CORNER
    inf_virtual_ceil_height_id_g =
      static_cast<int>(floor(virtual_ceil_height / inflation_resolution)) - inflation_step;
    inf_virtual_ground_height_id_g =
      static_cast<int>(floor(virtual_ground_height / inflation_resolution)) + inflation_step;

    fmt::print(" -- [ROG-Map] Init, resetMapSize: ORIGIN_AT_CORNER.\n");
#endif

    virtual_ceil_height = inf_virtual_ceil_height_id_g * inflation_resolution - 0.5 * inflation_resolution;
    virtual_ground_height =
      inf_virtual_ground_height_id_g * inflation_resolution + 0.5 * inflation_resolution;
    fmt::print(" -- [ROG-Map] Init, resetMapSize: virtual_ceil_height: {}, virtual_ground_height: {}.\n",
      virtual_ceil_height,
      virtual_ground_height);
  }
};
}  // namespace rog_map
