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

#include <cstddef>
#include <limits>
#include <cstdint>
#include <vector>

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <rog_map/rog_map_visualizer.hpp>
#include <rog_map_ros/cloud_registered_crop_filter.hpp>
#include <rog_map_ros/callback_gate.hpp>
#include <rog_map_ros/odometry_buffer.hpp>
#include <super_utils/color_msg_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

namespace rog_map {
using namespace super_utils;

class ROGMapROS : public ROGMap
{
  rclcpp::Node::SharedPtr nh_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_nh_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<ROGMapVisualizer> visualizer_driver_;
  std::unique_ptr<CloudRegisteredCropFilter> cloud_filter_;

  const double getSystemWalltimeNow() override { return now().seconds(); }

  void getSystemWalltimeNow(rclcpp::Time & _in) { _in = now(); };

  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  bool getPriorMapTransform(PriorMapTransform2D & transform) override
  {
    if (cfg_.prior_map_frame == cfg_.frame_id) {
      transform = PriorMapTransform2D{};
      return true;
    }
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF is unavailable because the shared TF buffer is null");
      return false;
    }
    try {
      // lookupTransform(target, source, ...) returns T_target_source.
      const auto tf_msg = tf_->lookupTransform(cfg_.prior_map_frame, cfg_.frame_id, tf2::TimePointZero);
      transform.tx = tf_msg.transform.translation.x;
      transform.ty = tf_msg.transform.translation.y;
      const auto & q = tf_msg.transform.rotation;
      transform.yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      if (std::isfinite(transform.tx) && std::isfinite(transform.ty) &&
          std::isfinite(transform.yaw)) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF from '%s' to '%s' contains a non-finite 2D transform",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str());
      return false;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] cannot transform projection from '%s' to prior map frame '%s': %s",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str(),
        error.what());
      return false;
    }
  }

  template <typename NodeT> void bindNode(NodeT node)
  {
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_timers_ = node->get_node_timers_interface();
    node_clock_ = node->get_node_clock_interface();
    node_logging_ = node->get_node_logging_interface();
    node_parameters_ = node->get_node_parameters_interface();
  }

  rclcpp::CallbackGroup::SharedPtr createCallbackGroup(rclcpp::CallbackGroupType type)
  {
    return node_base_->create_callback_group(type);
  }

  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr createPublisher(
    const std::string & topic, const rclcpp::QoS & qos)
  {
    return rclcpp::create_publisher<MsgT>(node_parameters_, node_topics_, topic, qos);
  }

  template <typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr createSubscription(const std::string & topic,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
  {
    return rclcpp::create_subscription<MsgT>(
      node_parameters_, node_topics_, topic, qos, std::forward<CallbackT>(callback), options);
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr createWallTimer(std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return rclcpp::create_wall_timer(
      period, std::forward<CallbackT>(callback), group, node_base_.get(), node_timers_.get());
  }

  std::shared_ptr<CallbackGate> callback_gate_{std::make_shared<CallbackGate>()};
  std::mutex map_access_mutex_;
  std::mutex odom_header_mutex_;
  std::condition_variable odom_ready_;
  OdometryBuffer odom_buffer_;
  std::chrono::steady_clock::time_point last_odom_received_{};
  double last_measurement_stamp_{-std::numeric_limits<double>::infinity()};
  ROGMapVisualizer::Publishers vm_;
  std::chrono::steady_clock::time_point last_global_map_viz_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr actual_input_pub_;

  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    sensor_msgs::msg::PointCloud2::UniquePtr pending_cloud;
    rclcpp::TimerBase::SharedPtr update_timer;
    std::chrono::steady_clock::time_point cloud_received;
    mutex updete_lock;
  } rc_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    if (performance_monitor_) {
      performance_monitor_->recordOdom(now().seconds());
    }
    std::lock_guard<std::mutex> odom_lock(odom_header_mutex_);
    const Pose pose{Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z), Quatf(odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)};
    if (odom_msg->header.frame_id != cfg_.frame_id ||
        !odom_buffer_.insert(rclcpp::Time(odom_msg->header.stamp).seconds(), pose)) { return; }
    last_odom_received_ = std::chrono::steady_clock::now();
    updateRobotState(pose);
    odom_ready_.notify_all();
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg)
  {
    const double stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
    const double processing = now().seconds();
    if (performance_monitor_) {
      performance_monitor_->recordCloudCallback(processing, cloud_msg->width * cloud_msg->height,
        std::max(0.0, processing - stamp) * 1000.0, 0.0);
    }
    if (cloud_msg->header.frame_id != cfg_.frame_id || stamp <= 0 ||
        std::abs(processing - stamp) > cfg_.max_stamp_age_sec) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(), *node_clock_->get_clock(), 2000,
        "[ROG sync] drop frame/clock mismatch: frame=%s expected=%s cloud=%.9f ROS=%.9f "
        "delta=%.3fs; historical bags require --clock and use_sim_time=true",
        cloud_msg->header.frame_id.c_str(), cfg_.frame_id.c_str(), stamp, processing, processing - stamp);
      return;
    }
    std::lock_guard<std::mutex> lock(rc_.updete_lock);
    rc_.pending_cloud = std::move(cloud_msg);
    rc_.cloud_received = std::chrono::steady_clock::now();
  }

  bool cloudTimePoseFromTf(const std_msgs::msg::Header & header, Pose & pose)
  {
    if (!tf_ || cfg_.robot_frame.empty() || cfg_.robot_frame == cfg_.frame_id) { return false; }
    try {
      // Point-LIO publishes a scan-end TF even when the 200 Hz odometry rate limiter
      // leaves its last sample a few milliseconds before the registered cloud stamp.
      // Request that exact time; never substitute the latest TF or extrapolate here.
      const auto transform = tf_->lookupTransform(cfg_.frame_id, cfg_.robot_frame,
        rclcpp::Time(header.stamp));
      const auto & t = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      Pose candidate{Vec3f(t.x, t.y, t.z), Quatf(q.w, q.x, q.y, q.z)};
      if (!candidate.first.allFinite() || !candidate.second.coeffs().allFinite() ||
          candidate.second.norm() < 1e-6) { return false; }
      candidate.second.normalize();
      pose = candidate;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void updateCallback()
  {
    const auto work_start = std::chrono::steady_clock::now();
    sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg;
    PointCloud temp_pc;
    Pose temp_pose;
    std_msgs::msg::Header input_header;
    double temp_odom_age_ms = 0, cloud_wait_ms = 0;
    {
      std::lock_guard<std::mutex> lock(rc_.updete_lock);
      if (!rc_.pending_cloud) { return; }
      cloud_msg = std::move(rc_.pending_cloud);
      cloud_wait_ms = std::chrono::duration<double, std::milli>(work_start - rc_.cloud_received).count();
    }

    input_header = cloud_msg->header;
    const double stamp = rclcpp::Time(input_header.stamp).seconds();
    bool synced = false;
    {
      std::unique_lock<std::mutex> odom_lock(odom_header_mutex_);
      temp_odom_age_ms = std::chrono::duration<double, std::milli>(work_start - last_odom_received_).count();
      synced = temp_odom_age_ms <= cfg_.odom_timeout * 1000 &&
        odom_buffer_.interpolate(stamp, temp_pose, cfg_.max_odom_gap_sec);
      if (!synced && temp_odom_age_ms <= cfg_.odom_timeout * 1000 &&
          cloudTimePoseFromTf(input_header, temp_pose)) {
        synced = true;
      }
      if (!synced) {
        const double remaining = cfg_.cloud_wait_timeout_ms - cloud_wait_ms;
        if (remaining > 0) {
          synced = odom_ready_.wait_for(odom_lock, std::chrono::duration<double, std::milli>(remaining), [&] {
            temp_odom_age_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - last_odom_received_).count();
            return temp_odom_age_ms <= cfg_.odom_timeout * 1000 &&
              odom_buffer_.interpolate(stamp, temp_pose, cfg_.max_odom_gap_sec);
          });
        }
      }
      if (!synced && temp_odom_age_ms <= cfg_.odom_timeout * 1000 &&
          cloudTimePoseFromTf(input_header, temp_pose)) {
        synced = true;
      }
    }
    if (!synced || stamp <= last_measurement_stamp_) {
      return;
    }

    // Conversion belongs to the map worker. The subscription callback only replaces
    // the single pending ROS message, so newer clouds can arrive while this frame waits for pose data.
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    if (temp_pc.empty()) { return; }

    std::lock_guard<std::mutex> map_lock(map_access_mutex_);
    if (cfg_.debug_clouds_enable) { publishDebugCloud(vm_.debug_raw_pub, temp_pc, temp_pose.first); }
    if (cloud_filter_ && !cloud_filter_->filter(temp_pc, input_header, temp_pose.first)) {
      return;
    }
    if (cfg_.debug_clouds_enable) { publishDebugCloud(vm_.debug_filtered_pub, temp_pc, temp_pose.first); }
    if (temp_pc.empty()) { return; }
    if (performance_monitor_) {
      performance_monitor_->recordValidCloud(temp_odom_age_ms);
    }
    if (cfg_.debug_clouds_enable || cfg_.debug_pipeline_enable) {
      // Publish the exact immutable cloud passed below, after latest-frame selection.
      // No ROI crop, frame substitution, timestamp substitution or extra filtering.
      sensor_msgs::msg::PointCloud2 actual;
      pcl::toROSMsg(temp_pc, actual);
      actual.header = input_header;
      actual_input_pub_->publish(actual);
    }
    const double measurement_stamp = rclcpp::Time(input_header.stamp).seconds();
    updateMapInternal(temp_pc, temp_pose, measurement_stamp);
    last_measurement_stamp_ = measurement_stamp;
    if (cfg_.debug_clouds_enable) {
      publishDebugObstacleCells(temp_pose.first);
    }
  }

  void vizCallback()
  {
    std::lock_guard<std::mutex> lock(map_access_mutex_);
    if (!cfg_.visualization_en) {
      return;
    }
    if (map_empty_) {
      return;
    }
    if (!hasVisualizationSubscriber()) {
      return;
    }

    const RobotState robot_state = getRobotState();
    Vec3f box_max = robot_state.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state.p - cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
      cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << endl;
      return;
    }

    if (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> unknown_map;
      boxSearch(box_min, box_max, UNKNOWN, unknown_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(unknown_map, cloud_msg);
      vm_.unknown_pub->publish(cloud_msg);
    }
    if (cfg_.unk_inflation_en && vm_.unknown_inf_pub &&
        vm_.unknown_inf_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> inf_unknown_map;
      boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(inf_unknown_map, cloud_msg);
      vm_.unknown_inf_pub->publish(cloud_msg);
    }

    if (layer_ && !layer_->empty()) {
      if (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(fused_projection_mask_, grid);
        vm_.layer_value_pub->publish(grid);
      }
      if (vm_.layer_value_dynamic_pub &&
          vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(layer_->mask(), grid);
        vm_.layer_value_dynamic_pub->publish(grid);
      }
      if (vm_.layer_value_static_pub &&
          vm_.layer_value_static_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(prior_projection_mask_, grid);
        vm_.layer_value_static_pub->publish(grid);
      }
      if (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> types(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          switch (layer_->cells()[i].type) {
          case CellType::UNKNOWN:
            types[i] = 0U;
            break;
          case CellType::FREE:
            types[i] = 33U;
            break;
          case CellType::PASSABLE:
            types[i] = 66U;
            break;
          case CellType::OCCUPIED:
            types[i] = 100U;
            break;
          }
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(types, grid);
        vm_.layer_type_pub->publish(grid);
      }
      if (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> confidence(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          confidence[i] = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::round(layer_->cells()[i].confidence * 100.0f)), 0, 100));
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(confidence, grid);
        vm_.layer_confidence_pub->publish(grid);
      }
      if (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) {
        sensor_msgs::msg::PointCloud2 height_delta_cloud;
        fillLayerHeightDeltaCloud(height_delta_cloud);
        vm_.layer_height_delta_pub->publish(height_delta_cloud);
      }
    }

    if (field_ && field_->isValid() && vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) {
      sensor_msgs::msg::PointCloud2 field_cloud;
      fillFieldCloud(field_cloud);
      vm_.field_pub->publish(field_cloud);
    }

    // 全局二维地形图（世界固定）：只在有订阅者时发布，并按 global_map_viz_rate 限频。
    // 两个话题：nav_grid（可通行状态）+ blocked_edges（平台/坡道边缘）。
    {
      const bool want_grid =
        vm_.global_nav_grid_pub && vm_.global_nav_grid_pub->get_subscription_count() >= 1;
      const bool want_edges =
        vm_.global_blocked_edges_pub && vm_.global_blocked_edges_pub->get_subscription_count() >= 1;
      if (global_grid_ && (want_grid || want_edges)) {
        const auto now_steady = std::chrono::steady_clock::now();
        if (last_global_map_viz_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now_steady - last_global_map_viz_).count() >=
              1.0 / cfg_.global_map_viz_rate) {
          last_global_map_viz_ = now_steady;
          if (want_grid) {
            nav_msgs::msg::OccupancyGrid grid;
            fillGlobalNavGrid(grid);
            vm_.global_nav_grid_pub->publish(grid);
          }
          if (want_edges) {
            visualization_msgs::msg::MarkerArray markers;
            fillGlobalBlockedEdges(markers);
            vm_.global_blocked_edges_pub->publish(markers);
          }
        }
      }
    }
    if (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) {
      sensor_msgs::msg::PointCloud2 decay_cloud;
      fillDecayCellsCloud(decay_cloud);
      vm_.decay_cells_pub->publish(decay_cloud);
    }

    if (cfg_.frontier_extraction_en && vm_.frontier_pub &&
        vm_.frontier_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> frontier_map;
      boxSearch(box_min, box_max, FRONTIER, frontier_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(frontier_map, cloud_msg);
      cloud_msg.header.stamp = now();
      vm_.frontier_pub->publish(cloud_msg);
    }

    vec_E<Vec3f> occ_map, inf_occ_map;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) {
      boxSearch(box_min, box_max, OCCUPIED, occ_map);
      vecEVec3fToPC2(occ_map, cloud_msg);
      vm_.occ_pub->publish(cloud_msg);
    }

    if (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> raw_occ_map;
      rawOccupiedBoxSearch(box_min, box_max, raw_occ_map);
      sensor_msgs::msg::PointCloud2 raw_cloud_msg;
      vecEVec3fToPC2(raw_occ_map, raw_cloud_msg);
      vm_.raw_occ_pub->publish(raw_cloud_msg);
    }

    if (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) {
      boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
      vecEVec3fToPC2(inf_occ_map, cloud_msg);
      vm_.occ_inf_pub->publish(cloud_msg);
    }

    /* visualize ESDF Map*/
    if (cfg_.esdf_en) {
      if (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) {
        PointCloud pc;
        esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state.p.z() - 0.5, pc);
        pcl::toROSMsg(pc, cloud_msg);
        cloud_msg.header.frame_id = cfg_.visualization_frame_id;
        cloud_msg.header.stamp = now();
        vm_.esdf_pub->publish(cloud_msg);
      }

      // if (vm_.esdf_neg_pub->get_subscription_count() >= 1) {
      //     PointCloud pc;
      //     esdf_map_->getNegativeESDFPointCloud(box_min, box_max, robot_state.p.z() - 0.5, pc);
      //     pcl::toROSMsg(pc, cloud_msg);
      //     cloud_msg.header.frame_id = "world";
      //     cloud_msg.header.stamp = now();
      //     vm_.esdf_neg_pub->publish(cloud_msg);
      // }

#ifdef ESDF_MAP_DEBUG
      esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
      cloud_msg.header.stamp = now();
      vm_.esdf_occ_pub->publish(cloud_msg);
#endif
    }

    if (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1) {
      visualization_msgs::msg::MarkerArray mkr_arr;
      visualizeBoundingBox(
        mkr_arr, now().seconds(), box_min, box_max, "Visualization Range", Color::Purple());
      visualizeText(mkr_arr,
        now().seconds(),
        "Visualization Range Text",
        "Visualization Range",
        box_max + Vec3f(0, 0, 0.5),
        Color::Purple(),
        0.6,
        0);

      Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
      boundBoxByLocalMap(local_map_min, local_map_max);
      visualizeBoundingBox(
        mkr_arr, now().seconds(), local_map_min, local_map_max, "Local Map Range", Color::Orange());
      visualizeText(mkr_arr,
        now().seconds(),
        "Local Map Range Text",
        "Local Map Range",
        local_map_max + Vec3f(0, 0, 1.0),
        Color::Orange(),
        0.6,
        0);

      visualizeBoundingBox(mkr_arr,
        now().seconds(),
        raycast_data_.cache_box_min,
        raycast_data_.cache_box_max,
        "Updating Range",
        Color::Green());
      visualizeText(mkr_arr,
        now().seconds(),
        "Updating Range Text",
        "Updating Range",
        raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
        Color::Green(),
        0.6,
        0);

      visualizePoint(
        mkr_arr, now().seconds(), local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

      if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(mkr_arr,
          now().seconds(),
          "ESDF Map Text",
          "ESDF Map",
          esdf_box_max + Vec3f(0, 0, 1.0),
          Color::Blue(),
          0.6,
          0);
        visualizeBoundingBox(
          mkr_arr, now().seconds(), esdf_box_min, esdf_box_max, "ESDF Updating Range", Color::Blue());
      }

      for (auto & marker : mkr_arr.markers) {
        marker.header.frame_id = cfg_.visualization_frame_id;
      }
      vm_.mkr_arr_pub->publish(mkr_arr);
    }
  }

  bool hasVisualizationSubscriber()
  {
    return (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) ||
           (vm_.unknown_inf_pub && vm_.unknown_inf_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_dynamic_pub &&
            vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_static_pub &&
            vm_.layer_value_static_pub->get_subscription_count() >= 1) ||
           (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) ||
           (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) ||
           (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) ||
           (vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) ||
           (vm_.global_nav_grid_pub && vm_.global_nav_grid_pub->get_subscription_count() >= 1) ||
           (vm_.global_blocked_edges_pub &&
            vm_.global_blocked_edges_pub->get_subscription_count() >= 1) ||
           (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) ||
           (vm_.frontier_pub && vm_.frontier_pub->get_subscription_count() >= 1) ||
           (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
           (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) ||
           (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) ||
           (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) ||
           (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1);
  }

  // 自车附近诊断：只保留机器人周围一个盒子内的点，用来判断 cost=254 到底来自哪里。
  void publishDebugCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const PointCloud & cloud,
    const Vec3f & origin)
  {
    if (!pub || pub->get_subscription_count() < 1) {
      return;
    }
    PointCloud cropped;
    const double half = cfg_.debug_clouds_half_extent_xy;
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (std::abs(point.x - origin.x()) > half || std::abs(point.y - origin.y()) > half ||
          point.z < cfg_.debug_clouds_min_z || point.z > cfg_.debug_clouds_max_z) {
        continue;
      }
      cropped.points.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cropped, msg);
    msg.header.stamp = now();
    msg.header.frame_id = cfg_.visualization_frame_id;
    pub->publish(msg);
  }

  // 最终 occupancy=254 的格心，同样限制在自车附近。
  void publishDebugObstacleCells(const Vec3f & origin)
  {
    if (!vm_.debug_obstacle_pub || vm_.debug_obstacle_pub->get_subscription_count() < 1) {
      return;
    }
    std::shared_ptr<const MapSnapshot> snapshot;
    if (const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(query_)) {
      snapshot = adapter->snapshot();
    }
    if (!snapshot) {
      return;
    }
    PointCloud cloud;
    const double half = cfg_.debug_clouds_half_extent_xy;
    for (int y = 0; y < snapshot->height; ++y) {
      for (int x = 0; x < snapshot->width; ++x) {
        const size_t idx = static_cast<size_t>(y) * snapshot->width + static_cast<size_t>(x);
        if (snapshot->values[idx] != 254U) {
          continue;
        }
        const double wx = snapshot->origin_x + (static_cast<double>(x) + 0.5) * snapshot->resolution;
        const double wy = snapshot->origin_y + (static_cast<double>(y) + 0.5) * snapshot->resolution;
        if (std::abs(wx - origin.x()) > half || std::abs(wy - origin.y()) > half) {
          continue;
        }
        PointType point;
        point.x = static_cast<float>(wx);
        point.y = static_cast<float>(wy);
        point.z = 0.0F;
        cloud.points.push_back(point);
      }
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = now();
    msg.header.frame_id = cfg_.visualization_frame_id;
    vm_.debug_obstacle_pub->publish(msg);
  }

  void vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud)
  {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillLayerGrid(const std::vector<uint8_t> & data, nav_msgs::msg::OccupancyGrid & grid)
  {
    grid.header.stamp = now();
    grid.header.frame_id = cfg_.visualization_frame_id;
    grid.info.resolution = static_cast<float>(layer_->resolution());
    grid.info.width = static_cast<uint32_t>(layer_->width());
    grid.info.height = static_cast<uint32_t>(layer_->height());
    grid.info.origin.position.x = layer_->origin().x();
    grid.info.origin.position.y = layer_->origin().y();
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      // OccupancyGrid 用 -1 表示 unknown。之前把 255 也 clamp 成 100，
      // 于是 /rog_map/layer_value 里 UNKNOWN 和 OCCUPIED 完全无法区分。
      grid.data[i] = (data[i] == 255U) ? -1 : static_cast<int8_t>(std::min<int>(100, data[i]));
    }
  }

  void fillLayerMaskGrid(const std::vector<uint8_t> & mask, nav_msgs::msg::OccupancyGrid & grid)
  {
    std::vector<uint8_t> occupancy(mask.size(), 0U);
    for (size_t i = 0; i < mask.size(); ++i) {
      occupancy[i] = mask[i] == 0U ? 100U : 0U;
    }
    fillLayerGrid(occupancy, grid);
  }

  void fillLayerHeightDeltaCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto & cells = layer_->cells();
    pcl_cloud.reserve(cells.size());
    for (int y = 0; y < layer_->height(); ++y) {
      for (int x = 0; x < layer_->width(); ++x) {
        const size_t idx =
          static_cast<size_t>(y) * static_cast<size_t>(layer_->width()) + static_cast<size_t>(x);
        if (idx >= cells.size()) {
          continue;
        }
        const auto & cell = cells[idx];
        if (cell.type == CellType::UNKNOWN) {
          continue;
        }
        pcl::PointXYZI p;
        p.x =
          static_cast<float>(layer_->origin().x() + (static_cast<double>(x) + 0.5) * layer_->resolution());
        p.y =
          static_cast<float>(layer_->origin().y() + (static_cast<double>(y) + 0.5) * layer_->resolution());
        p.z = cell.occupied_z_max_abs;
        p.intensity = cell.height_delta;
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillFieldCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto distances = field_->distances();
    const int width = field_->width();
    const int height = field_->height();
    const double resolution = field_->resolution();
    const Eigen::Vector2d origin = field_->origin();
    pcl_cloud.reserve(distances.size());
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
        if (idx >= distances.size()) {
          continue;
        }
        const double d = distances[idx];
        if (!std::isfinite(d)) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(origin.x() + (static_cast<double>(x) + 0.5) * resolution);
        p.y = static_cast<float>(origin.y() + (static_cast<double>(y) + 0.5) * resolution);
        p.z = 0.0f;
        p.intensity = static_cast<float>(d);
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  // 全局二维地形图 -> 彩色点云。只画“有信息”的格子（未知留空），避免每帧几十万点：
  //   已观测可通行 = 浅蓝灰；BLOCKED 边 = 橙；障碍 = 红。
  // 全局二维地形图 -> OccupancyGrid：-1 UNKNOWN / 0 FREE / 100 OCCUPIED。
  // 边缘单独走 /global_terrain/blocked_edges，不混进可通行层。
  void fillGlobalNavGrid(nav_msgs::msg::OccupancyGrid & grid)
  {
    if (!global_grid_) {
      return;
    }
    const int width = static_cast<int>(global_grid_->sizeX());
    const int height = static_cast<int>(global_grid_->sizeY());
    if (width <= 0 || height <= 0) {
      return;
    }
    std::vector<unsigned char> values;
    if (!global_grid_->copyValues(values) ||
        values.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
      return;
    }
    grid.header.stamp = now();
    grid.header.frame_id = cfg_.visualization_frame_id;
    grid.info.resolution = static_cast<float>(global_grid_->resolution());
    grid.info.width = static_cast<uint32_t>(width);
    grid.info.height = static_cast<uint32_t>(height);
    grid.info.origin.position.x = global_grid_->originX();
    grid.info.origin.position.y = global_grid_->originY();
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
      if (values[i] == 255U) {
        grid.data[i] = -1;                       // UNKNOWN
      } else if (values[i] >= 253U) {
        grid.data[i] = 100;                      // OCCUPIED
      } else {
        grid.data[i] = 0;                        // FREE / 已观测可通行
      }
    }
  }

  // 全局二维地形图 -> BLOCKED_EDGE 线段（LINE_LIST，世界系）。
  void fillGlobalBlockedEdges(visualization_msgs::msg::MarkerArray & markers)
  {
    if (!global_grid_) {
      return;
    }
    const int width = static_cast<int>(global_grid_->sizeX());
    const int height = static_cast<int>(global_grid_->sizeY());
    if (width <= 0 || height <= 0) {
      return;
    }
    const std::vector<uint8_t> edges = global_grid_->exportEdgeStates();
    if (edges.size() != static_cast<size_t>(width) * static_cast<size_t>(height) * 2U) {
      return;
    }
    // 只画“地形原因”的 BLOCKED 边（高差超限 / 已确认悬空）。障碍自身的轮廓已经能在
    // nav_grid 的 OCCUPIED 里看到，这里再画一遍只会淹没真正的平台/坡道边缘。
    std::vector<unsigned char> values;
    if (!global_grid_->copyValues(values) ||
        values.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
      return;
    }
    const auto is_obstacle = [&values, width](int x, int y) {
      if (x < 0 || y < 0) {
        return true;
      }
      const size_t idx =
        static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      return idx >= values.size() || values[idx] >= 253U;
    };
    const double resolution = global_grid_->resolution();
    const double origin_x = global_grid_->originX();
    const double origin_y = global_grid_->originY();
    const auto center = [&](int x, int y) {
      return std::pair<float, float>(
        static_cast<float>(origin_x + (static_cast<double>(x) + 0.5) * resolution),
        static_cast<float>(origin_y + (static_cast<double>(y) + 0.5) * resolution));
    };
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = cfg_.visualization_frame_id;
    marker.ns = "global_blocked_edges";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.01, 0.5 * resolution);
    marker.color.r = 1.0F;
    marker.color.g = 0.55F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    constexpr uint8_t kBlocked = static_cast<uint8_t>(EdgeState::BLOCKED);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t base = (static_cast<size_t>(y) * static_cast<size_t>(width) +
          static_cast<size_t>(x)) * 2U;
        if (edges[base] == kBlocked && !is_obstacle(x, y) && !is_obstacle(x + 1, y)) {
          const auto a = center(x, y);
          const auto b = center(x + 1, y);
          geometry_msgs::msg::Point pa;
          pa.x = a.first; pa.y = a.second; pa.z = 0.0;
          geometry_msgs::msg::Point pb;
          pb.x = b.first; pb.y = b.second; pb.z = 0.0;
          marker.points.push_back(pa);
          marker.points.push_back(pb);
        }
        if (edges[base + 1] == kBlocked && !is_obstacle(x, y) && !is_obstacle(x, y + 1)) {
          const auto a = center(x, y);
          const auto b = center(x, y + 1);
          geometry_msgs::msg::Point pa;
          pa.x = a.first; pa.y = a.second; pa.z = 0.0;
          geometry_msgs::msg::Point pb;
          pb.x = b.first; pb.y = b.second; pb.z = 0.0;
          marker.points.push_back(pa);
          marker.points.push_back(pb);
        }
      }
    }
    markers.markers.push_back(marker);
  }


  void fillDecayCellsCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(active_ids_.size());
    const double now_s = now().seconds();
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) || !active_flags_[hash_id] ||
          !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      pcl::PointXYZI p;
      p.x = static_cast<float>(pos.x());
      p.y = static_cast<float>(pos.y());
      p.z = static_cast<float>(pos.z());
      p.intensity = static_cast<float>(std::max(0.0, now_s - static_cast<double>(last_hit_time_[hash_id])));
      pcl_cloud.push_back(p);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void initializeRos()
  {
    // TODO: The current implementation uses a lenient QoS configuration for message transmission.
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

    if (cfg_.prior_map_enable) {
      prior_map_ = loadPriorMap(cfg_.prior_map_yaml_path, cfg_.prior_map_pgm_path);
      RCLCPP_INFO(node_logging_->get_logger(),
        "[ROGMap] loaded prior map %dx%d at %.3f m/cell in frame '%s'",
        prior_map_.width,
        prior_map_.height,
        prior_map_.resolution,
        cfg_.prior_map_frame.c_str());
    }
    if (cfg_.cloud_filter_en) {
      CloudRegisteredCropFilter::MarkerPublisher::SharedPtr marker_publisher;
      if (cfg_.cloud_filter_publish_visualization) {
        marker_publisher = createPublisher<visualization_msgs::msg::MarkerArray>(
          cfg_.cloud_filter_visualization_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
      }
      cloud_filter_ = std::make_unique<CloudRegisteredCropFilter>(
        cfg_, tf_, node_clock_->get_clock(), node_logging_->get_logger(), marker_publisher);
    }
    if (cfg_.debug_clouds_enable || cfg_.debug_pipeline_enable) {
      actual_input_pub_ = createPublisher<sensor_msgs::msg::PointCloud2>(
        "/debug/rog_actual_input", rclcpp::SensorDataQoS().keep_last(1));
    }
    init();
    /// Initialize visualization module
    if (cfg_.visualization_en) {
      visualizer_driver_ = std::make_unique<ROGMapVisualizer>();
      visualizer_driver_->configure(node_base_,
        node_parameters_,
        node_topics_,
        node_timers_,
        cfg_,
        [this, gate = callback_gate_] { gate->run([&] { vizCallback(); }); });
      const auto & pubs = visualizer_driver_->publishers();
      vm_.occ_pub = pubs.occ_pub;
      vm_.raw_occ_pub = pubs.raw_occ_pub;
      vm_.unknown_pub = pubs.unknown_pub;
      vm_.esdf_neg_pub = pubs.esdf_neg_pub;
      vm_.esdf_occ_pub = pubs.esdf_occ_pub;
      vm_.occ_inf_pub = pubs.occ_inf_pub;
      vm_.unknown_inf_pub = pubs.unknown_inf_pub;
      vm_.frontier_pub = pubs.frontier_pub;
      vm_.esdf_pub = pubs.esdf_pub;
      vm_.layer_height_delta_pub = pubs.layer_height_delta_pub;
      vm_.field_pub = pubs.field_pub;
      vm_.decay_cells_pub = pubs.decay_cells_pub;
      vm_.layer_value_pub = pubs.layer_value_pub;
      vm_.layer_value_dynamic_pub = pubs.layer_value_dynamic_pub;
      vm_.layer_value_static_pub = pubs.layer_value_static_pub;
      vm_.layer_type_pub = pubs.layer_type_pub;
      vm_.layer_confidence_pub = pubs.layer_confidence_pub;
      vm_.mkr_arr_pub = pubs.mkr_arr_pub;
      vm_.debug_raw_pub = pubs.debug_raw_pub;
      vm_.debug_filtered_pub = pubs.debug_filtered_pub;
      vm_.debug_obstacle_pub = pubs.debug_obstacle_pub;
      vm_.global_nav_grid_pub = pubs.global_nav_grid_pub;
      vm_.global_blocked_edges_pub = pubs.global_blocked_edges_pub;
    }

    if (cfg_.ros_callback_en) {
      rc_.odom_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = createSubscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, qos, [this, gate = callback_gate_](nav_msgs::msg::Odometry::SharedPtr msg) { gate->run([&] { odomCallback(msg); }); }, so);
      so.callback_group = rc_.cloud_me_cbk_group;
      so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      rc_.cloud_sub = createSubscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic,
        rclcpp::SensorDataQoS().keep_last(1),
        [this, gate = callback_gate_](sensor_msgs::msg::PointCloud2::UniquePtr msg) { gate->run([&] { cloudCallback(std::move(msg)); }); },
        so);
      rc_.update_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = createWallTimer(std::chrono::milliseconds(cfg_.update_period_ms),
        [this, gate = callback_gate_] { gate->run([&] { updateCallback(); }); },
        rc_.update_cbk_group);
    }
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  ROGMapROS(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : lifecycle_nh_(nh), tf_(tf)
  {
    bindNode(lifecycle_nh_);
    cfg_ = cfg;
    odom_buffer_ = OdometryBuffer(cfg_.odom_buffer_sec);
    initializeRos();
  }

  ROGMapROS(const rclcpp::Node::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : nh_(nh), tf_(tf)
  {
    bindNode(nh_);
    cfg_ = cfg;
    odom_buffer_ = OdometryBuffer(cfg_.odom_buffer_sec);
    initializeRos();
  }

  ~ROGMapROS() override { shutdown(); }
  // 只停回调与定时器，不析构订阅/定时器：本函数可能在 executor 回调里被调用，
  // 此时 executor 线程可能正持着这些实体的 waitset。实体随对象一起析构（在容器停止 spin 之后）。
  void shutdown() {
    callback_gate_->stop();  // Close gate and drain every in-flight callback, including visualization.
    if (rc_.update_timer) { rc_.update_timer->cancel(); }
    if (visualizer_driver_) { visualizer_driver_->stop(); }
  }

  std::shared_ptr<rog_map::MapQueryInterface> queryInterface() const
  {
    return getQueryInterface();
  }

  // planner 使用的全局二维地形图查询接口；未启用时返回 nullptr。
  std::shared_ptr<rog_map::MapQueryInterface> globalQueryInterface() const
  {
    return getGlobalQueryInterface();
  }

private:
  static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray & mkrarr,
    const double & stamp,
    const Vec3f & box_min,
    const Vec3f & box_max,
    const string & ns,
    const Color & color,
    const double & size_x = 0.1,
    const double & alpha = 1.0,
    const bool & print_ns = true)
  {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();

    // Publish Bounding box
    int id = 0;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.stamp = rclcpp::Time(stamp);
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++;  // unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;  // marker type
    line_strip.scale.x = size_x;

    line_strip.color = color;
    line_strip.color.a = alpha;  //不透明度，设0则全透明
    geometry_msgs::msg::Point p[8];

    // vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
      line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
  }

  static void visualizeText(visualization_msgs::msg::MarkerArray & mkr_arr,
    const double & stamp,
    const std::string & ns,
    const std::string & text,
    const Vec3f & position,
    const Color & c = Color::White(),
    const double & size = 0.6,
    const int & id = -1)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = rclcpp::Time(stamp);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  };

  static void visualizePoint(visualization_msgs::msg::MarkerArray & mkr_arr,
    const double & stamp,
    const Vec3f & pt,
    Color color = Color::Pink(),
    std::string ns = "pt",
    double size = 0.1,
    int id = -1,
    const bool & print_ns = true)
  {
    visualization_msgs::msg::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
      return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = rclcpp::Time(stamp);
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::msg::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::msg::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);

    // add test
    if (print_ns) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = rclcpp::Time(stamp);
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.ns = ns + "_text";
      if (id >= 0) {
        marker.id = id;
      } else {
        static int id = 0;
        marker.id = id++;
      }
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.6;
      marker.color = color;
      marker.text = ns;
      marker.pose.position.x = cur_pos.x();
      marker.pose.position.y = cur_pos.y();
      marker.pose.position.z = cur_pos.z() + 0.5;
      marker.pose.orientation.w = 1.0;
      mkr_arr.markers.push_back(marker);
    }
  }
};
}  // namespace rog_map
#endif  // ROG_MAP_ROS_HPP
