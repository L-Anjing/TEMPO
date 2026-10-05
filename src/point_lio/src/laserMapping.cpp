#include "imu_odometry_predictor.h"
// #include <so3_math.h>
#include <cstdint>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <limits>
#include <vector>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>

#include "li_initialization.h"
#include "runtime_statistics.h"
#include "sensor_time_rate_limiter.h"
#include "robot_extrinsics.h"
#include "output_frame.h"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

bool init_map = false, flg_first_scan = true;

bool flg_reset = false, flg_exit = false;

// surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
// Global map for visualization
PointCloudXYZI::Ptr global_map_ptr(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;

std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;
point_lio::RobotExtrinsics robot_extrinsics;
std::string odom_frame = "camera_init", robot_frame = "base_link";
std::string lidar_frame = "livox_frame", imu_frame = "imu_link";

// ---- 输出零点：延迟冻结，odom / 高频 odom / TF / 世界系点云 / path 共用同一个变换 ----
std::mutex output_frame_mutex;
point_lio::OutputFrame output_frame;
std::atomic_bool output_frame_locked{false};

// 冻结前不对外发布任何世界系输出：此时零点还没定，发布出去必然带一次性跳变。
bool outputReady() { return output_frame_locked.load(std::memory_order_acquire); }

point_lio::OutputFrame currentOutputFrame()
{
  std::lock_guard<std::mutex> lock(output_frame_mutex);
  return output_frame;
}

point_lio::RobotKinematics toOutputFrame(const point_lio::RobotKinematics & state)
{
  auto robot = state;
  currentOutputFrame().apply(robot);
  return robot;
}

template <typename PointT, typename Allocator>
void toOutputFrame(std::vector<PointT, Allocator> & points)
{
  const auto frame = currentOutputFrame();
  for (auto & point : points) { frame.applyPoint(point); }
}

void toOutputFrame(PointCloudXYZI & cloud) { toOutputFrame(cloud.points); }


auto LOGGER = rclcpp::get_logger("laserMapping");

struct FullCloudPoint
{
  PointType point;
  double stamp{0.0};
};

struct StateSnapshot
{
  double stamp{0.0};
  V3D pos{V3D::Zero()};
  M3D rot{M3D::Identity()};
  V3D vel{V3D::Zero()};
  V3D acc{V3D::Zero()};
  V3D omg{V3D::Zero()};
  M3D R_L_I{M3D::Identity()};
  V3D T_L_I{V3D::Zero()};
};

constexpr double kFullCloudMaxQueueAge = 0.20;
constexpr double kFullCloudStampEps = 1.0e-6;

bool fullCloudPointStampLess(const FullCloudPoint & a, const FullCloudPoint & b)
{
  return a.stamp < b.stamp;
}

std::deque<FullCloudPoint> full_cloud_queue;
bool has_full_cloud_anchor = false;
StateSnapshot full_cloud_anchor;
PointCloudXYZI::Ptr full_cloud_world(new PointCloudXYZI());
uint64_t full_cloud_dropped_points_this_scan = 0;
uint64_t last_full_cloud_enqueued_points = 0;
uint64_t last_full_cloud_out_of_order = 0;

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

bool isFinitePoint(const PointType & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void enqueueFullCloud(const PointCloudXYZI::Ptr & cloud, double lidar_beg_time)
{
  last_full_cloud_enqueued_points = 0;
  last_full_cloud_out_of_order = 0;
  if (!cloud) {
    return;
  }

  std::vector<FullCloudPoint> batch;
  batch.reserve(cloud->points.size());

  double prev_stamp = -std::numeric_limits<double>::infinity();
  for (const auto & point : cloud->points) {
    if (!isFinitePoint(point)) {
      continue;
    }

    FullCloudPoint full_point;
    full_point.point = point;
    full_point.stamp = lidar_beg_time + static_cast<double>(point.curvature) / 1000.0;
    if (!std::isfinite(full_point.stamp)) {
      continue;
    }

    if (full_point.stamp < prev_stamp) {
      ++last_full_cloud_out_of_order;
    }
    prev_stamp = full_point.stamp;

    batch.emplace_back(std::move(full_point));
    ++last_full_cloud_enqueued_points;
  }

  if (batch.empty()) {
    return;
  }

  // Sort batch by stamp before appending to queue
  std::stable_sort(batch.begin(), batch.end(), fullCloudPointStampLess);

  // Append sorted batch to queue
  for (auto & fp : batch) {
    full_cloud_queue.push_back(std::move(fp));
  }

  // Ensure full queue is globally sorted (handles residual future points from
  // prior scans interleaved with current-scan points)
  std::stable_sort(full_cloud_queue.begin(), full_cloud_queue.end(), fullCloudPointStampLess);
}

StateSnapshot makeStateSnapshot(double stamp)
{
  StateSnapshot snapshot;
  snapshot.stamp = stamp;

  if (!use_imu_as_input) {
    snapshot.pos = kf_output.x_.pos;
    snapshot.rot = kf_output.x_.rot;
    snapshot.vel = kf_output.x_.vel;
    snapshot.acc = kf_output.x_.rot * kf_output.x_.acc + kf_output.x_.gravity;
    snapshot.omg = kf_output.x_.omg;
    if (extrinsic_est_en) {
      snapshot.R_L_I = kf_output.x_.offset_R_L_I;
      snapshot.T_L_I = kf_output.x_.offset_T_L_I;
    } else {
      snapshot.R_L_I = Lidar_R_wrt_IMU;
      snapshot.T_L_I = Lidar_T_wrt_IMU;
    }
  } else {
    snapshot.pos = kf_input.x_.pos;
    snapshot.rot = kf_input.x_.rot;
    snapshot.vel = kf_input.x_.vel;
    snapshot.acc = kf_input.x_.rot * (input_in.acc - kf_input.x_.ba) + kf_input.x_.gravity;
    snapshot.omg = input_in.gyro - kf_input.x_.bg;
    if (extrinsic_est_en) {
      snapshot.R_L_I = kf_input.x_.offset_R_L_I;
      snapshot.T_L_I = kf_input.x_.offset_T_L_I;
    } else {
      snapshot.R_L_I = Lidar_R_wrt_IMU;
      snapshot.T_L_I = Lidar_T_wrt_IMU;
    }
  }

  return snapshot;
}

PointType transformFullPointForward(const StateSnapshot & anchor, const FullCloudPoint & full_point)
{
  const double dt = full_point.stamp - anchor.stamp;
  const M3D R_pred = anchor.rot * Exp(anchor.omg, dt);
  const V3D p_pred = anchor.pos + anchor.vel * dt + 0.5 * anchor.acc * dt * dt;

  const V3D p_lidar(full_point.point.x, full_point.point.y, full_point.point.z);
  const V3D p_imu = anchor.R_L_I * p_lidar + anchor.T_L_I;
  const V3D p_world = R_pred * p_imu + p_pred;

  PointType output = full_point.point;
  output.x = p_world.x();
  output.y = p_world.y();
  output.z = p_world.z();
  return output;
}

void processFullCloudSegmentWithSnapshot(
  const StateSnapshot & current_snapshot)
{
  if (!std::isfinite(current_snapshot.stamp)) {
    return;
  }

  if (!has_full_cloud_anchor) {
    full_cloud_anchor = current_snapshot;
    has_full_cloud_anchor = true;
    return;
  }

  if (current_snapshot.stamp <= full_cloud_anchor.stamp + kFullCloudStampEps) {
    full_cloud_anchor = current_snapshot;
    return;
  }

  size_t dropped_old_points = 0;

  while (!full_cloud_queue.empty() &&
         current_snapshot.stamp - full_cloud_queue.front().stamp > kFullCloudMaxQueueAge) {
    full_cloud_queue.pop_front();
    ++dropped_old_points;
  }

  while (!full_cloud_queue.empty()) {
    const FullCloudPoint & full_point = full_cloud_queue.front();
    if (full_point.stamp + kFullCloudStampEps < full_cloud_anchor.stamp) {
      full_cloud_queue.pop_front();
      ++dropped_old_points;
      continue;
    }
    if (full_point.stamp > current_snapshot.stamp + kFullCloudStampEps) {
      break;
    }

    const PointType point_world = transformFullPointForward(full_cloud_anchor, full_point);
    if (isFinitePoint(point_world)) {
      full_cloud_world->push_back(point_world);
    }
    full_cloud_queue.pop_front();
  }

  full_cloud_dropped_points_this_scan += dropped_old_points;
  full_cloud_anchor = current_snapshot;
}

void publishAccumulatedFullCloudWorld(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  double stamp)
{
  if (!pub || !full_cloud_world || full_cloud_world->empty()) {
    return;
  }

  // 零点冻结前丢弃，避免在启动瞬间攒下一大团点后一次性爆发。
  if (!outputReady()) {
    full_cloud_world->clear();
    full_cloud_dropped_points_this_scan = 0;
    return;
  }
  toOutputFrame(*full_cloud_world);

  full_cloud_world->width = static_cast<uint32_t>(full_cloud_world->size());
  full_cloud_world->height = 1;
  full_cloud_world->is_dense = false;

  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*full_cloud_world, *cloud_msg);
  cloud_msg->header.stamp = get_ros_time(stamp);
  cloud_msg->header.frame_id = odom_frame;
  pub->publish(std::move(cloud_msg));

  if (point_lio::RuntimeStatistics::instance().enabled() && print_cloud_input_fps) {
    RCLCPP_INFO(LOGGER,
      "[Point-LIO][FullCloud] enqueued=%llu published=%zu queue_remaining=%zu "
      "dropped_old=%llu out_of_order=%llu",
      static_cast<unsigned long long>(last_full_cloud_enqueued_points),
      full_cloud_world->size(),
      full_cloud_queue.size(),
      static_cast<unsigned long long>(full_cloud_dropped_points_this_scan),
      static_cast<unsigned long long>(last_full_cloud_out_of_order));
  }

  full_cloud_world->clear();
  full_cloud_dropped_points_this_scan = 0;
}

void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
            fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
            fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);

  if (pcd_save_en && !points_to_add.empty()) {
    global_map_ptr->points.insert(global_map_ptr->points.end(), points_to_add.begin(), points_to_add.end());
    global_map_ptr->width = static_cast<uint32_t>(global_map_ptr->points.size());
    global_map_ptr->height = 1;
    global_map_ptr->is_dense = true;
  }
}

void publish_accumulated_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
  if (!pcd_save_en || accumulated_map_publish_hz <= 0.0 || !global_map_ptr || global_map_ptr->empty() ||
      !outputReady()) {
    return;
  }

  static double last_publish_time = -std::numeric_limits<double>::infinity();
  const double publish_period = 1.0 / accumulated_map_publish_hz;
  if (lidar_end_time - last_publish_time < publish_period) {
    return;
  }
  last_publish_time = lidar_end_time;

  // 累加图始终存在滤波器内部世界系，发布时统一换到输出系（与 odom/TF 同系）。
  PointCloudXYZI map_in_output_frame = *global_map_ptr;
  toOutputFrame(map_in_output_frame);
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_in_output_frame, map_msg);
  map_msg.header.stamp = get_ros_time(lidar_end_time);
  map_msg.header.frame_id = odom_frame;
  pubLaserCloudMap->publish(map_msg);
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

point_lio::StateLogRecord makeStateLogRecord()
{
  point_lio::StateLogRecord record;
  record.relative_time = Measures.lidar_beg_time - first_lidar_time;
  record.use_imu_as_input = use_imu_as_input;

  const auto copy_vector = [](const auto & source, std::array<double, 3> & destination) {
    destination = {source(0), source(1), source(2)};
  };

  if (use_imu_as_input) {
    copy_vector(SO3ToEuler(kf_input.x_.rot), record.euler);
    copy_vector(kf_input.x_.pos, record.pos);
    copy_vector(kf_input.x_.vel, record.vel);
    copy_vector(kf_input.x_.gravity, record.gravity);
    copy_vector(kf_input.x_.bg, record.bg);
    copy_vector(kf_input.x_.ba, record.ba);
  } else {
    copy_vector(SO3ToEuler(kf_output.x_.rot), record.euler);
    copy_vector(kf_output.x_.pos, record.pos);
    copy_vector(kf_output.x_.vel, record.vel);
    copy_vector(kf_output.x_.omg, record.omega);
    copy_vector(kf_output.x_.acc, record.acc);
    copy_vector(kf_output.x_.gravity, record.gravity);
    copy_vector(kf_output.x_.bg, record.bg);
    copy_vector(kf_output.x_.ba, record.ba);
  }
  record.point_count = feats_undistort ? feats_undistort->size() : 0;
  return record;
}

void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en && outputReady()) {
    // feats_down_world 仍留在滤波器内部世界系，拷贝一份换到输出系再发布。
    PointCloudXYZI cloud_in_output_frame = *feats_down_world;
    toOutputFrame(cloud_in_output_frame);
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(cloud_in_output_frame, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *pcl_wait_save += cloud_in_output_frame;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(
          string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = imu_frame;
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

point_lio::RobotKinematics currentRobotKinematics()
{
  if (use_imu_as_input) {
    return point_lio::robotKinematics(robot_extrinsics, kf_input.x_.pos,
      Eigen::Matrix3d(kf_input.x_.rot), kf_input.x_.vel, input_in.gyro - kf_input.x_.bg);
  }
  return point_lio::robotKinematics(robot_extrinsics, kf_output.x_.pos,
    Eigen::Matrix3d(kf_output.x_.rot), kf_output.x_.vel, kf_output.x_.omg);
}

template <typename T> void set_posestamp(T & out, const point_lio::RobotKinematics & state)
{
  out.position.x = state.position.x();
  out.position.y = state.position.y();
  out.position.z = state.position.z();
  out.orientation.x = state.rotation.x();
  out.orientation.y = state.rotation.y();
  out.orientation.z = state.rotation.z();
  out.orientation.w = state.rotation.w();
}

// robot 必须已经在输出系里；odom 与 TF 用同一个 state，不能各自再变换一次。
void sendRobotTransform(const point_lio::RobotKinematics & robot, double stamp,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> & broadcaster)
{
  if (!tf_send_en) { return; }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = odom_frame;
  transform.header.stamp = get_ros_time(stamp);
  transform.child_frame_id = robot_frame;
  transform.transform.translation.x = robot.position.x();
  transform.transform.translation.y = robot.position.y();
  transform.transform.translation.z = robot.position.z();
  transform.transform.rotation.x = robot.rotation.x();
  transform.transform.rotation.y = robot.rotation.y();
  transform.transform.rotation.z = robot.rotation.z();
  transform.transform.rotation.w = robot.rotation.w();
  broadcaster->sendTransform(transform);
}

void publish_robot_transform(const point_lio::RobotKinematics & state, double stamp,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> & broadcaster)
{
  if (!tf_send_en || !outputReady()) { return; }
  sendRobotTransform(toOutputFrame(state), stamp, broadcaster);
}

// Public pose and twist belong to the robot center, while the EKF stays in the IMU frame.
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br, double initial_stamp = -1.0)
{
  static point_lio::SensorTimeRateLimiter rate_limiter;
  if (!outputReady()) { return; }
  const double stamp = initial_stamp >= 0.0 ? initial_stamp :
    (publish_odometry_without_downsample ? time_current : lidar_end_time);
  if (!rate_limiter.should_publish(stamp, orig_odom_freq)) { return; }
  const auto state = toOutputFrame(currentRobotKinematics());
  odomAftMapped.header.frame_id = odom_frame;
  odomAftMapped.child_frame_id = robot_frame;
  odomAftMapped.header.stamp = get_ros_time(stamp);
  set_posestamp(odomAftMapped.pose.pose, state);
  auto & twist = odomAftMapped.twist.twist;
  twist.linear.x = state.linear_velocity.x();
  twist.linear.y = state.linear_velocity.y();
  twist.linear.z = state.linear_velocity.z();
  twist.angular.x = state.angular_velocity.x();
  twist.angular.y = state.angular_velocity.y();
  twist.angular.z = state.angular_velocity.z();
  publisher->publish(odomAftMapped);
  if (odom_log_interval_sec > 0.0) {
    static auto last_log_time = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (last_log_time.time_since_epoch().count() == 0 ||
        now - last_log_time >= std::chrono::duration<double>(odom_log_interval_sec)) {
      // ZYX (yaw, pitch, roll) extraction, matching the fastlio console convention.
      // atan2 avoids the spurious 180-degree jumps of Eigen::eulerAngles at yaw=0.
      const Eigen::Matrix3d rotation = state.rotation.toRotationMatrix();
      constexpr double radians_to_degrees = 57.29577951308232;
      const double roll = std::atan2(rotation(2, 1), rotation(2, 2)) * radians_to_degrees;
      const double pitch = std::atan2(-rotation(2, 0),
        std::hypot(rotation(0, 0), rotation(1, 0))) * radians_to_degrees;
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0)) * radians_to_degrees;
      RCLCPP_INFO(LOGGER,
        "odom[%s -> %s] xyz=(%.3f, %.3f, %.3f) m rpy=(%.2f, %.2f, %.2f) deg",
        odom_frame.c_str(), robot_frame.c_str(),
        state.position.x(), state.position.y(), state.position.z(), roll, pitch, yaw);
      last_log_time = now;
    }
  }
  const double publish_time =
    std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  point_lio::RuntimeStatistics::instance().recordOdomPublish(stamp, publish_time);
  sendRobotTransform(state, stamp, tf_br);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  if (!outputReady()) { return; }
  set_posestamp(msg_body_pose.pose, toOutputFrame(currentRobotKinematics()));
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = odom_frame;
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

namespace point_lio {

class LaserMappingNode : public rclcpp::Node
{
public:
  explicit LaserMappingNode(const rclcpp::NodeOptions & options) : rclcpp::Node("laserMapping", options)
  {
    initialize();
    startWorker();
  }

  ~LaserMappingNode() override
  {
    stopWorker();
    if (!pcl_wait_save->empty() && pcd_save_en) {
      string file_name = string("scans.pcd");
      string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
      pcl::PCDWriter pcd_writer;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    RuntimeStatistics::instance().shutdown();
  }

private:
  void initialize()
  {
    readParameters(*this);
    zero_odom_at_start_ = declare_parameter<bool>("odometry.zero_at_start", true);
    zero_delay_sec_ = declare_parameter<double>("odometry.zero_delay_sec", 1.5);
    if (!std::isfinite(zero_delay_sec_) || zero_delay_sec_ < 0.0) {
      throw std::invalid_argument("odometry.zero_delay_sec must be non-negative");
    }
    const std::vector<double> identity{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    auto rotation = [&](const std::string & name) {
      return point_lio::extrinsicRotation(declare_parameter<std::vector<double>>(name, identity), name);
    };
    auto translation = [&](const std::string & name) {
      return point_lio::extrinsicTranslation(
        declare_parameter<std::vector<double>>(name, std::vector<double>{0., 0., 0.}), name);
    };
    robot_extrinsics.R_imu_in_lidar = rotation("sensor_extrinsic.imu_in_lidar_R");
    robot_extrinsics.t_imu_in_lidar = translation("sensor_extrinsic.imu_in_lidar_T");
    robot_extrinsics.R_lidar_in_robot = rotation("robot_extrinsic.lidar_to_robot_R");
    robot_extrinsics.t_lidar_in_robot = translation("robot_extrinsic.lidar_to_robot_T");
    odom_frame = declare_parameter<std::string>("frames.odom", "camera_init");
    robot_frame = declare_parameter<std::string>("frames.robot", "base_link");
    lidar_frame = declare_parameter<std::string>("frames.lidar", "livox_frame");
    imu_frame = declare_parameter<std::string>("frames.imu", "imu_link");
    const std::vector<std::string> frames{odom_frame, robot_frame, lidar_frame, imu_frame};
    for (size_t i = 0; i < frames.size(); ++i) {
      if (frames[i].empty()) { throw std::invalid_argument("TF frame names must not be empty"); }
      for (size_t j = 0; j < i; ++j) {
        if (frames[i] == frames[j]) { throw std::invalid_argument("TF frame names must be distinct"); }
      }
    }
    if (extrinsic_est_en) {
      throw std::invalid_argument("Calibrated robot TF requires mapping.extrinsic_est_en=false");
    }
    robot_extrinsics.initializeTransforms();
    Lidar_R_wrt_IMU = robot_extrinsics.lidarRotationInImu();
    Lidar_T_wrt_IMU = robot_extrinsics.lidarOriginInImu();
    if (blind_center_enable && blind_center.size() == 3U) {
      p_pre->setBlindCenter(blind_center[0], blind_center[1], blind_center[2]);
      RCLCPP_INFO(get_logger(), "[Point-LIO] Blind center enabled: [%.3f, %.3f, %.3f]",
        blind_center[0], blind_center[1], blind_center[2]);
    } else {
      if (blind_center_enable) {
        RCLCPP_WARN(get_logger(),
          "[Point-LIO] Invalid blind_center size (%zu), fallback to LiDAR origin.",
          blind_center.size());
      }
      p_pre->setBlindCenter(0.0, 0.0, 0.0);
      RCLCPP_INFO(get_logger(), "[Point-LIO] Blind center disabled, using LiDAR origin.");
    }
    std::cout << "lidar_type: " << lidar_type << '\n';
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = odom_frame;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    reset_cov(P_init_);
    kf_input.change_P(P_init_);
    reset_cov_output(P_init_output_);
    kf_output.change_P(P_init_output_);
    Q_input_ = process_noise_cov_input();
    Q_output_ = process_noise_cov_output();

    double statistics_print_period =
      cloud_input_fps_print_period > 0.05 ? cloud_input_fps_print_period : 1.0;
    if (debug_pose_update_detail &&
        (!print_cloud_input_fps || debug_pose_update_detail_period < statistics_print_period)) {
      statistics_print_period = debug_pose_update_detail_period;
    }
    const std::filesystem::path log_dir =
      resolveLogDirectory(runtime_log_path, std::filesystem::path(root_dir));
    if (!RuntimeStatistics::instance().initialize(
        runtime_pos_log, log_dir, print_cloud_input_fps, debug_pose_update_detail,
        statistics_print_period)) {
      RCLCPP_ERROR(get_logger(),
        "[Point-LIO] Failed to initialize runtime statistics under %s. Runtime logging disabled.",
        log_dir.c_str());
      runtime_pos_log = false;
    }

    if (runtime_pos_log && print_cloud_input_fps) {
      RCLCPP_INFO(get_logger(),
        "[Point-LIO] runtime rate print enabled, period: %.2f s, lidar topic: %s",
        cloud_input_fps_print_period > 0.0 ? cloud_input_fps_print_period : 1.0,
        lid_topic.c_str());
    }

    if (p_pre->lidar_type == AVIA) {
      sub_pcl_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(), [](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
          RuntimeStatistics::instance().recordRate(RuntimeRateEvent::CloudInput);
          livox_pcl_cbk(std::move(msg));
        });
    } else {
      sub_pcl_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(), [](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
          RuntimeStatistics::instance().recordRate(RuntimeRateEvent::CloudInput);
          standard_pcl_cbk(std::move(msg));
        });
    }
    high_frequency_enable_ = declare_parameter<bool>("odometry.high_frequency_enable", true);
    max_correction_age_ = declare_parameter<double>("odometry.max_correction_age_sec", 0.3);
    if (!std::isfinite(max_correction_age_) || max_correction_age_ <= 0.0) {
      throw std::invalid_argument("odometry.max_correction_age_sec must be positive");
    }
    high_frequency_frame_ = declare_parameter<std::string>("frames.robot_high_frequency", "base_link_hf");
    if (high_frequency_frame_ == robot_frame || high_frequency_frame_ == odom_frame) {
      throw std::invalid_argument("frames.robot_high_frequency must be a separate TF child");
    }
    pub_high_frequency_ = create_publisher<nav_msgs::msg::Odometry>("/OdometryHighFreq", 20);
    pub_scan_odometry_ = create_publisher<nav_msgs::msg::Odometry>("/point_lio/odometry_scan", 20);
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_cbk(msg);
        if (!high_frequency_enable_) { return; }
        ImuOdometryPredictor::State predicted;
        {
          std::lock_guard<std::mutex> lock(predictor_mutex_);
          predictor_.push({get_time_sec(msg->header.stamp) - timediff_imu_wrt_lidar -
            time_lag_IMU_wtr_lidar,
            {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z},
            {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z}});
          if (!predictor_.predict(predicted, max_correction_age_)) { return; }
        }
        publishPredictedOdometry(predicted);
      });
    pub_laser_cloud_full_res_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
    pub_laser_cloud_full_world_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_full", rclcpp::SensorDataQoS().keep_last(1));
    pub_laser_cloud_full_res_body_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 20);
    pub_laser_cloud_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
    pub_odom_aft_mapped_ = create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 20);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 20);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    if (tf_send_en) {
      auto transform = [&](const std::string & parent, const std::string & child,
          const Eigen::Matrix3d & rotation, const Eigen::Vector3d & translation) {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = parent;
        msg.child_frame_id = child;
        msg.transform.translation.x = translation.x();
        msg.transform.translation.y = translation.y();
        msg.transform.translation.z = translation.z();
        const Eigen::Quaterniond q(rotation);
        msg.transform.rotation.x = q.x(); msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z(); msg.transform.rotation.w = q.w();
        return msg;
      };
      static_tf_broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
        transform(robot_frame, lidar_frame, robot_extrinsics.R_lidar_in_robot,
          robot_extrinsics.t_lidar_in_robot),
        transform(lidar_frame, imu_frame, robot_extrinsics.R_imu_in_lidar,
          robot_extrinsics.t_imu_in_lidar)});
    }
  }

  void publishPredictedOdometry(const ImuOdometryPredictor::State & predicted)
  {
    if (!outputReady()) { return; }
    const auto state = toOutputFrame(robotKinematics(robot_extrinsics, predicted.pos, predicted.rot,
      predicted.vel, predicted.omega));
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = odom_frame;
    msg.header.stamp = get_ros_time(predicted.stamp);
    msg.child_frame_id = high_frequency_frame_;
    set_posestamp(msg.pose.pose, state);
    msg.twist.twist.linear.x = state.linear_velocity.x();
    msg.twist.twist.linear.y = state.linear_velocity.y();
    msg.twist.twist.linear.z = state.linear_velocity.z();
    msg.twist.twist.angular.x = state.angular_velocity.x();
    msg.twist.twist.angular.y = state.angular_velocity.y();
    msg.twist.twist.angular.z = state.angular_velocity.z();
    // Covariance is not estimated by this predictor (same convention as corrected output).
    pub_high_frequency_->publish(msg);
    if (tf_send_en) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header;
      tf.child_frame_id = high_frequency_frame_;
      tf.transform.translation.x = state.position.x();
      tf.transform.translation.y = state.position.y();
      tf.transform.translation.z = state.position.z();
      tf.transform.rotation = msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // batch 模式下每个 batch 更新后都会调用（publish_scan=false），只更新 posterior 供
  // /OdometryHighFreq 使用；整帧结束时的调用仍然发布 /point_lio/odometry_scan。
  void seedHighFrequencyOdometry(bool publish_scan = true)
  {
    if (is_first_frame || time_current <= 0.0) { return; }
    ImuOdometryPredictor::State state;
    const auto snapshot = makeStateSnapshot(time_current);
    state.stamp = time_current;
    state.pos = snapshot.pos;
    state.rot = snapshot.rot;
    state.vel = snapshot.vel;
    state.omega = snapshot.omg;
    state.acc_scale = G_m_s2 / acc_norm;
    if (use_imu_as_input) {
      state.gravity = kf_input.x_.gravity;
      state.bg = kf_input.x_.bg;
      state.ba = kf_input.x_.ba;
    } else {
      state.gravity = kf_output.x_.gravity;
      state.bg = kf_output.x_.bg;
      state.ba = kf_output.x_.ba;
    }
    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);
      predictor_.seed(state);
      const double correction_age = predictor_.latestStamp() - state.stamp;
      if (high_frequency_enable_ && correction_age > max_correction_age_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[Point-LIO] LiDAR correction is %.3f s behind IMU; /OdometryHighFreq withheld (limit %.3f s)",
          correction_age, max_correction_age_);
      }
    }
    // Publish the actual final corrected time, not a scan-end timestamp attached to an older state.
    if (!publish_scan || !outputReady()) { return; }
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = odom_frame;
    msg.header.stamp = get_ros_time(state.stamp);
    msg.child_frame_id = robot_frame;
    const auto robot = toOutputFrame(currentRobotKinematics());
    set_posestamp(msg.pose.pose, robot);
    msg.twist.twist.linear.x = robot.linear_velocity.x();
    msg.twist.twist.linear.y = robot.linear_velocity.y();
    msg.twist.twist.linear.z = robot.linear_velocity.z();
    msg.twist.twist.angular.x = robot.angular_velocity.x();
    msg.twist.twist.angular.y = robot.angular_velocity.y();
    msg.twist.twist.angular.z = robot.angular_velocity.z();
    pub_scan_odometry_->publish(msg);
  }

  void startWorker()
  {
    running_.store(true);
    worker_thread_ = std::thread(&LaserMappingNode::processingLoop, this);
  }

  void stopWorker()
  {
    running_.store(false);
    sig_buffer.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  void processingLoop();

  // 输出零点延迟冻结：IMU 初始化时的姿态还没被 LiDAR 校正过（FromTwoVectors 只定 roll/pitch，
  // yaw 任意），此时冻结会把没收敛的偏置写进所有对外输出。等 LiDAR IEKF 跑够 zero_delay_sec_
  // 之后再记录 T_WB0，滤波器内部状态/协方差/地图一律不重写。
  void updateOutputFrame()
  {
    if (output_frame_locked.load(std::memory_order_acquire)) { return; }
    if (!zero_odom_at_start_) {
      output_frame_locked.store(true, std::memory_order_release);
      return;
    }
    if (!init_map || time_current <= 0.0) { return; }
    if (output_frame_lock_deadline_ < 0.0) {
      output_frame_lock_deadline_ = time_current + zero_delay_sec_;
      RCLCPP_INFO(get_logger(),
        "[Point-LIO] Output origin freezes at t=%.3f (%.2f s after the first LiDAR correction)",
        output_frame_lock_deadline_, zero_delay_sec_);
      return;
    }
    if (time_current < output_frame_lock_deadline_) { return; }
    const point_lio::OutputFrame frozen(currentRobotKinematics());
    {
      std::lock_guard<std::mutex> lock(output_frame_mutex);
      output_frame = frozen;
    }
    output_frame_locked.store(true, std::memory_order_release);
    RCLCPP_INFO(get_logger(),
      "[Point-LIO] Output origin frozen at internal xyz=(%.3f, %.3f, %.3f) m; "
      "odom, TF and world clouds now share it",
      frozen.origin.x(), frozen.origin.y(), frozen.origin.z());
  }

  std::atomic_bool running_{false};
  std::thread worker_thread_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_full_res_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_full_world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_full_res_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  ImuOdometryPredictor predictor_;
  std::mutex predictor_mutex_;
  bool high_frequency_enable_{true};
  double max_correction_age_{0.3};
  std::string high_frequency_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_high_frequency_, pub_scan_odometry_;

  double last_proc_time_{-1.0};
  int startup_frame_cnt_{0};
  bool zero_odom_at_start_{true};
  double zero_delay_sec_{1.5};
  double output_frame_lock_deadline_{-1.0};
  static constexpr uint64_t kIvoxStatisticsPeriodFrames = 20;
  uint64_t ivox_statistics_frame_counter_{0};
  Eigen::Matrix<double, 24, 24> P_init_;
  Eigen::Matrix<double, 30, 30> P_init_output_;
  Eigen::Matrix<double, 24, 24> Q_input_;
  Eigen::Matrix<double, 30, 30> Q_output_;
};

// ======================== Batch-LIO 批量去畸变 ========================
// 参考 Batch-LIO 的 deskew.h：把 batch 内每个点从自己的采样时刻补偿到 batch 末端时刻。
// tau = t_end - t_i >= 0，常数体坐标角速度 omg 与常数世界系速度 vel：
//   p_imu(t_end) = Exp(-omg * tau) * ( p_imu(t_i) - R_wl^T * vel * tau )
// 其中 p_imu = R_LI * p_lidar + T_LI，R_wl = kf_output.x_.rot（已传播到 t_end）。
// 结果写回 LiDAR 系，保证下游 pbody_list / crossmat_list / pointBodyToWorld 用的是同一套坐标。
static constexpr double kBatchDeskewMaxTauSec = 0.02;  // tau 异常保护（正常 <= batch.dt）

void deskewBatchInto(const PointCloudXYZI::Ptr & raw,
  PointCloudXYZI::Ptr & out,
  int begin,
  int count,
  double t_beg,
  double t_end)
{
  if (!raw || !out || count <= 0) {
    return;
  }
  const M3D R_wl = kf_output.x_.rot;     // world <- IMU（batch 末端状态）
  const V3D omg = kf_output.x_.omg;      // IMU 体系角速度
  const V3D v_imu = R_wl.transpose() * kf_output.x_.vel;  // 世界速度转到 IMU 系
  const int n = static_cast<int>(raw->points.size());
  for (int j = 0; j < count; ++j) {
    const int i = begin + j;
    if (i < 0 || i >= n) {
      break;
    }
    const PointType & p_raw = raw->points[i];
    const double tau = t_end - (t_beg + p_raw.curvature / 1000.0);
    if (!(tau > 0.0) || tau > kBatchDeskewMaxTauSec) {
      out->points[i] = p_raw;  // 时刻异常：保持原始坐标，不制造伪补偿
      continue;
    }
    V3D p_imu = Lidar_R_wrt_IMU * V3D(p_raw.x, p_raw.y, p_raw.z) + Lidar_T_wrt_IMU;
    const V3D rot_vec = (-omg * tau).eval();  // Exp 需要具体矩阵，先求值
    p_imu = Exp(rot_vec) * (p_imu - v_imu * tau);
    const V3D p_lidar = Lidar_R_wrt_IMU.transpose() * (p_imu - Lidar_T_wrt_IMU);
    out->points[i].x = p_lidar.x();
    out->points[i].y = p_lidar.y();
    out->points[i].z = p_lidar.z();
    out->points[i].curvature = p_raw.curvature;  // 时间戳保持不变
    out->points[i].intensity = p_raw.intensity;
  }
}

// 去畸变改变了点的体坐标，必须同步重建依赖它的 Jacobian 缓存，否则会出现
// “KNN 用补偿后坐标、Jacobian 用补偿前坐标”的隐蔽错误。
void rebuildBatchPointCaches(int begin, int count)
{
  for (int j = 0; j < count; ++j) {
    const int i = begin + j;
    if (i < 0 || i >= static_cast<int>(feats_down_body->points.size())) {
      break;
    }
    V3D point_this(feats_down_body->points[i].x, feats_down_body->points[i].y,
      feats_down_body->points[i].z);
    pbody_list[i] = point_this;
    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    crossmat_list[i] = point_crossmat;
  }
}

void LaserMappingNode::processingLoop()
{
  rclcpp::Rate rate(500);
  while (rclcpp::ok() && running_.load()) {
    if (flg_exit)
      break;
    if (sync_packages(Measures)) {
      const bool statistics_enabled = RuntimeStatistics::instance().enabled();
      const auto process_wall_start =
        statistics_enabled ? std::chrono::steady_clock::now()
                           : std::chrono::steady_clock::time_point{};
      RuntimeStatistics::instance().recordRate(RuntimeRateEvent::SyncInput);
      RuntimeStatistics::instance().beginFrame(
        Measures.lidar_beg_time, Measures.lidar ? Measures.lidar->size() : 0);
      startup_frame_cnt_++;

      bool trigger_exit = false;
      std::string exit_reason = "";

      // 1. [故障监测] 时间戳乱序 (严重系统错误)
      if (last_proc_time_ > 0 && Measures.lidar_beg_time < last_proc_time_) {
        exit_reason = "Time Disorder Detected (Curr < Last)";
        trigger_exit = true;
      }

      // 2. [故障监测] 网络大延时 (严重丢包/网络拥塞)
      if (!trigger_exit && last_proc_time_ > 0 && (Measures.lidar_beg_time - last_proc_time_) > 1.0 &&
          startup_frame_cnt_ > 2000) {
        exit_reason = "Large Time Gap (>0.5s)";
        trigger_exit = true;
      }

      // [执行] 触发退出，由 ROS launch 的 respawn 机制接管
      if (trigger_exit) {
        RCLCPP_FATAL(LOGGER, "[CRASH MONITOR] %s. Respawning...", exit_reason.c_str());

        exit(EXIT_FAILURE);
      }

      last_proc_time_ = Measures.lidar_beg_time;
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
        // 重新开始：零点也要重新延迟冻结，冻结前的输出一律不发布。
        {
          std::lock_guard<std::mutex> lock(output_frame_mutex);
          output_frame = point_lio::OutputFrame();
        }
        output_frame_locked.store(false, std::memory_order_release);
        output_frame_lock_deadline_ = -1.0;
        {
          std::lock_guard<std::mutex> lock(predictor_mutex_);
          predictor_.reset();
        }
        path.poses.clear();
        init_feats_world->clear();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_x(state_in);
          kf_input.change_P(P_init_);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_x(state_out);
          kf_output.change_P(P_init_output_);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;
        full_cloud_queue.clear();
        has_full_cloud_anchor = false;
        full_cloud_world->clear();
        full_cloud_dropped_points_this_scan = 0;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
        global_map_ptr->clear();
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      /*** downsample the feature points in a scan ***/
      if (statistics_enabled) {
        const double full_undistort_start = omp_get_wtime();
        p_imu->Process(Measures, feats_undistort);
        RuntimeStatistics::instance().recordFullUndistort(
          (omp_get_wtime() - full_undistort_start) * 1000.0);
      } else {
        p_imu->Process(Measures, feats_undistort);
      }
      enqueueFullCloud(feats_undistort, Measures.lidar_beg_time);
      if (!has_full_cloud_anchor) {
        full_cloud_anchor = makeStateSnapshot(Measures.lidar_beg_time);
        full_cloud_anchor.stamp = Measures.lidar_beg_time;
        has_full_cloud_anchor = true;
      }
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        // Batch-LIO 模式：batch.dt 决定时间窗宽度（秒）；<=0 时回退逐时间戳(逐点)更新。
        // 非 batch 模式保持原 pose_update_time_bin_ms 行为，便于严格 A/B 对照。
        const double update_bin_ms =
          batch_enable ? std::max(0.0, batch_dt * 1000.0) : pose_update_time_bin_ms;
        time_seq = time_compressing<int>(feats_down_body, update_bin_ms);
        feats_down_size = feats_down_body->points.size();
        // batch 去畸变以一个不可变的原始副本为基准，保证每个 batch 的补偿幂等（不累积）。
        if (batch_enable && batch_deskew && feats_down_size > 0) {
          *feats_down_body_raw = *feats_down_body;
        }
      }
      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
            // Startup assumes a stationary IMU, as in FAST-LIO. Do not discard its bias estimate.
            kf_input.x_.bg = p_imu->gyroBias();
            kf_output.x_.bg = p_imu->gyroBias();
            RCLCPP_INFO(get_logger(), "[Point-LIO] Initial gyro bias=(%.6f, %.6f, %.6f) rad/s",
              p_imu->gyroBias().x(), p_imu->gyroBias().y(), p_imu->gyroBias().z());
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      updateOutputFrame();
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          // Initialize exclusively from this run's observed points.
          ivox_->AddPoints(init_feats_world->points);
          if (pcd_save_en) {
            *global_map_ptr += *init_feats_world;
          }
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        full_cloud_queue.clear();
        has_full_cloud_anchor = false;
        full_cloud_world->clear();
        full_cloud_dropped_points_this_scan = 0;
        continue;
      }

      /*** Online scan-to-local-map matching and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      /*** iterated state estimation ***/
      crossmat_list.resize(feats_down_size);
      pbody_list.resize(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty())
                    break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu =
                get_time_sec(imu_next.header.stamp) == get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const && !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty())
                    break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output_, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    kf_output.predict(dt_cov, Q_output_, input_in, false, true);
                    kf_output.update_iterated_dyn_share_IMU();
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output_, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output_, input_in, true, false);
            time_predict_last_const = time_current;

            // Batch-LIO：状态已传播到 batch 末端 tN，用该状态把本 batch 的点补偿到 tN，
            // 并同步重建 pbody_list / crossmat_list。IEKF 迭代复用这套坐标，不重复补偿。
            if (batch_enable && batch_deskew && !feats_down_body_raw->empty()) {
              deskewBatchInto(feats_down_body_raw,
                feats_down_body,
                idx + 1,
                time_seq[k],
                pcl_beg_time,
                time_current);
              rebuildBatchPointCaches(idx + 1, time_seq[k]);
            }

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            const auto update_start = statistics_enabled ? std::chrono::steady_clock::now()
                                                         : std::chrono::steady_clock::time_point{};
            const bool update_success = kf_output.update_iterated_dyn_share_modified();
            if (statistics_enabled) {
              const double update_time_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - update_start)
                  .count();
              RuntimeStatistics::instance().recordPoseUpdate(
                static_cast<uint64_t>(time_seq[k]), time_current, update_time_ms, update_success);
            }
            if (!update_success) {
              idx = idx + time_seq[k];
              continue;
            }
            processFullCloudSegmentWithSnapshot(makeStateSnapshot(time_current));

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped_, tf_broadcaster_);
              if (RuntimeStatistics::instance().enabled()) {
                RuntimeStatistics::instance().recordMat(makeStateLogRecord());
              }
            }

            // Batch-LIO：每个 batch 的 IEKF posterior 都刷新到高频输出源，
            // 使 /OdometryHighFreq 来自 batch 校正状态而非整帧末状态。
            if (batch_enable && high_frequency_enable_) {
              seedHighFrequencyOdometry(false);
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (
                      get_time_sec(imu_next.header.stamp) < Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty())
                        break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output_, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output_, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input_, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  // time_current;
              }
              kf_input.predict(dt, Q_input_, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty())
                break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input_, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input_, input_in, true, false);

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            const auto update_start = statistics_enabled ? std::chrono::steady_clock::now()
                                                         : std::chrono::steady_clock::time_point{};
            const bool update_success = kf_input.update_iterated_dyn_share_modified();
            if (statistics_enabled) {
              const double update_time_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - update_start)
                  .count();
              RuntimeStatistics::instance().recordPoseUpdate(
                static_cast<uint64_t>(time_seq[k]), time_current, update_time_ms, update_success);
            }
            if (!update_success) {
              idx = idx + time_seq[k];
              continue;
            }
            processFullCloudSegmentWithSnapshot(makeStateSnapshot(time_current));

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped_, tf_broadcaster_);
              if (RuntimeStatistics::instance().enabled()) {
                RuntimeStatistics::instance().recordMat(makeStateLogRecord());
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (
                      get_time_sec(imu_next.header.stamp) < Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty())
                        break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input_, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  // time_current;
                }
                // kf_input.predict(dt, Q_input_, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped_, tf_broadcaster_);
      }

      seedHighFrequencyOdometry();

      /*** add the feature points to map ***/
      double map_incremental_ms = 0.0;
      bool map_incremental_called = false;
      if (feats_down_size > 4) {
        const double map_incremental_start = statistics_enabled ? omp_get_wtime() : 0.0;
        MapIncremental();
        if (statistics_enabled) {
          map_incremental_ms = (omp_get_wtime() - map_incremental_start) * 1000.0;
        }
        map_incremental_called = true;
      }
      if (statistics_enabled && map_incremental_called) {
        RuntimeStatistics::instance().recordMapIncremental(map_incremental_ms);
      }

      /******* Publish points *******/
      if (tf_send_en && !is_first_frame && time_current > 0.0) {
        // The last downsampled point/limited odometry may precede the full cloud stamp.
        // Extend TF to the scan end, independently of the public odometry rate limiter.
        auto end_state = makeStateSnapshot(time_current);
        const double dt = std::max(0.0, lidar_end_time - time_current);
        end_state.pos += end_state.vel * dt + 0.5 * end_state.acc * dt * dt;
        end_state.vel += end_state.acc * dt;
        const Eigen::Vector3d angle = end_state.omg * dt;
        end_state.rot = end_state.rot * Exp(angle.x(), angle.y(), angle.z());
        publish_robot_transform(robotKinematics(robot_extrinsics, end_state.pos,
          end_state.rot, end_state.vel, end_state.omg), lidar_end_time, tf_broadcaster_);
      }
      double path_publish_ms = 0.0;
      double cloud_publish_ms = 0.0;
      double full_cloud_publish_ms = 0.0;
      double map_publish_ms = 0.0;
      if (path_en) {
        const double publish_start = statistics_enabled ? omp_get_wtime() : 0.0;
        publish_path(pub_path_);
        if (statistics_enabled) {
          path_publish_ms = (omp_get_wtime() - publish_start) * 1000.0;
        }
      }
      if (scan_pub_en || pcd_save_en) {
        const double publish_start = statistics_enabled ? omp_get_wtime() : 0.0;
        publish_frame_world(pub_laser_cloud_full_res_);
        if (statistics_enabled) {
          cloud_publish_ms += (omp_get_wtime() - publish_start) * 1000.0;
        }
      }
      const double full_cloud_publish_start = statistics_enabled ? omp_get_wtime() : 0.0;
      if (has_full_cloud_anchor && lidar_end_time > full_cloud_anchor.stamp) {
        processFullCloudSegmentWithSnapshot(makeStateSnapshot(lidar_end_time));
      }
      publishAccumulatedFullCloudWorld(pub_laser_cloud_full_world_, lidar_end_time);
      if (statistics_enabled) {
        full_cloud_publish_ms = (omp_get_wtime() - full_cloud_publish_start) * 1000.0;
      }
      if (scan_pub_en && scan_body_pub_en) {
        const double publish_start = statistics_enabled ? omp_get_wtime() : 0.0;
        publish_frame_body(pub_laser_cloud_full_res_body_);
        if (statistics_enabled) {
          cloud_publish_ms += (omp_get_wtime() - publish_start) * 1000.0;
        }
      }
      if (scan_pub_en) {
        const double publish_start = statistics_enabled ? omp_get_wtime() : 0.0;
        publish_accumulated_map(pub_laser_cloud_map_);
        if (statistics_enabled) {
          map_publish_ms = (omp_get_wtime() - publish_start) * 1000.0;
        }
      }

      if (statistics_enabled) {
        const StateLogRecord state_log = makeStateLogRecord();
        if (!publish_odometry_without_downsample) {
          RuntimeStatistics::instance().recordMat(state_log);
        }
        RuntimeStatistics::instance().recordPos(state_log);

        const auto process_wall_end = std::chrono::steady_clock::now();
        FramePerformanceRecord performance;
        performance.lidar_beg_stamp = Measures.lidar_beg_time;
        performance.lidar_end_stamp = lidar_end_time;
        performance.process_ms =
          std::chrono::duration<double, std::milli>(process_wall_end - process_wall_start).count();
        performance.path_publish_ms = path_publish_ms;
        performance.cloud_publish_ms = cloud_publish_ms;
        performance.full_cloud_publish_ms = full_cloud_publish_ms;
        performance.map_publish_ms = map_publish_ms;
        performance.input_points = Measures.lidar ? Measures.lidar->size() : 0;
        performance.downsample_points = feats_down_size;
        performance.full_cloud_queue_points = full_cloud_queue.size();
        performance.global_map_points = global_map_ptr ? global_map_ptr->size() : 0;
        performance.pcd_wait_points = pcl_wait_save ? pcl_wait_save->size() : 0;
        performance.path_pose_count = path.poses.size();
        performance.effective_feature_points =
          effct_feat_num > 0 ? static_cast<std::size_t>(effct_feat_num) : 0;
        ++ivox_statistics_frame_counter_;
        if (ivox_ && ivox_statistics_frame_counter_ >= kIvoxStatisticsPeriodFrames) {
          const auto ivox_statistics_start = std::chrono::steady_clock::now();
          const std::vector<float> grid_statistics = ivox_->StatGridPoints();
          performance.ivox.collection_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - ivox_statistics_start)
                                             .count();
          performance.ivox.sampled = true;
          if (grid_statistics.size() >= 5) {
            performance.ivox.valid_grids = static_cast<std::size_t>(std::max(0.0F, grid_statistics[0]));
            performance.ivox.points_per_grid_avg = grid_statistics[1];
            performance.ivox.points_per_grid_max =
              static_cast<std::size_t>(std::max(0.0F, grid_statistics[2]));
            performance.ivox.points_per_grid_stddev = grid_statistics[4];
          }
          ivox_statistics_frame_counter_ = 0;
        }
        RuntimeStatistics::instance().recordFrame(performance);
      }
    }
    rate.sleep();
  }
}

}  // namespace point_lio

#ifndef POINT_LIO_BUILD_MAIN
RCLCPP_COMPONENTS_REGISTER_NODE(point_lio::LaserMappingNode)
#endif

#ifdef POINT_LIO_BUILD_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, SigHandle);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  auto node = std::make_shared<point_lio::LaserMappingNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
