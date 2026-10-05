#include <gtest/gtest.h>
#include <rog_map_ros/cloud_registered_crop_filter.hpp>
#include <Eigen/Geometry>

TEST(BodyCropFilter, RotatedBodyIsRemovedAndExternalGroundPreserved)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto buffer = std::make_shared<tf2_ros::Buffer>(clock);
  rog_map::Config config;
  config.cloud_filter_position_frame = "base_link";
  config.cloud_filter_mode = "transform_cloud";
  config.cloud_filter_z_filter_enable = false;
  config.cloud_filter_center_z = 0.7;
  config.cloud_filter_size_x = config.cloud_filter_size_y = 0.7;
  config.cloud_filter_size_z = 1.4;
  const Eigen::Quaterniond rotation(
    Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d translation(3., 4., 2.);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "camera_init";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = translation.x();
  tf.transform.translation.y = translation.y();
  tf.transform.translation.z = translation.z();
  tf.transform.rotation.x = rotation.x(); tf.transform.rotation.y = rotation.y();
  tf.transform.rotation.z = rotation.z(); tf.transform.rotation.w = rotation.w();
  ASSERT_TRUE(buffer->setTransform(tf, "test", true));
  rog_map::CloudRegisteredCropFilter filter(config, buffer, clock,
    rclcpp::get_logger("body_crop_test"), nullptr);
  rog_map::PointCloud cloud;
  auto insert = [&](const Eigen::Vector3d & body_point) {
    const Eigen::Vector3d world_point = translation + rotation * body_point;
    pcl::PointXYZI point;
    point.x = world_point.x(); point.y = world_point.y(); point.z = world_point.z();
    point.intensity = static_cast<float>(cloud.size());
    cloud.push_back(point);
  };
  insert(Eigen::Vector3d(0.2, -0.2, 1.2));  // Inside the tilted body.
  insert(Eigen::Vector3d(0.36, 0., 0.2));  // Close obstacle outside the box.
  insert(Eigen::Vector3d(1., 0., 0.));     // Ground below the robot's world Z.
  insert(Eigen::Vector3d(0., 0., -0.1));   // Surface below the body's bottom.
  const auto original = cloud;
  std_msgs::msg::Header header;
  header.frame_id = "camera_init";
  ASSERT_TRUE(filter.filter(cloud, header, translation));
  ASSERT_EQ(cloud.size(), 3U);
  for (size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_FLOAT_EQ(cloud[i].x, original[i + 1].x);
    EXPECT_FLOAT_EQ(cloud[i].y, original[i + 1].y);
    EXPECT_FLOAT_EQ(cloud[i].z, original[i + 1].z);
    EXPECT_FLOAT_EQ(cloud[i].intensity, original[i + 1].intensity);
  }
}
