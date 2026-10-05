#include <gtest/gtest.h>
#include <thread>
#include "minco_core/minco_planner.hpp"

namespace minco_planner {
namespace {
class TestMap : public rog_map::MapQueryInterface
{
public:
  bool blocked{false};
  bool worldToMap(double x, double y, unsigned int & mx, unsigned int & my) const override
  { mx = static_cast<unsigned int>((x + 20.0) * 10.0);
    my = static_cast<unsigned int>((y + 20.0) * 10.0); return true; }
  void mapToWorld(unsigned int mx, unsigned int my, double & x, double & y) const override
  { x = mx * .1 - 20.0; y = my * .1 - 20.0; }
  unsigned int sizeX() const override { return 400; }
  unsigned int sizeY() const override { return 400; }
  double resolution() const override { return .1; }
  double originX() const override { return -20.0; }
  double originY() const override { return -20.0; }
  uint8_t value(unsigned int mx, unsigned int) const override
  { return blocked && mx >= 238U ? 254U : 0U; }
  const unsigned char * values() const override { return nullptr; }
  bool isValid(unsigned int mx, unsigned int my) const override { return mx < 400 && my < 400; }
  bool isFree(unsigned int mx, unsigned int my) const override { return value(mx, my) == 0; }
  bool evaluate(const Eigen::Vector3d &, double & distance, Eigen::Vector3d & gradient) const override
  { distance = 100.0; gradient.setZero(); return true; }
};
geometry_msgs::msg::PoseStamped pose(double x, double y = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "camera_init";
  p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1.0;
  return p;
}
}  // namespace

class PlanReuseTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("plan_reuse_test");
    planner = std::make_shared<MincoPlanner>();
    map = std::make_shared<TestMap>();
    planner->clock_ = node->get_clock();
    planner->active_.store(true);
    planner->is_traj_safe_.store(true);
    planner->rog_query_raw_ = map;
    const auto result_qos = rclcpp::QoS(1).reliable().transient_local();
    planner->plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("test_plan", result_qos);
    planner->plan_meta_pub_ = node->create_publisher<navigation::msg::PlanMeta>("test_plan_meta", result_qos);
    planner->output_frame_ = "camera_init";
    planner->planning_path_id_ = 42;
    cache(0.0, 4.0);
  }
  void cache(double start, double end)
  {
    planner->cached_plan_.poses.clear();
    planner->cached_plan_s_.clear();
    planner->cached_plan_.header.frame_id = "camera_init";
    for (int i = 0; i <= 80; ++i) {
      planner->cached_plan_.poses.push_back(pose(start + (end - start) * i / 80.0));
      planner->cached_plan_s_.push_back((end - start) * i / 80.0);
    }
    planner->cached_progress_s_ = 0.0;
    planner->cached_plan_goal_generation_ = planner->pending_goal_generation_.load();
  }
  double progress() const { return planner->cached_progress_s_; }
  std::string reason() const { return planner->last_path_block_info_.reason; }
  bool geometryBlocked() const { return planner->last_path_block_info_.geometry_block; }
  bool retained() const { return planner->cached_plan_.poses.size() >= 2U; }
  uint64_t sequence() const { return planner->planning_publish_seq_; }
  uint64_t pathId() const { return planner->planning_path_id_; }
  void elapsed(double seconds)
  { planner->input_unavailable_since_ = std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds)); }
  void missionEndpoint() { planner->cached_plan_reaches_goal_ = true; }
  void changeGoal() { ++planner->pending_goal_generation_; }
  rclcpp::Node::SharedPtr node;
  MincoPlanner::Ptr planner;
  std::shared_ptr<TestMap> map;
};

TEST_F(PlanReuseTest, SafetyValidationDetectsRollingEndAndForwardOverrun)
{
  EXPECT_TRUE(planner->reusePlan(pose(3.4), false));
  EXPECT_TRUE(planner->localHorizonExhausted());
  EXPECT_EQ(reason(), "LOCAL_HORIZON_EXHAUSTED");
  EXPECT_FALSE(geometryBlocked());
  EXPECT_NEAR(progress(), 3.4, 1e-6);
  EXPECT_TRUE(planner->reusePlan(pose(4.8, .1), false));
  EXPECT_TRUE(planner->localHorizonExhausted());
  EXPECT_NEAR(progress(), 4.0, 1e-6);
  EXPECT_EQ(pathId(), 42U);
  EXPECT_EQ(sequence(), 0U);
  // Emulate acceptance of the next cropped local window; its progress starts at zero.
  cache(3.4, 7.4);
  EXPECT_TRUE(planner->reusePlan(pose(4.8), false));
  EXPECT_FALSE(planner->localHorizonExhausted());
  EXPECT_NEAR(progress(), 1.4, 1e-6);
}

TEST_F(PlanReuseTest, LateralDeviationIsNotHorizonExhaustion)
{
  EXPECT_FALSE(planner->reusePlan(pose(4.8, .8), false));
  EXPECT_EQ(reason(), "ROBOT_OFF_PATH");
  EXPECT_FALSE(geometryBlocked());
  EXPECT_FALSE(planner->localHorizonExhausted());
}

TEST_F(PlanReuseTest, MissionEndpointRemainsBounded)
{
  missionEndpoint();
  EXPECT_FALSE(planner->reusePlan(pose(4.8), false));
  EXPECT_EQ(reason(), "ROBOT_OFF_PATH");
}

TEST_F(PlanReuseTest, RealObstacleTakesPrecedenceAtRollingEnd)
{
  map->blocked = true;
  EXPECT_FALSE(planner->reusePlan(pose(3.4), false));
  EXPECT_TRUE(geometryBlocked());
  EXPECT_EQ(reason(), "CELL_OCCUPIED");
  EXPECT_FALSE(planner->localHorizonExhausted());
  planner->publishPlanningStatus("obstacle_entered_corridor");
  EXPECT_FALSE(retained());
  EXPECT_EQ(sequence(), 1U);
}

TEST_F(PlanReuseTest, InputGapRetainsPathIdAndProgressUntilExpiry)
{
  ASSERT_TRUE(planner->reusePlan(pose(2.0), false));
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  elapsed(.8);
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  EXPECT_TRUE(retained());
  EXPECT_NEAR(progress(), 2.0, 1e-6);
  EXPECT_EQ(pathId(), 42U);
  EXPECT_EQ(sequence(), 0U);
  elapsed(1.1);
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  EXPECT_FALSE(retained());
  EXPECT_FALSE(planner->isTrajSafe());
  EXPECT_EQ(sequence(), 1U);
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  EXPECT_EQ(sequence(), 1U);
}

TEST_F(PlanReuseTest, RecoveryResetsGraceAndAllowsNormalReuse)
{
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  elapsed(.8);
  planner->inputAvailable();
  EXPECT_TRUE(planner->reusePlan(pose(1.0), true));
  EXPECT_EQ(sequence(), 1U);
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  EXPECT_TRUE(retained());
  EXPECT_EQ(sequence(), 1U);
}

TEST_F(PlanReuseTest, CancellationAndGoalChangeBypassGrace)
{
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  planner->publishPlanningStatus("cancelled");
  EXPECT_FALSE(retained());
  EXPECT_EQ(sequence(), 1U);
  cache(0.0, 4.0);
  changeGoal();
  planner->publishPlanningStatus("INPUT_UNAVAILABLE");
  EXPECT_FALSE(retained());
  EXPECT_EQ(sequence(), 2U);
}

TEST_F(PlanReuseTest, LateSubscriberReceivesCurrentPlanAndLatestRetraction)
{
  ASSERT_TRUE(planner->reusePlan(pose(1.0), true));
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto receive_latest = [&]() {
    bool received = false;
    size_t count = 0;
    auto subscription = node->create_subscription<nav_msgs::msg::Path>("test_plan", qos,
      [&](const nav_msgs::msg::Path & path) { received = true; count = path.poses.size(); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!received && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(received);
    return count;
  };
  EXPECT_GT(receive_latest(), 0U);
  EXPECT_EQ(sequence(), 1U);  // Late subscription does not republish or create a new path ID.
  EXPECT_EQ(pathId(), 42U);
  planner->publishPlanningStatus("cancelled");
  EXPECT_EQ(receive_latest(), 0U);  // Old valid reference must not survive an explicit cancellation.
  EXPECT_EQ(sequence(), 2U);
}
}  // namespace minco_planner
