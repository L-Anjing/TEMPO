#include <rog_map/rog_map.h>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>
#include <regex>

using namespace rog_map;
using namespace super_utils;

void flattenParameters(const YAML::Node & yaml, const std::string & prefix,
  std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & item : yaml) {
    const std::string key = prefix + "." + item.first.as<std::string>();
    const auto & value = item.second;
    if (value.IsMap()) { flattenParameters(value, key, parameters); }
    else if (value.IsSequence()) { parameters.emplace_back(key, value.as<std::vector<double>>()); }
    else {
      const std::string text = value.as<std::string>();
      if (text == "true" || text == "false") { parameters.emplace_back(key, text == "true"); }
      else if (std::regex_match(text, std::regex("[-+]?[0-9]+"))) {
        parameters.emplace_back(key, static_cast<int64_t>(std::stoll(text)));
      } else if (std::regex_match(text, std::regex("[-+]?[0-9]+\\.[0-9]+([eE][-+]?[0-9]+)?"))) {
        parameters.emplace_back(key, std::stod(text));
      } else { parameters.emplace_back(key, text); }
    }
  }
}

class PipelineProbe : public ROGMap {
public:
  explicit PipelineProbe(const Config & config) { cfg_ = config; init(); }
  const double getSystemWalltimeNow() override { return 10.0; }
  void odomOnly() { updateRobotState({Vec3f::Zero(), Quatf::Identity()}); }
  void inspectDerivedLayers() { setUpdateTime(10.0); refreshLayers(); refreshQuery(); }
  GridType rawState(const Vec3f & pos) const {
    return classifyProb(occupancy_buffer_[getHashIndexFromPos(pos)]);
  }
  bool checkProbabilityUpdates() {
    const Vec3f pos(.025, .025, .275);
    const int hash = getHashIndexFromPos(pos);
    const float initial = occupancy_buffer_[hash];
    missPointUpdate(pos, hash, 1);
    const float after_miss = occupancy_buffer_[hash];
    const bool weak_miss_is_unknown = classifyProb(after_miss) == GridType::UNKNOWN;
    hitPointUpdate(pos, hash, 1);
    const float after_hit = occupancy_buffer_[hash];
    hitPointUpdate(pos, hash, 1000000);
    const float upper = occupancy_buffer_[hash];
    missPointUpdate(pos, hash, 1000000);
    const float lower = occupancy_buffer_[hash];
    std::cout << "UPDATE_BOUNDS initial=" << initial << " after_miss=" << after_miss
      << " after_hit=" << after_hit << " lower=" << lower << " upper=" << upper << "\n";
    return weak_miss_is_unknown && after_miss < initial && after_hit > after_miss &&
      upper == cfg_.l_max && lower == cfg_.l_min;
  }
  bool originHasNoSupport() const {
    if (!layer_ || layer_->empty()) { return true; }
    const int x = static_cast<int>(std::floor(-layer_->origin().x() / layer_->resolution()));
    const int y = static_cast<int>(std::floor(-layer_->origin().y() / layer_->resolution()));
    const auto & cell = terrain_layer_.cells().at(y * layer_->width() + x);
    return !cell.support_observed && !std::isfinite(cell.surface_z) && cell.type == TerrainType::UNKNOWN;
  }
  bool checkOccupiedPositions(bool expect_empty, double wall_height = 1.05) const {
    size_t occupied = 0, off_wall = 0, wrong_round_trip = 0;
    Vec3f minimum = Vec3f::Constant(INFINITY), maximum = -minimum;
    for (size_t hash = 0; hash < occupancy_buffer_.size(); ++hash) {
      Vec3i id; hashIdToGlobalIndex(static_cast<int>(hash), id);
      if (getHashIndexFromGlobalIndex(id) != static_cast<int>(hash)) { ++wrong_round_trip; }
      if (classifyProb(occupancy_buffer_[hash]) != GridType::OCCUPIED) { continue; }
      Vec3f point; globalIndexToPos(id, point);
      minimum = minimum.cwiseMin(point); maximum = maximum.cwiseMax(point); ++occupied;
      if (std::abs(point.x() - 2.025) > .026 || std::abs(point.y()) > .45 ||
          point.z() < 0 || point.z() > wall_height) { ++off_wall; }
    }
    std::cout << "OCC_GEOMETRY count=" << occupied << " bbox_min=" << minimum.transpose()
      << " bbox_max=" << maximum.transpose() << " off_wall=" << off_wall
      << " index_round_trip_errors=" << wrong_round_trip << "\n";
    return wrong_round_trip == 0 && off_wall == 0 && (expect_empty ? occupied == 0 : occupied > 0);
  }
};

int main(int argc, char ** argv)
{
  const std::string mode = argc > 1 ? argv[1] : "zero";
  rclcpp::init(argc, argv);
  const auto yaml = YAML::LoadFile(std::string(ROOT_DIR) + "../navigation/params/online_planner.yaml");
  std::vector<rclcpp::Parameter> overrides;
  flattenParameters(yaml["planner_server"]["ros__parameters"]["MincoPlanner"]["rog_map"], "map", overrides);
  if (mode == "config_guard") {
    bool pass = true;
    const std::vector<std::pair<std::string, double>> invalid{
      {"p_miss", .5}, {"p_miss", .7}, {"p_miss", 0.0},
      {"p_miss", std::numeric_limits<double>::quiet_NaN()},
      {"p_hit", .5}, {"p_hit", 1.0},
      {"p_min", .499}, {"p_free", .5}, {"p_occ", .98}, {"p_max", 1.0}};
    for (size_t i = 0; i < invalid.size(); ++i) {
      auto params = overrides;
      const auto key = "map.raycasting." + invalid[i].first;
      params.erase(std::remove_if(params.begin(), params.end(),
        [&](const auto & param) { return param.get_name() == key; }), params.end());
      params.emplace_back(key, invalid[i].second);
      auto node = std::make_shared<rclcpp::Node>("config_guard_" + std::to_string(i),
        rclcpp::NodeOptions().parameter_overrides(params));
      bool rejected = false;
      try { Config candidate; candidate.loadFromRosNode(node, "map"); }
      catch (const std::invalid_argument & error) {
        rejected = std::string(error.what()).find("invalid raycasting probabilities") != std::string::npos;
      }
      std::cout << "CONFIG_GUARD " << key << "=" << invalid[i].second
        << " rejected=" << rejected << "\n";
      pass = pass && rejected;
    }
    // Also exercise the defaults with no probability overrides.
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(), [](const auto & param) {
      return param.get_name().find("map.raycasting.p_") == 0;
    }), overrides.end());
    auto node = std::make_shared<rclcpp::Node>("config_defaults",
      rclcpp::NodeOptions().parameter_overrides(overrides));
    Config defaults; defaults.loadFromRosNode(node, "map");
    pass = pass && defaults.p_miss < .5f && defaults.l_miss < 0;
    std::cout << "RESULT config_guard " << (pass ? "PASS" : "FAIL") << "\n";
    rclcpp::shutdown(); return pass ? 0 : 1;
  }
  auto node = std::make_shared<rclcpp::Node>("map_pipeline_probe", rclcpp::NodeOptions().parameter_overrides(overrides));
  Config config; config.loadFromRosNode(node, "map");
  // Test harness uses one external writer and a known sensor origin; production YAML is untouched.
  config.ros_callback_en = false;
  config.performance_enable = false;
  config.sensor_in_robot = Vec3f(0, 0, .2634);
  config.sensor_in_robot_q = Quatf::Identity();
  config.debug_pipeline_enable = true;
  if (mode.find("miss_sign") != std::string::npos) {
    // Causal contrast only: production configuration and implementation stay unchanged.
    config.p_miss = .45f;
    config.l_miss = std::log(config.p_miss / (1.0f - config.p_miss));
  }
  std::cout << "PROBABILITY_MODEL p_hit=" << config.p_hit << " p_miss=" << config.p_miss
    << " p_free=" << config.p_free << " p_occ=" << config.p_occ
    << " l_hit=" << config.l_hit << " l_miss=" << config.l_miss << "\n";
  PipelineProbe map(config); map.odomOnly();
  bool pass = true;
  if (mode == "update_bounds") {
    pass = map.checkProbabilityUpdates();
  } else if (mode == "zero") {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // The production zero-cloud case has no published query snapshot. Force only the
    // derived readout here to test that scanning initialized UNKNOWN cannot create support.
    map.inspectDerivedLayers();
    pass = map.checkOccupiedPositions(true) && map.originHasNoSupport();
  } else {
    PointCloud cloud;
    const int vertical_layers = mode.find("start_trace") != std::string::npos ? 160 : 20;
    // The map initializes halfway between l_free and l_occ. With p_miss=0.49,
    // one retained ray is insufficient to classify every traversed voxel as FREE.
    const int required_misses = static_cast<int>(std::ceil(
      ((config.l_occ + config.l_free) * .5 - config.l_free) / -config.l_miss)) + 1;
    const int repeats = std::max(6, required_misses * config.point_filt_num * 2);
    for (int y = -8; y <= 8; ++y) for (int z = 0; z <= vertical_layers; ++z) {
      pcl::PointXYZI point;
      point.x = 2.025f; point.y = y * .05f + .025f; point.z = z * .05f + .025f;
      point.intensity = 1.0f;
      // Repeated samples stay within the same wall voxel; there is still one scan/update.
      // This is one dense wall scan and still exactly one updateMap() call.
      for (int repeat = 0; repeat < repeats; ++repeat) {
        point.x = 2.025f + (repeat % 6) * .001f; cloud.push_back(point);
      }
    }
    std::cout << "ACTUAL_SYNTHETIC_INPUT points=" << cloud.size()
      << " frame=camera_init stamp=10 sensor=(0,0,0.2634) x=2.025"
      << " required_misses=" << required_misses << " returns_per_wall_voxel=" << repeats << "\n";
    map.updateMap(cloud, {Vec3f::Zero(), Quatf::Identity()});
    pass = map.checkOccupiedPositions(false, vertical_layers * .05 + .05) && map.originHasNoSupport();
    for (const auto & check : std::vector<std::pair<Vec3f,GridType>>{
      {Vec3f(1.025, .025, .275), GridType::KNOWN_FREE},
      {Vec3f(2.025, .025, .325), GridType::OCCUPIED},
      {Vec3f(-2, -2, .3), GridType::UNKNOWN}}) {
      const auto actual = map.rawState(check.first);
      std::cout << "CHECK position=" << check.first.transpose() << " expected="
        << GridTypeStr[check.second] << " actual=" << GridTypeStr[actual] << "\n";
      pass = pass && actual == check.second;
    }
  }
  map.writeMapDiagnostics(std::cout);
  std::cout << "RESULT " << mode << " " << (pass ? "PASS" : "FAIL") << "\n";
  rclcpp::shutdown();
  return pass ? 0 : 1;
}
