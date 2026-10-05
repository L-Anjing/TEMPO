#include <rog_map/rog_map.h>
#include <iomanip>

namespace rog_map {
using namespace super_utils;
void ROGMap::writeMapDiagnostics(std::ostream & out, const Vec3f & target) const
{
  size_t unknown = 0, free = 0, occupied = 0;
  for (float value : occupancy_buffer_) {
    const auto type = classifyProb(value);
    unknown += type == UNKNOWN; free += type == KNOWN_FREE; occupied += type == OCCUPIED;
  }
  auto counts = [&](const char * name, size_t n, size_t u, size_t f, size_t o) {
    const double scale = n ? 100.0 / n : 0.0;
    out << name << " total=" << n << " unknown=" << u << "(" << u * scale << "%) free="
        << f << "(" << f * scale << "%) occupied=" << o << "(" << o * scale << "%)\n";
  };
  counts("ProbMap", occupancy_buffer_.size(), unknown, free, occupied);
  const auto inf = inf_map_->diagnosticStats();
  counts("CounterMap", inf.total, inf.counter_unknown, inf.counter_free, inf.counter_occupied);
  counts("InfMap", inf.total, inf.inf_unknown, inf.inf_free, inf.inf_occupied);
  out << "InfMap inflated_only=" << inf.inflated_only << " unknown_inflation_enabled="
      << cfg_.unk_inflation_en << " (disabled: unoccupied collision mask is FREE, including unknown)\n";
  if (!layer_ || layer_->empty()) {
    out << "projection/terrain/query: not initialized (no map update)\n";
    return;
  }
  size_t pu = 0, pf = 0, po = 0, passable = 0;
  for (const auto & cell : layer_->cells()) {
    pu += cell.type == CellType::UNKNOWN; pf += cell.type == CellType::FREE;
    po += cell.type == CellType::OCCUPIED; passable += cell.type == CellType::PASSABLE;
  }
  counts("Projection", layer_->cells().size(), pu, pf + passable, po);
  out << "Projection passable=" << passable << "\n";
  size_t tu = 0, tf = 0, to = 0;
  for (uint8_t value : terrain_layer_.values()) { tu += value == 255; tf += value < 253; to += value == 254; }
  counts("Terrain", terrain_layer_.values().size(), tu, tf, to);

  Vec3i target_id; posToGlobalIndex(target, target_id);
  out << std::setprecision(9) << "trace world=" << target.transpose() << " global_xy="
      << target_id.head<2>().transpose() << " update_time=" << current_update_time_ << "\n";
  for (int z = projection_z_min_id_; z <= projection_z_max_id_; ++z) {
    const Vec3i id(target_id.x(), target_id.y(), z);
    if (!insideLocalMap(id)) { continue; }
    Vec3f center; globalIndexToPos(id, center);
    const int hash = getHashIndexFromGlobalIndex(id);
    const double value = occupancy_buffer_[hash];
    const auto counter = inf_map_->diagnosticCounters(center);
    const auto it = diagnostic_origin_updates_.find(z);
    const bool origin = target_id.x() == 0 && target_id.y() == 0;
    const int hits = origin && it != diagnostic_origin_updates_.end() ? it->second.first : 0;
    const int misses = origin && it != diagnostic_origin_updates_.end() ? it->second.second : 0;
    out << " voxel=" << id.transpose() << " center=" << center.transpose()
        << " logodds=" << value << " probability=" << 1.0 / (1.0 + std::exp(-value))
        << " raw=" << GridTypeStr[classifyProb(value)] << " hits=" << hits << " misses=" << misses
        << " last_hit=" << last_hit_time_[hash] << " last_update=" << last_update_time_[hash]
        << " counter=" << GridTypeStr[counter.state] << " counter_occ=" << counter.occupied
        << " counter_unknown=" << counter.unknown << " inflation_count=" << counter.inflated;
    if (counter.inflated > 0) {
      Vec3i inf_id; inf_map_->infMapPosToGlobalIndex(center, inf_id);
      bool found = false;
      for (const auto & delta : cfg_.inf_spherical_neighbor) {
        const Vec3i source = inf_id - delta;
        Vec3f source_pos; inf_map_->infMapGlobalIndexToPos(source, source_pos);
        if (inf_map_->diagnosticCounters(source_pos).state == OCCUPIED) {
          out << " inflation_source=" << source_pos.transpose() << " distance=" << (source_pos - center).norm();
          found = true; break;
        }
      }
      if (!found) { out << " inflation_source=NONE_INCONSISTENT_COUNTER"; }
    }
    out << "\n";
  }
  const int mx = static_cast<int>(std::floor((target.x() - layer_->origin().x()) / layer_->resolution()));
  const int my = static_cast<int>(std::floor((target.y() - layer_->origin().y()) / layer_->resolution()));
  if (mx < 0 || my < 0 || mx >= layer_->width() || my >= layer_->height()) {
    out << "trace cell outside projection\n"; return;
  }
  const size_t index = static_cast<size_t>(my) * layer_->width() + mx;
  const auto & projection = layer_->cells()[index];
  out << " projection_cell=" << mx << "," << my << " raw_type=" << static_cast<int>(projection.raw_type)
      << " final_type=" << static_cast<int>(projection.type) << " reason=" << static_cast<int>(projection.raw_reason)
      << " confidence=" << projection.confidence << " occupied_z_min=" << projection.occupied_z_min_abs
      << " occupied_z_max=" << projection.occupied_z_max_abs << " span=" << projection.height_delta << "\n";
  if (index < terrain_layer_.cells().size()) {
    const auto & cell = terrain_layer_.cells()[index];
    out << " terrain=" << static_cast<int>(cell.type) << " support_observed=" << cell.support_observed
        << " surface_z=" << cell.surface_z << " terrain_cost=" << static_cast<int>(terrain_layer_.values()[index]) << "\n";
  }
  if (query_) { out << " planner_query_cost=" << static_cast<int>(query_->value(mx, my)) << "\n"; }
  size_t blocked = 0, unknown_cells = 0, traversable = 0, outside = 0;
  for (int dy = -2; dy <= 2; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      const int x = mx + dx, y = my + dy;
      if (x < 0 || y < 0 || x >= layer_->width() || y >= layer_->height()) {
        ++outside; continue;
      }
      const size_t neighbor = static_cast<size_t>(y) * layer_->width() + x;
      if (neighbor >= terrain_layer_.values().size()) { ++outside; continue; }
      const uint8_t cost = terrain_layer_.values()[neighbor];
      if (cost == 255) { ++unknown_cells; }
      else if (cost >= 253) { ++blocked; }
      else { ++traversable; }
    }
  }
  out << " terrain_5x5 blocked=" << blocked << " unknown=" << unknown_cells
      << " traversable=" << traversable << " outside=" << outside << "\n";
}
}  // namespace rog_map
