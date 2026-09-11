#include <dac_sfc/dac_route_adapter.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace dac_sfc_deployment
{

void DacRouteAdapter::initialize(const GridMap::Ptr &map,
                                 const Eigen::Vector3i &astar_pool_size)
{
  map_ = map;
  astar_.reset(new AStar);
  astar_->initGridMap(map_, astar_pool_size);
}

bool DacRouteAdapter::lineIsFree(const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &end,
                                 const double clearance_radius) const
{
  if (!map_ || !start.allFinite() || !end.allFinite() ||
      !std::isfinite(clearance_radius) || clearance_radius < 0.0)
    return false;

  return map_->isInflatedLineClear(start, end, clearance_radius);
}

double DacRouteAdapter::pathLength(const std::vector<Eigen::Vector3d> &path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    length += (path[i] - path[i - 1]).norm();
  return length;
}

void DacRouteAdapter::removeConsecutiveDuplicates(std::vector<Eigen::Vector3d> &path)
{
  if (path.empty())
    return;

  std::vector<Eigen::Vector3d> clean;
  clean.reserve(path.size());
  clean.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    if ((path[i] - clean.back()).norm() > 1.0e-8)
      clean.push_back(path[i]);
  }
  path.swap(clean);
}

bool DacRouteAdapter::build(const Eigen::Vector3d &start,
                            const Eigen::Vector3d &goal,
                            const RouteOptions &options,
                            std::vector<Eigen::Vector3d> &raw_path,
                            std::vector<Eigen::Vector3d> &sparse_route,
                            RouteDiagnostics &diagnostics,
                            const bool allow_occupied_goal_adjustment)
{
  diagnostics = RouteDiagnostics();
  raw_path.clear();
  sparse_route.clear();

  if (!map_ || !astar_ || !start.allFinite() || !goal.allFinite() ||
      options.max_segment_length <= 0.0 || options.line_sample_step_ratio <= 0.0 ||
      !std::isfinite(options.clearance_radius) ||
      options.clearance_radius < 0.0)
  {
    diagnostics.failure_stage = "route_input";
    return false;
  }
  Eigen::Vector3d map_lower;
  Eigen::Vector3d map_upper;
  if (!map_->getInflatedMapBounds(map_lower, map_upper))
  {
    diagnostics.failure_stage = "route_map_not_ready";
    return false;
  }
  if (!map_->isInInflatedMap(start))
  {
    diagnostics.failure_stage = "route_start_outside_map";
    return false;
  }
  if (!map_->isInInflatedMap(goal))
  {
    diagnostics.failure_stage = "route_goal_outside_map";
    return false;
  }

  const int start_occupancy = map_->getInflateOccupancy(start);
  if (start_occupancy != 0)
  {
    diagnostics.failure_stage = "route_start_occupied";
    return false;
  }
  if (!map_->isInflatedPointClear(start, options.clearance_radius))
  {
    diagnostics.failure_stage = "route_start_clearance";
    return false;
  }

  Eigen::Vector3d route_goal = goal;
  const int goal_occupancy = map_->getInflateOccupancy(route_goal);
  if (!map_->isInflatedPointClear(route_goal, options.clearance_radius))
  {
    if (!allow_occupied_goal_adjustment)
    {
      diagnostics.failure_stage =
          goal_occupancy != 0 ? "route_goal_occupied" : "route_goal_clearance";
      return false;
    }

    const Eigen::Vector3d goal_direction = goal - start;
    const double goal_distance = goal_direction.norm();
    if (!std::isfinite(goal_distance) || goal_distance <= 1.0e-6)
    {
      diagnostics.failure_stage =
          goal_occupancy != 0 ? "route_goal_occupied" : "route_goal_clearance";
      return false;
    }

    const Eigen::Vector3d forward = goal_direction / goal_distance;
    const double search_step = std::max(map_->getResolution(), 1.0e-3);
    const double search_limit =
        std::max(options.max_segment_length, search_step);
    bool adjusted = false;
    for (double offset = search_step;
         offset <= search_limit + 1.0e-9;
         offset += search_step)
    {
      const Eigen::Vector3d candidate = goal + offset * forward;
      if (!map_->isInInflatedMap(candidate))
        break;
      if (map_->isInflatedPointClear(candidate, options.clearance_radius))
      {
        route_goal = candidate;
        diagnostics.goal_adjusted = true;
        diagnostics.goal_adjustment_distance = offset;
        adjusted = true;
        ROS_WARN_STREAM("[DAC-SFC] Unsafe local goal shifted forward by "
                        << offset << " m: " << goal.transpose()
                        << " -> " << route_goal.transpose());
        break;
      }
    }

    if (!adjusted)
    {
      diagnostics.failure_stage = "route_goal_adjustment_failed";
      return false;
    }
  }

  const auto search_started = std::chrono::steady_clock::now();
  diagnostics.direct_path =
      lineIsFree(start, route_goal, options.clearance_radius);
  if (diagnostics.direct_path)
  {
    raw_path.push_back(start);
    raw_path.push_back(route_goal);
  }
  else
  {
    const ASTAR_RET search_result =
        astar_->AstarSearch(map_->getResolution(), start, route_goal, true,
                            options.clearance_radius);
    if (search_result != ASTAR_RET::SUCCESS)
    {
      diagnostics.search_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - search_started)
                                  .count();
      diagnostics.failure_stage =
          search_result == ASTAR_RET::INIT_ERR ? "astar_init" : "astar_search";
      return false;
    }
    raw_path = astar_->getPath();
    if (raw_path.empty())
    {
      diagnostics.failure_stage = "astar_empty";
      return false;
    }

    // Preserve the A* lattice endpoints.  Replacing them directly with the
    // continuous start/goal can merge a short connector with the first or
    // last grid edge and cut across a nearby occupied voxel.
    if (!lineIsFree(start, raw_path.front(), options.clearance_radius))
    {
      ROS_WARN_STREAM("[DAC-SFC] A* start connector lacks clearance: "
                      << start.transpose() << " -> "
                      << raw_path.front().transpose()
                      << " clearance_m=" << options.clearance_radius);
      diagnostics.failure_stage = "astar_start_connector";
      return false;
    }
    if (!lineIsFree(raw_path.back(), route_goal, options.clearance_radius))
    {
      ROS_WARN_STREAM("[DAC-SFC] A* goal connector lacks clearance: "
                      << raw_path.back().transpose() << " -> "
                      << route_goal.transpose()
                      << " clearance_m=" << options.clearance_radius);
      diagnostics.failure_stage = "astar_goal_connector";
      return false;
    }

    if ((raw_path.front() - start).norm() > 1.0e-8)
      raw_path.insert(raw_path.begin(), start);
    else
      raw_path.front() = start;

    if ((raw_path.back() - route_goal).norm() > 1.0e-8)
      raw_path.push_back(route_goal);
    else
      raw_path.back() = route_goal;
  }
  diagnostics.search_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - search_started)
                              .count();

  removeConsecutiveDuplicates(raw_path);
  diagnostics.raw_point_count = static_cast<int>(raw_path.size());
  diagnostics.raw_length = pathLength(raw_path);
  if (raw_path.size() < 2)
  {
    diagnostics.failure_stage = "raw_route";
    return false;
  }

  const auto shortcut_started = std::chrono::steady_clock::now();
  sparse_route.push_back(raw_path.front());

  if (diagnostics.direct_path)
  {
    const int segment_count = std::max(
        1, static_cast<int>(std::ceil(
               (route_goal - start).norm() / options.max_segment_length)));
    for (int i = 1; i <= segment_count; ++i)
    {
      const double alpha = static_cast<double>(i) /
                           static_cast<double>(segment_count);
      sparse_route.push_back((1.0 - alpha) * start + alpha * route_goal);
    }
    diagnostics.shortcut_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - shortcut_started)
                                  .count();
    diagnostics.sparse_point_count = static_cast<int>(sparse_route.size());
    diagnostics.sparse_length = pathLength(sparse_route);
    diagnostics.success = true;
    return true;
  }

  std::size_t current = 0;
  while (current + 1 < raw_path.size())
  {
    std::size_t best = current;
    for (std::size_t candidate = current + 1; candidate < raw_path.size(); ++candidate)
    {
      if ((raw_path[candidate] - raw_path[current]).norm() >
          options.max_segment_length + 1.0e-9)
        break;
      if (lineIsFree(raw_path[current], raw_path[candidate],
                     options.clearance_radius))
        best = candidate;
    }

    if (best == current)
    {
      diagnostics.shortcut_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - shortcut_started)
                                    .count();
      ROS_WARN_STREAM("[DAC-SFC] Shortcut cannot reach the next raw point: index="
                      << current << "/" << raw_path.size()
                      << " from=" << raw_path[current].transpose()
                      << " next=" << raw_path[current + 1].transpose()
                      << " clearance_m=" << options.clearance_radius);
      diagnostics.failure_stage = "shortcut_visibility";
      return false;
    }

    sparse_route.push_back(raw_path[best]);
    current = best;
  }

  diagnostics.shortcut_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - shortcut_started)
                                .count();
  diagnostics.sparse_point_count = static_cast<int>(sparse_route.size());
  diagnostics.sparse_length = pathLength(sparse_route);
  diagnostics.success = sparse_route.size() >= 2;
  if (!diagnostics.success)
    diagnostics.failure_stage = "sparse_route";
  return diagnostics.success;
}

} // namespace dac_sfc_deployment
