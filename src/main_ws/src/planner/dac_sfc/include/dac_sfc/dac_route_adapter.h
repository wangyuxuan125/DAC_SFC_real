#ifndef DAC_ROUTE_ADAPTER_H
#define DAC_ROUTE_ADAPTER_H

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

#include <memory>
#include <string>
#include <vector>

namespace dac_sfc_deployment
{

struct RouteOptions
{
  double max_segment_length = 1.0;
  double line_sample_step_ratio = 0.5;
};

struct RouteDiagnostics
{
  bool success = false;
  bool direct_path = false;
  bool goal_adjusted = false;
  double goal_adjustment_distance = 0.0;
  std::string failure_stage;
  int raw_point_count = 0;
  int sparse_point_count = 0;
  double raw_length = 0.0;
  double sparse_length = 0.0;
  double search_ms = 0.0;
  double shortcut_ms = 0.0;
};

class DacRouteAdapter
{
public:
  using Ptr = std::shared_ptr<DacRouteAdapter>;

  void initialize(const GridMap::Ptr &map,
                  const Eigen::Vector3i &astar_pool_size);

  bool build(const Eigen::Vector3d &start,
             const Eigen::Vector3d &goal,
             const RouteOptions &options,
             std::vector<Eigen::Vector3d> &raw_path,
             std::vector<Eigen::Vector3d> &sparse_route,
             RouteDiagnostics &diagnostics,
             bool allow_occupied_goal_adjustment = true);

private:
  bool lineIsFree(const Eigen::Vector3d &start,
                  const Eigen::Vector3d &end,
                  double sample_step_ratio) const;

  static double pathLength(const std::vector<Eigen::Vector3d> &path);
  static void removeConsecutiveDuplicates(std::vector<Eigen::Vector3d> &path);

  GridMap::Ptr map_;
  AStar::Ptr astar_;
};

} // namespace dac_sfc_deployment

#endif
