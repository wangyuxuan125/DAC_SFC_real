#include <dac_sfc/dac_sfc_engine.h>

#include <iostream>
#include <string>
#include <vector>

namespace
{

bool runCase(const std::string &label,
             const std::vector<Eigen::Vector3d> &route,
             const int expected_corridors,
             const int expected_valid_csgn_metrics)
{
  Eigen::Matrix3d initial_pva = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d terminal_pva = Eigen::Matrix3d::Zero();
  initial_pva.col(0) = route.front();
  terminal_pva.col(0) = route.back();

  dac_sfc_deployment::EngineOptions options;
  options.max_velocity = 2.0;
  options.max_extra_radius = 0.8;
  options.overlap_radius = 0.04;

  dac_sfc_deployment::EngineResult result;
  dac_sfc_deployment::DacSfcEngine engine;
  const bool success = engine.plan(
      route, std::vector<Eigen::Vector3d>(), Eigen::Vector3d(-3.0, -3.0, 0.0),
      Eigen::Vector3d(3.0, 3.0, 3.0), initial_pva, terminal_pva, options,
      result);

  if (!success)
  {
    std::cerr << label << " failed at: "
              << result.diagnostics.failure_stage << std::endl;
    return false;
  }

  if (result.diagnostics.corridor_count != expected_corridors ||
      static_cast<int>(result.coefficients.size()) != expected_corridors ||
      result.durations.size() != expected_corridors)
  {
    std::cerr << label << " has an unexpected segment/corridor mapping."
              << std::endl;
    return false;
  }

  if (result.diagnostics.valid_csgn_metrics !=
      expected_valid_csgn_metrics)
  {
    std::cerr << label << " has an unexpected CSGN metric count: "
              << result.diagnostics.valid_csgn_metrics << std::endl;
    return false;
  }

  std::cout << label << " passed: corridors="
            << result.diagnostics.corridor_count
            << " faces=" << result.diagnostics.total_faces
            << " valid_csgn="
            << result.diagnostics.valid_csgn_metrics
            << " duration=" << result.diagnostics.trajectory_duration
            << std::endl;
  return true;
}

} // namespace

int main()
{
  const std::vector<Eigen::Vector3d> multi_piece_route{
      Eigen::Vector3d(-1.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.2, 1.0),
      Eigen::Vector3d(1.0, 0.0, 1.0)};
  if (!runCase("multi-piece CSGN", multi_piece_route, 2, 2))
    return 1;

  const std::vector<Eigen::Vector3d> single_piece_route{
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.075, 0.0, 1.0)};
  if (!runCase("single-piece isotropic fallback",
               single_piece_route, 1, 0))
    return 2;

  return 0;
}
