#include <dac_sfc/dac_sfc_engine.h>

#include <iostream>

int main()
{
  std::vector<Eigen::Vector3d> route;
  route.push_back(Eigen::Vector3d(-1.0, 0.0, 1.0));
  route.push_back(Eigen::Vector3d(0.0, 0.2, 1.0));
  route.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));

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
    std::cerr << "DAC-SFC engine self-test failed at: "
              << result.diagnostics.failure_stage << std::endl;
    return 1;
  }
  if (result.diagnostics.corridor_count != 2 ||
      result.coefficients.size() != 2 || result.durations.size() != 2)
  {
    std::cerr << "Unexpected one-route-segment/one-corridor mapping." << std::endl;
    return 2;
  }

  std::cout << "DAC-SFC engine self-test passed: corridors="
            << result.diagnostics.corridor_count
            << " faces=" << result.diagnostics.total_faces
            << " duration=" << result.diagnostics.trajectory_duration << std::endl;
  return 0;
}
