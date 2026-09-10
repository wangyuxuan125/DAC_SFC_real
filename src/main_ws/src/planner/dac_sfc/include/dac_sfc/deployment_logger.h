#ifndef DAC_SFC_DEPLOYMENT_LOGGER_H
#define DAC_SFC_DEPLOYMENT_LOGGER_H

#include <dac_sfc/dac_route_adapter.h>
#include <dac_sfc/dac_sfc_engine.h>

#include <cstdint>
#include <mutex>
#include <string>

namespace dac_sfc_deployment
{

struct DeploymentRecord
{
  double timestamp_s = 0.0;
  std::uint64_t run_id = 0;
  std::string data_source = "simulation";
  std::string execution_mode = "shadow";
  bool pipeline_success = false;
  bool trajectory_activated = false;
  bool ego_fallback_used = false;
  bool sampled_collision_free = false;
  std::string failure_stage;
  double obstacle_extract_ms = 0.0;
  double total_ms = 0.0;
  RouteDiagnostics route;
  EngineDiagnostics engine;
};

class DeploymentLogger
{
public:
  void configure(bool enabled, const std::string &directory);
  bool append(const DeploymentRecord &record);
  const std::string &path() const { return path_; }

private:
  bool enabled_ = false;
  std::string directory_;
  std::string path_;
  std::mutex mutex_;
};

} // namespace dac_sfc_deployment

#endif
