#include <dac_sfc/deployment_logger.h>

#include <cerrno>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace dac_sfc_deployment
{
namespace
{

bool ensureDirectory(const std::string &path)
{
  if (path.empty())
    return false;

  std::string current;
  if (path.front() == '/')
    current = "/";

  std::size_t begin = 0;
  while (begin < path.size())
  {
    const std::size_t end = path.find('/', begin);
    const std::string part = path.substr(begin, end - begin);
    if (!part.empty())
    {
      if (!current.empty() && current.back() != '/')
        current += '/';
      current += part;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
        return false;
    }
    if (end == std::string::npos)
      break;
    begin = end + 1;
  }
  return true;
}

bool fileIsEmpty(const std::string &path)
{
  std::ifstream input(path.c_str(), std::ios::binary | std::ios::ate);
  return !input || input.tellg() == std::streampos(0);
}

} // namespace

void DeploymentLogger::configure(const bool enabled, const std::string &directory)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
  directory_ = directory;
  path_.clear();

  if (!enabled_)
    return;

  const std::string runs_directory = directory_ + "/runs";
  if (!ensureDirectory(runs_directory))
    return;

  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  if (::localtime_r(&now, &local_time) == nullptr)
    return;

  std::ostringstream filename;
  filename << runs_directory << "/dac_sfc_"
           << std::put_time(&local_time, "%Y%m%d_%H%M%S")
           << "_pid" << static_cast<long>(::getpid()) << ".csv";
  path_ = filename.str();

  // A stable pointer file lets analysis commands select the current planner
  // session without slicing or mixing the legacy aggregate CSV.
  std::ofstream latest((directory_ + "/latest_csv_path.txt").c_str(),
                       std::ios::out | std::ios::trunc);
  if (!latest)
  {
    path_.clear();
    return;
  }
  latest << path_ << '\n';
}

bool DeploymentLogger::append(const DeploymentRecord &record)
{
  if (!enabled_)
    return true;

  std::lock_guard<std::mutex> lock(mutex_);
  if (path_.empty())
    return false;

  const bool write_header = fileIsEmpty(path_);
  std::ofstream output(path_.c_str(), std::ios::out | std::ios::app);
  if (!output)
    return false;

  if (write_header)
  {
    output << "timestamp_s,run_id,data_source,execution_mode,pipeline_success,"
              "trajectory_activated,ego_fallback_used,sampled_collision_free,failure_stage,"
              "direct_path,raw_astar_points,sparse_route_points,raw_length_m,sparse_length_m,"
              "astar_ms,shortcut_ms,obstacle_extract_ms,guide_ms,csgn_ms,corridor_ms,"
              "optimizer_setup_ms,optimizer_ms,total_ms,obstacle_points,corridor_count,"
              "total_faces,geometry_evaluations_per_call,active_witness_rounds,"
              "redundancy_removed,valid_csgn_metrics,mean_corridor_anisotropy,"
              "max_corridor_anisotropy,final_cost,final_corridor_violation_m,"
              "trajectory_duration_s,max_velocity_mps,max_acceleration_mps2,"
              "optimizer_attempts,final_position_weight,"
              "violation_piece,violation_corridor,violation_face,"
              "violation_sample,violation_piece_time_s,"
              "violation_x,violation_y,violation_z,optimizer_piece_count,"
              "final_dynamic_penalty_scale,terminal_velocity_aligned,"
              "terminal_velocity_alignment_angle_deg,"
              "requested_start_clearance_m,available_start_clearance_m\n";
  }

  output << std::setprecision(17)
         << record.timestamp_s << ',' << record.run_id << ',' << record.data_source << ','
         << record.execution_mode << ',' << record.pipeline_success << ','
         << record.trajectory_activated << ',' << record.ego_fallback_used << ','
         << record.sampled_collision_free << ',' << record.failure_stage << ','
         << record.route.direct_path << ',' << record.route.raw_point_count << ','
         << record.route.sparse_point_count << ',' << record.route.raw_length << ','
         << record.route.sparse_length << ',' << record.route.search_ms << ','
         << record.route.shortcut_ms << ',' << record.obstacle_extract_ms << ','
         << record.engine.guide_ms << ',' << record.engine.csgn_ms << ','
         << record.engine.corridor_ms << ',' << record.engine.optimizer_setup_ms << ','
         << record.engine.optimizer_ms << ',' << record.total_ms << ','
         << record.engine.obstacle_points << ',' << record.engine.corridor_count << ','
         << record.engine.total_faces << ','
         << record.engine.geometry_evaluations_per_call << ','
         << record.engine.active_witness_rounds << ','
         << record.engine.redundancy_removed << ','
         << record.engine.valid_csgn_metrics << ','
         << record.engine.mean_corridor_anisotropy << ','
         << record.engine.max_corridor_anisotropy << ',' << record.engine.final_cost << ','
         << record.engine.final_corridor_violation << ','
         << record.engine.trajectory_duration << ',' << record.engine.max_velocity << ','
         << record.engine.max_acceleration << ','
         << record.engine.optimizer_attempts << ','
         << record.engine.final_position_weight << ','
         << record.engine.violation_piece << ','
         << record.engine.violation_corridor << ','
         << record.engine.violation_face << ','
         << record.engine.violation_sample << ','
         << record.engine.violation_time << ','
         << record.engine.violation_position.x() << ','
         << record.engine.violation_position.y() << ','
         << record.engine.violation_position.z() << ','
         << record.engine.optimizer_piece_count << ','
         << record.engine.final_dynamic_penalty_scale << ','
         << record.engine.terminal_velocity_aligned << ','
         << record.engine.terminal_velocity_alignment_angle_deg << ','
         << record.route.requested_start_clearance << ','
         << record.route.available_start_clearance << '\n';
  return static_cast<bool>(output);
}

} // namespace dac_sfc_deployment
