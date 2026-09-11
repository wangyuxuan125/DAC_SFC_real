#ifndef DAC_SFC_ENGINE_H
#define DAC_SFC_ENGINE_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <string>
#include <vector>

namespace dac_sfc_deployment
{

using CoefficientMat = Eigen::Matrix<double, 3, 6>;
using CoefficientMats = std::vector<
    CoefficientMat,
    Eigen::aligned_allocator<CoefficientMat>>;

struct EngineOptions
{
  double max_velocity = 2.0;
  double max_acceleration = 4.0;
  double max_body_rate = 2.1;
  double max_tilt_angle = 1.05;
  double min_thrust = 2.0;
  double max_thrust = 12.0;

  double vehicle_mass = 0.61;
  double gravity = 9.8;
  double horizontal_drag = 0.70;
  double vertical_drag = 0.80;
  double parasitic_drag = 0.01;
  double speed_smoothing = 1.0e-4;

  double time_weight = 20.0;
  double position_weight = 1.0e4;
  double velocity_weight = 1.0e4;
  double acceleration_weight = 1.0e4;
  double body_rate_weight = 1.0e4;
  double tilt_weight = 1.0e4;
  double thrust_weight = 1.0e5;
  double smoothing_epsilon = 1.0e-2;
  int quadrature_resolution = 16;
  double relative_cost_tolerance = 1.0e-5;

  double guide_reference_speed_ratio = 0.5;
  double min_piece_time = 1.0e-3;
  // GCOPTER subdivides long corridors so the quintic trajectory has enough
  // internal degrees of freedom to remain inside narrow/turning polytopes.
  double optimizer_piece_length = 0.75;
  // The rolling-horizon terminal velocity may point across the final safe
  // segment. Project it onto the positive final-route tangent so a large
  // heading correction also reduces the terminal speed.
  bool align_terminal_velocity_with_route = true;
  double csgn_displacement_step = 0.01;
  double csgn_relative_damping = 1.0e-3;
  double csgn_proximity_power = 4.0;
  double max_corridor_anisotropy = 10.0;

  double max_extra_radius = 1.0;
  double min_extra_ratio = 0.25;
  double overlap_radius = 0.04;
  double map_boundary_margin = 0.02;
  // The obstacle cloud contains occupied voxel centers.  Pass their
  // resolution so Active-Witness uses the complete axis-aligned voxel support
  // during face generation, set cover, and final safety verification.
  double obstacle_voxel_size = 0.0;
  double max_final_corridor_violation = 0.002;
  // Retry GCOPTER with a stronger complete constraint penalty group when
  // the optimized trajectory exceeds the verified corridor.
  int max_corridor_retries = 2;
  double corridor_penalty_scale = 100.0;
  double dynamic_penalty_scale = 10.0;
};

struct EngineDiagnostics
{
  bool success = false;
  std::string failure_stage;

  double guide_ms = 0.0;
  double csgn_ms = 0.0;
  double corridor_ms = 0.0;
  double optimizer_setup_ms = 0.0;
  double optimizer_ms = 0.0;

  int obstacle_points = 0;
  int corridor_count = 0;
  int total_faces = 0;
  int geometry_evaluations_per_call = 0;
  int active_witness_rounds = 0;
  int redundancy_removed = 0;
  int valid_csgn_metrics = 0;

  double mean_corridor_anisotropy = 1.0;
  double max_corridor_anisotropy = 1.0;
  double final_cost = 0.0;
  double final_corridor_violation = 0.0;
  int violation_piece = -1;
  int violation_corridor = -1;
  int violation_face = -1;
  int violation_sample = -1;
  double violation_time = 0.0;
  Eigen::Vector3d violation_position = Eigen::Vector3d::Zero();
  int optimizer_attempts = 0;
  double final_position_weight = 0.0;
  double final_dynamic_penalty_scale = 1.0;
  bool terminal_velocity_aligned = false;
  double terminal_velocity_alignment_angle_deg = 0.0;
  double terminal_velocity_speed_ratio = 1.0;
  double trajectory_duration = 0.0;
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  int optimizer_piece_count = 0;
};

struct EngineResult
{
  Eigen::VectorXd durations;
  CoefficientMats coefficients;
  std::vector<Eigen::MatrixX4d> corridors;
  EngineDiagnostics diagnostics;
};

class DacSfcEngine
{
public:
  bool plan(const std::vector<Eigen::Vector3d> &route,
            const std::vector<Eigen::Vector3d> &obstacle_surface,
            const Eigen::Vector3d &map_lower,
            const Eigen::Vector3d &map_upper,
            const Eigen::Matrix3d &initial_pva,
            const Eigen::Matrix3d &terminal_pva,
            const EngineOptions &options,
            EngineResult &result) const;
};

} // namespace dac_sfc_deployment

#endif
