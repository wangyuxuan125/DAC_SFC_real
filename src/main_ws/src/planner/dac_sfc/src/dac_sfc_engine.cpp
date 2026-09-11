#include <dac_sfc/dac_sfc_engine.h>

// The onboard EGO-Planner process already links another lbfgs/root-finder
// implementation. Rename the upstream namespaces in this translation unit so
// both optimizers can coexist without ODR/linker collisions.
#define lbfgs dac_gcopter_lbfgs
#define RootFinder DacGcopterRootFinder
#include <gcopter/geo_utils.hpp>
#include <gcopter/traj_relevant_corridor.hpp>
#include <gcopter/gcopter.hpp>
#undef RootFinder
#undef lbfgs

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>

namespace dac_sfc_deployment
{
namespace
{

using Clock = std::chrono::steady_clock;

double millisecondsSince(const Clock::time_point &start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool optionsAreValid(const EngineOptions &options)
{
  return options.max_velocity > 0.0 && options.max_acceleration > 0.0 &&
         options.max_body_rate > 0.0 &&
         options.max_tilt_angle > 0.0 && options.min_thrust >= 0.0 &&
         options.max_thrust > options.min_thrust && options.vehicle_mass > 0.0 &&
         options.gravity > 0.0 && options.speed_smoothing > 0.0 &&
         options.quadrature_resolution > 0 && options.guide_reference_speed_ratio > 0.0 &&
         options.min_piece_time > 0.0 &&
         std::isfinite(options.optimizer_piece_length) &&
         options.optimizer_piece_length > 0.0 &&
         options.csgn_displacement_step > 0.0 &&
         options.csgn_relative_damping > 0.0 && options.csgn_proximity_power >= 0.0 &&
         options.max_corridor_anisotropy >= 1.0 && options.max_extra_radius > 0.0 &&
         options.min_extra_ratio >= 0.0 && options.min_extra_ratio <= 1.0 &&
         options.overlap_radius >= 0.0 && options.map_boundary_margin >= 0.0 &&
         std::isfinite(options.obstacle_voxel_size) &&
         options.obstacle_voxel_size >= 0.0 &&
         std::isfinite(options.position_weight) &&
         options.position_weight > 0.0 &&
         std::isfinite(options.acceleration_weight) &&
         options.acceleration_weight > 0.0 &&
         options.max_corridor_retries >= 0 &&
         std::isfinite(options.corridor_penalty_scale) &&
         options.corridor_penalty_scale > 1.0 &&
         std::isfinite(options.dynamic_penalty_scale) &&
         options.dynamic_penalty_scale > 1.0;
}

} // namespace

bool DacSfcEngine::plan(const std::vector<Eigen::Vector3d> &route,
                        const std::vector<Eigen::Vector3d> &obstacle_surface,
                        const Eigen::Vector3d &map_lower,
                        const Eigen::Vector3d &map_upper,
                        const Eigen::Matrix3d &initial_pva,
                        const Eigen::Matrix3d &terminal_pva,
                        const EngineOptions &options,
                        EngineResult &result) const
{
  result = EngineResult();
  result.diagnostics.obstacle_points = static_cast<int>(obstacle_surface.size());

  if (route.size() < 2 || !map_lower.allFinite() || !map_upper.allFinite() ||
      (map_upper.array() <= map_lower.array()).any() || !initial_pva.allFinite() ||
      !terminal_pva.allFinite() || !optionsAreValid(options))
  {
    result.diagnostics.failure_stage = "engine_input";
    return false;
  }
  for (const Eigen::Vector3d &point : route)
  {
    if (!point.allFinite())
    {
      result.diagnostics.failure_stage = "route_nonfinite";
      return false;
    }
  }

  Eigen::Matrix3d effective_terminal_pva = terminal_pva;
  const Eigen::Vector3d final_route_delta =
      route.back() - route[route.size() - 2];
  const double final_route_length = final_route_delta.norm();
  const double requested_terminal_speed =
      terminal_pva.col(1).norm();
  if (options.align_terminal_velocity_with_route &&
      final_route_length > 1.0e-6 &&
      requested_terminal_speed > 1.0e-6)
  {
    const Eigen::Vector3d final_route_tangent =
        final_route_delta / final_route_length;
    const Eigen::Vector3d requested_velocity_direction =
        terminal_pva.col(1) / requested_terminal_speed;
    const double direction_cosine =
        std::max(-1.0, std::min(
                           1.0,
                           requested_velocity_direction.dot(
                               final_route_tangent)));
    result.diagnostics.terminal_velocity_alignment_angle_deg =
        std::acos(direction_cosine) *
        180.0 / std::acos(-1.0);
    effective_terminal_pva.col(1) =
        requested_terminal_speed * final_route_tangent;
    result.diagnostics.terminal_velocity_aligned =
        result.diagnostics.terminal_velocity_alignment_angle_deg >
        1.0e-3;
  }

  const int piece_count = static_cast<int>(route.size()) - 1;
  Eigen::Matrix3Xd inner_points(3, std::max(piece_count - 1, 0));
  for (int i = 1; i < static_cast<int>(route.size()) - 1; ++i)
    inner_points.col(i - 1) = route[i];

  Eigen::VectorXd guide_times(piece_count);
  const double reference_speed = std::max(
      options.max_velocity * options.guide_reference_speed_ratio, 1.0e-3);
  for (int i = 0; i < piece_count; ++i)
  {
    guide_times(i) = std::max((route[i + 1] - route[i]).norm() / reference_speed,
                              options.min_piece_time);
  }

  Trajectory<5> guide_trajectory;
  const Clock::time_point guide_started = Clock::now();
  minco::MINCO_S3NU guide_minco;
  guide_minco.setConditions(initial_pva, effective_terminal_pva, piece_count);
  guide_minco.setParameters(inner_points, guide_times);
  guide_minco.getTrajectory(guide_trajectory);
  result.diagnostics.guide_ms = millisecondsSince(guide_started);

  if (guide_trajectory.getPieceNum() != piece_count)
  {
    result.diagnostics.failure_stage = "minco_guide";
    return false;
  }
  for (int i = 0; i < piece_count; ++i)
  {
    if (!std::isfinite(guide_trajectory[i].getDuration()) ||
        guide_trajectory[i].getDuration() <= 0.0 ||
        !guide_trajectory[i].getCoeffMat().allFinite())
    {
      result.diagnostics.failure_stage = "minco_guide_nonfinite";
      return false;
    }
  }

  Eigen::VectorXd magnitude_bounds(6);
  magnitude_bounds << options.max_velocity, options.max_body_rate,
      options.max_tilt_angle, options.min_thrust, options.max_thrust,
      options.max_acceleration;
  Eigen::VectorXd physical_parameters(6);
  physical_parameters << options.vehicle_mass, options.gravity,
      options.horizontal_drag, options.vertical_drag, options.parasitic_drag,
      options.speed_smoothing;

  gcopter::GCOPTER_PolytopeSFC metric_evaluator;
  gcopter::GCOPTER_PolytopeSFC::GaussNewtonDeformationMetrics metrics;
  const bool single_piece_csgn_fallback = piece_count == 1;
  const Clock::time_point csgn_started = Clock::now();
  bool csgn_success = false;

  if (single_piece_csgn_fallback)
  {
    // A one-piece trajectory has no internal waypoint to perturb, so its
    // Cartesian Gauss-Newton deformation metric is unobservable.  The
    // isotropic identity is the neutral limit: it adds no invented preferred
    // direction while still allowing Active-Witness and GCOPTER to run.
    metrics.resize(1);
    metrics.front().valid = true;
    metrics.front().failureReason = "single_piece_identity_fallback";
    metrics.front().corridorUtility.setIdentity();
    metrics.front().corridorUtilityEigenvalues.setOnes();
    metrics.front().corridorAnisotropy = 1.0;
    metrics.front().maxCorridorAnisotropyUsed =
        options.max_corridor_anisotropy;
    csgn_success = true;
  }
  else
  {
    if (!metric_evaluator.setGaussNewtonReferenceState(
            initial_pva, effective_terminal_pva, inner_points, guide_times,
            options.quadrature_resolution, magnitude_bounds,
            physical_parameters))
    {
      result.diagnostics.failure_stage = "csgn_setup";
      return false;
    }

    csgn_success = metric_evaluator.computeGaussNewtonDeformationMetrics(
        metrics, options.csgn_displacement_step,
        options.csgn_relative_damping, options.csgn_proximity_power,
        options.max_corridor_anisotropy);
  }

  result.diagnostics.csgn_ms = millisecondsSince(csgn_started);
  if (!csgn_success || static_cast<int>(metrics.size()) != piece_count)
  {
    result.diagnostics.failure_stage = "csgn";
    return false;
  }

  double anisotropy_sum = 0.0;
  for (const auto &metric : metrics)
  {
    if (!metric.valid || !metric.corridorUtility.allFinite())
    {
      result.diagnostics.failure_stage = "csgn_metric";
      return false;
    }
    // Keep this counter as the number of metrics actually estimated by CSGN.
    // Zero with a successful one-piece run identifies the isotropic fallback.
    if (!single_piece_csgn_fallback)
      ++result.diagnostics.valid_csgn_metrics;
    anisotropy_sum += metric.corridorAnisotropy;
    result.diagnostics.max_corridor_anisotropy =
        std::max(result.diagnostics.max_corridor_anisotropy,
                 metric.corridorAnisotropy);
  }
  result.diagnostics.mean_corridor_anisotropy =
      anisotropy_sum / static_cast<double>(metrics.size());

  const Eigen::Vector3d safe_lower =
      map_lower.array() + options.map_boundary_margin;
  const Eigen::Vector3d safe_upper =
      map_upper.array() - options.map_boundary_margin;

  const Clock::time_point corridor_started = Clock::now();
  result.corridors.reserve(piece_count);
  for (int i = 0; i < piece_count; ++i)
  {
    traj_relevant::CompactCorridorOptions corridor_options;
    corridor_options.candidate_selection_mode =
        traj_relevant::CandidateSelectionMode::ACTIVE_WITNESS;
    corridor_options.max_extra_radius = options.max_extra_radius;
    corridor_options.min_extra_ratio = options.min_extra_ratio;
    corridor_options.overlap_radius = options.overlap_radius;
    corridor_options.obstacle_half_extent =
        0.5 * options.obstacle_voxel_size;
    corridor_options.metric_enabled = true;
    corridor_options.deformation_utility = metrics[i].corridorUtility;
    corridor_options.epsilon = 1.0e-6;

    Eigen::MatrixX4d corridor;
    traj_relevant::CompactCorridorDiagnostics diagnostics;
    if (!traj_relevant::buildCompactSegmentPolytope(
            obstacle_surface, safe_lower, safe_upper, route[i], route[i + 1],
            corridor, corridor_options, &diagnostics))
    {
      result.diagnostics.corridor_ms = millisecondsSince(corridor_started);
      result.diagnostics.failure_stage =
          diagnostics.failure_reason.empty()
              ? "active_witness_corridor"
              : diagnostics.failure_reason;
      std::cerr << "[DAC-SFC] Active-Witness corridor rejected segment=" << i
                << " reason=" << result.diagnostics.failure_stage
                << " margin_m=" << diagnostics.failure_margin
                << " obstacle=" << diagnostics.failure_obstacle
                << " euclidean_fallbacks="
                << diagnostics.euclidean_witness_fallbacks
                << " start=" << route[i].transpose()
                << " end=" << route[i + 1].transpose()
                << std::endl;
      return false;
    }

    if (diagnostics.euclidean_witness_fallbacks > 0)
    {
      std::cerr << "[DAC-SFC] Active-Witness repaired voxel normals segment="
                << i
                << " count=" << diagnostics.euclidean_witness_fallbacks
                << std::endl;
    }

    if (!result.corridors.empty() &&
        !geo_utils::overlap(result.corridors.back(), corridor,
                            std::max(1.0e-6, 0.5 * options.overlap_radius)))
    {
      result.diagnostics.corridor_ms = millisecondsSince(corridor_started);
      result.diagnostics.failure_stage = "corridor_overlap";
      return false;
    }

    result.diagnostics.total_faces += static_cast<int>(corridor.rows());
    result.diagnostics.active_witness_rounds += diagnostics.active_witness_rounds;
    result.diagnostics.redundancy_removed += diagnostics.redundancy_removed;
    result.corridors.push_back(corridor);
  }
  result.diagnostics.corridor_ms = millisecondsSince(corridor_started);
  result.diagnostics.corridor_count = static_cast<int>(result.corridors.size());
  result.diagnostics.geometry_evaluations_per_call =
      (options.quadrature_resolution + 1) * result.diagnostics.total_faces;

  Eigen::VectorXd penalty_weights(6);
  penalty_weights << options.position_weight, options.velocity_weight,
      options.body_rate_weight, options.tilt_weight, options.thrust_weight,
      options.acceleration_weight;

  gcopter::GCOPTER_PolytopeSFC optimizer;
  const Clock::time_point setup_started = Clock::now();
  const bool setup_success = optimizer.setup(
      options.time_weight, initial_pva, effective_terminal_pva, result.corridors,
      options.optimizer_piece_length, options.smoothing_epsilon,
      options.quadrature_resolution, magnitude_bounds, penalty_weights,
      physical_parameters);
  result.diagnostics.optimizer_setup_ms = millisecondsSince(setup_started);
  if (!setup_success)
  {
    result.diagnostics.failure_stage = "gcopter_setup";
    return false;
  }

  Trajectory<5> optimized_trajectory;
  bool corridor_satisfied = false;
  double cumulative_penalty_scale = 1.0;
  double cumulative_dynamic_penalty_scale = 1.0;
  const int total_optimizer_attempts = 1 + options.max_corridor_retries;

  for (int attempt = 0; attempt < total_optimizer_attempts; ++attempt)
  {
    Trajectory<5> candidate_trajectory;
    const Clock::time_point optimize_started = Clock::now();
    const double candidate_cost =
        attempt == 0
            ? optimizer.optimize(
                  candidate_trajectory,
                  options.relative_cost_tolerance)
            : optimizer.continueOptimizeWithPenaltyScales(
                  candidate_trajectory,
                  options.relative_cost_tolerance,
                  cumulative_penalty_scale,
                  cumulative_dynamic_penalty_scale);
    result.diagnostics.optimizer_ms += millisecondsSince(optimize_started);
    ++result.diagnostics.optimizer_attempts;
    result.diagnostics.final_cost = candidate_cost;
    result.diagnostics.final_position_weight =
        options.position_weight * cumulative_penalty_scale;
    result.diagnostics.final_dynamic_penalty_scale =
        cumulative_dynamic_penalty_scale;

    if (!std::isfinite(candidate_cost) ||
        candidate_trajectory.getPieceNum() <= 0)
    {
      result.diagnostics.failure_stage =
          attempt == 0 ? "gcopter_optimize"
                       : "gcopter_corridor_continuation";
      return false;
    }

    result.diagnostics.optimizer_piece_count =
        candidate_trajectory.getPieceNum();
    result.diagnostics.max_velocity = 0.0;
    result.diagnostics.max_acceleration = 0.0;
    for (int piece_id = 0;
         piece_id < candidate_trajectory.getPieceNum(); ++piece_id)
    {
      const auto &candidate_piece = candidate_trajectory[piece_id];
      result.diagnostics.max_velocity =
          std::max(result.diagnostics.max_velocity,
                   candidate_piece.getMaxVelRate());
      result.diagnostics.max_acceleration =
          std::max(result.diagnostics.max_acceleration,
                   candidate_piece.getMaxAccRate());
    }

    const auto &final_corridor = optimizer.getFinalCorridorDiagnostics();
    result.diagnostics.final_corridor_violation =
        final_corridor.maxViolationM;
    result.diagnostics.violation_piece =
        final_corridor.maxViolationPiece;
    result.diagnostics.violation_corridor =
        final_corridor.maxViolationCorridor;
    result.diagnostics.violation_face =
        final_corridor.maxViolationFace;
    result.diagnostics.violation_sample =
        final_corridor.maxViolationSample;
    result.diagnostics.violation_time =
        final_corridor.maxViolationTime;
    result.diagnostics.violation_position =
        final_corridor.maxViolationPosition;

    if (std::isfinite(final_corridor.maxViolationM) &&
        final_corridor.maxViolationM <=
            options.max_final_corridor_violation)
    {
      optimized_trajectory = candidate_trajectory;
      corridor_satisfied = true;
      break;
    }

    if (!std::isfinite(final_corridor.maxViolationM))
    {
      result.diagnostics.failure_stage = "corridor_violation";
      return false;
    }

    if (attempt + 1 < total_optimizer_attempts)
    {
      const double next_penalty_scale =
          cumulative_penalty_scale * options.corridor_penalty_scale;
      const double next_dynamic_penalty_scale =
          cumulative_dynamic_penalty_scale *
          options.dynamic_penalty_scale;
      const double next_position_weight =
          options.position_weight * next_penalty_scale;
      if (!std::isfinite(next_penalty_scale) ||
          !std::isfinite(next_dynamic_penalty_scale) ||
          !std::isfinite(next_position_weight))
      {
        result.diagnostics.failure_stage = "corridor_penalty_overflow";
        return false;
      }

      std::cerr << "[DAC-SFC] GCOPTER warm corridor continuation attempt="
                << (attempt + 1) << "/" << total_optimizer_attempts
                << " violation_m=" << final_corridor.maxViolationM
                << " limit_m=" << options.max_final_corridor_violation
                << " piece=" << final_corridor.maxViolationPiece
                << " corridor=" << final_corridor.maxViolationCorridor
                << " face=" << final_corridor.maxViolationFace
                << " sample=" << final_corridor.maxViolationSample
                << " piece_t=" << final_corridor.maxViolationTime
                << " point="
                << final_corridor.maxViolationPosition.transpose()
                << " position_weight="
                << result.diagnostics.final_position_weight
                << " dynamic_scale="
                << cumulative_dynamic_penalty_scale
                << " next_dynamic_scale="
                << next_dynamic_penalty_scale
                << " candidate_max_vel="
                << result.diagnostics.max_velocity
                << " candidate_max_acc="
                << result.diagnostics.max_acceleration
                << " next_position_weight=" << next_position_weight
                << std::endl;

      cumulative_penalty_scale = next_penalty_scale;
      cumulative_dynamic_penalty_scale =
          next_dynamic_penalty_scale;
    }
  }

  if (!corridor_satisfied)
  {
    double terminal_endpoint_face_violation =
        std::numeric_limits<double>::quiet_NaN();
    double terminal_velocity_face_component =
        std::numeric_limits<double>::quiet_NaN();
    double route_tangent_face_component =
        std::numeric_limits<double>::quiet_NaN();

    const int violation_corridor =
        result.diagnostics.violation_corridor;
    const int violation_face =
        result.diagnostics.violation_face;
    if (violation_corridor >= 0 &&
        violation_corridor <
            static_cast<int>(result.corridors.size()) &&
        violation_face >= 0 &&
        violation_face <
            result.corridors[violation_corridor].rows())
    {
      const Eigen::Vector4d face =
          result.corridors[violation_corridor]
              .row(violation_face)
              .transpose();
      const Eigen::Vector3d normal = face.head<3>();
      const double normal_norm = normal.norm();
      if (normal_norm > 1.0e-9)
      {
        terminal_endpoint_face_violation =
            (normal.dot(effective_terminal_pva.col(0)) + face(3)) /
            normal_norm;
        terminal_velocity_face_component =
            normal.dot(effective_terminal_pva.col(1)) /
            normal_norm;

        const Eigen::Vector3d route_delta =
            route.back() - route[route.size() - 2];
        const double route_delta_norm = route_delta.norm();
        if (route_delta_norm > 1.0e-9)
        {
          route_tangent_face_component =
              normal.dot(route_delta / route_delta_norm) /
              normal_norm;
        }
      }
    }

    std::cerr << "[DAC-SFC] GCOPTER warm corridor continuation exhausted attempts="
              << result.diagnostics.optimizer_attempts
              << " violation_m="
              << result.diagnostics.final_corridor_violation
              << " limit_m=" << options.max_final_corridor_violation
              << " piece=" << result.diagnostics.violation_piece
              << " corridor=" << result.diagnostics.violation_corridor
              << " face=" << result.diagnostics.violation_face
              << " sample=" << result.diagnostics.violation_sample
              << " piece_t=" << result.diagnostics.violation_time
              << " point="
              << result.diagnostics.violation_position.transpose()
              << " final_position_weight="
              << result.diagnostics.final_position_weight
              << " final_dynamic_scale="
              << result.diagnostics.final_dynamic_penalty_scale
              << " candidate_max_vel="
              << result.diagnostics.max_velocity
              << " candidate_max_acc="
              << result.diagnostics.max_acceleration
              << " terminal_point="
              << effective_terminal_pva.col(0).transpose()
              << " terminal_velocity="
              << effective_terminal_pva.col(1).transpose()
              << " requested_terminal_velocity="
              << terminal_pva.col(1).transpose()
              << " terminal_velocity_aligned="
              << result.diagnostics.terminal_velocity_aligned
              << " terminal_velocity_alignment_angle_deg="
              << result.diagnostics.terminal_velocity_alignment_angle_deg
              << " endpoint_face_violation_m="
              << terminal_endpoint_face_violation
              << " terminal_velocity_face_component="
              << terminal_velocity_face_component
              << " route_tangent_face_component="
              << route_tangent_face_component
              << std::endl;
    result.diagnostics.failure_stage = "corridor_violation";
    return false;
  }

  result.durations = optimized_trajectory.getDurations();
  result.coefficients.reserve(optimized_trajectory.getPieceNum());
  for (int i = 0; i < optimized_trajectory.getPieceNum(); ++i)
  {
    const auto &piece = optimized_trajectory[i];
    if (!piece.getCoeffMat().allFinite() || !std::isfinite(piece.getDuration()) ||
        piece.getDuration() <= 0.0)
    {
      result.diagnostics.failure_stage = "trajectory_nonfinite";
      return false;
    }
    result.coefficients.push_back(piece.getCoeffMat());
    result.diagnostics.max_velocity =
        std::max(result.diagnostics.max_velocity, piece.getMaxVelRate());
    result.diagnostics.max_acceleration =
        std::max(result.diagnostics.max_acceleration, piece.getMaxAccRate());
  }

  result.diagnostics.trajectory_duration = result.durations.sum();
  result.diagnostics.success = true;
  return true;
}

} // namespace dac_sfc_deployment
