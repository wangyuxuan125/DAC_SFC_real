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
  return options.max_velocity > 0.0 && options.max_body_rate > 0.0 &&
         options.max_tilt_angle > 0.0 && options.min_thrust >= 0.0 &&
         options.max_thrust > options.min_thrust && options.vehicle_mass > 0.0 &&
         options.gravity > 0.0 && options.speed_smoothing > 0.0 &&
         options.quadrature_resolution > 0 && options.guide_reference_speed_ratio > 0.0 &&
         options.min_piece_time > 0.0 && options.csgn_displacement_step > 0.0 &&
         options.csgn_relative_damping > 0.0 && options.csgn_proximity_power >= 0.0 &&
         options.max_corridor_anisotropy >= 1.0 && options.max_extra_radius > 0.0 &&
         options.min_extra_ratio >= 0.0 && options.min_extra_ratio <= 1.0 &&
         options.overlap_radius >= 0.0 && options.map_boundary_margin >= 0.0 &&
         std::isfinite(options.obstacle_voxel_size) &&
         options.obstacle_voxel_size >= 0.0;
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
  guide_minco.setConditions(initial_pva, terminal_pva, piece_count);
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

  Eigen::VectorXd magnitude_bounds(5);
  magnitude_bounds << options.max_velocity, options.max_body_rate,
      options.max_tilt_angle, options.min_thrust, options.max_thrust;
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
            initial_pva, terminal_pva, inner_points, guide_times,
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
      result.diagnostics.failure_stage = "active_witness_corridor";
      return false;
    }

    // buildCompactSegmentPolytope treats obstacle samples as mathematical
    // points, while the grid map regards the complete resolution-sized voxel
    // as occupied.  Erode each halfspace by the support function of an
    // axis-aligned half voxel: h * ||a||_1 for a^T x + b <= 0.  This is
    // equivalent to building against all eight voxel corners, without the
    // eightfold obstacle-cloud and Active-Witness cost.
    if (options.obstacle_voxel_size > 0.0)
    {
      const double half_voxel = 0.5 * options.obstacle_voxel_size;
      for (int face_id = 0; face_id < corridor.rows(); ++face_id)
      {
        const Eigen::Vector3d normal =
            corridor.block<1, 3>(face_id, 0).transpose();
        corridor(face_id, 3) += half_voxel * normal.cwiseAbs().sum();
      }

      // A convex corridor contains the complete guide segment iff it contains
      // both endpoints.  Report a geometric failure when voxel erosion makes
      // the discrete A* segment too tight instead of handing an invalid SFC to
      // GCOPTER.
      const double containment_tolerance = 1.0e-8;
      const Eigen::VectorXd start_face_values =
          corridor.leftCols<3>() * route[i] + corridor.col(3);
      const Eigen::VectorXd end_face_values =
          corridor.leftCols<3>() * route[i + 1] + corridor.col(3);
      Eigen::Index start_face = 0;
      Eigen::Index end_face = 0;
      const double start_violation = start_face_values.maxCoeff(&start_face);
      const double end_violation = end_face_values.maxCoeff(&end_face);
      if (start_violation > containment_tolerance ||
          end_violation > containment_tolerance)
      {
        const bool start_is_worst = start_violation >= end_violation;
        const Eigen::Index worst_face = start_is_worst ? start_face : end_face;
        const double worst_violation =
            start_is_worst ? start_violation : end_violation;
        const Eigen::Vector3d worst_normal =
            corridor.block<1, 3>(worst_face, 0).transpose();
        const double voxel_shift =
            half_voxel * worst_normal.cwiseAbs().sum();
        std::cerr << "[DAC-SFC] Voxel clearance rejected segment=" << i
                  << " face=" << worst_face
                  << " domain_faces=" << diagnostics.domain_face_count
                  << " post_violation_m=" << worst_violation /
                         std::max(worst_normal.norm(), 1.0e-12)
                  << " pre_violation_m=" <<
                         (worst_violation - voxel_shift) /
                         std::max(worst_normal.norm(), 1.0e-12)
                  << " voxel_shift_m=" << voxel_shift /
                         std::max(worst_normal.norm(), 1.0e-12)
                  << " start=" << route[i].transpose()
                  << " end=" << route[i + 1].transpose()
                  << std::endl;
        result.diagnostics.corridor_ms = millisecondsSince(corridor_started);
        result.diagnostics.failure_stage = "voxel_clearance_corridor";
        return false;
      }
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

  Eigen::VectorXd penalty_weights(5);
  penalty_weights << options.position_weight, options.velocity_weight,
      options.body_rate_weight, options.tilt_weight, options.thrust_weight;

  gcopter::GCOPTER_PolytopeSFC optimizer;
  const Clock::time_point setup_started = Clock::now();
  const bool setup_success = optimizer.setup(
      options.time_weight, initial_pva, terminal_pva, result.corridors,
      std::numeric_limits<double>::infinity(), options.smoothing_epsilon,
      options.quadrature_resolution, magnitude_bounds, penalty_weights,
      physical_parameters);
  result.diagnostics.optimizer_setup_ms = millisecondsSince(setup_started);
  if (!setup_success)
  {
    result.diagnostics.failure_stage = "gcopter_setup";
    return false;
  }

  Trajectory<5> optimized_trajectory;
  const Clock::time_point optimize_started = Clock::now();
  result.diagnostics.final_cost =
      optimizer.optimize(optimized_trajectory, options.relative_cost_tolerance);
  result.diagnostics.optimizer_ms = millisecondsSince(optimize_started);
  if (!std::isfinite(result.diagnostics.final_cost) ||
      optimized_trajectory.getPieceNum() <= 0)
  {
    result.diagnostics.failure_stage = "gcopter_optimize";
    return false;
  }

  const auto &final_corridor = optimizer.getFinalCorridorDiagnostics();
  result.diagnostics.final_corridor_violation = final_corridor.maxViolationM;
  if (!std::isfinite(final_corridor.maxViolationM) ||
      final_corridor.maxViolationM > options.max_final_corridor_violation)
  {
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
