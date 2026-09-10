// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    nh.param("manager/dac_sfc/enabled", dac_sfc_enabled_, false);
    nh.param("manager/dac_sfc/shadow_only", dac_sfc_shadow_only_, true);
    nh.param("manager/dac_sfc/fallback_to_ego", dac_sfc_fallback_to_ego_, true);
    nh.param<std::string>("manager/dac_sfc/data_source", dac_sfc_data_source_, "simulation");
    nh.param("manager/dac_sfc/visualization_enabled",
             dac_sfc_visualization_enabled_, true);
    nh.param<std::string>("manager/dac_sfc/visualization_frame",
                          dac_sfc_visualization_frame_, "world");
    nh.param("manager/dac_sfc/max_segment_length", dac_route_options_.max_segment_length, 1.0);
    nh.param("manager/dac_sfc/line_sample_step_ratio", dac_route_options_.line_sample_step_ratio, 0.5);

    dac_engine_options_.max_velocity = pp_.max_vel_;
    nh.param("manager/dac_sfc/max_body_rate", dac_engine_options_.max_body_rate, 2.1);
    nh.param("manager/dac_sfc/max_tilt_angle", dac_engine_options_.max_tilt_angle, 1.05);
    nh.param("manager/dac_sfc/min_thrust", dac_engine_options_.min_thrust, 2.0);
    nh.param("manager/dac_sfc/max_thrust", dac_engine_options_.max_thrust, 12.0);
    nh.param("manager/dac_sfc/vehicle_mass", dac_engine_options_.vehicle_mass, 0.61);
    nh.param("manager/dac_sfc/gravity", dac_engine_options_.gravity, 9.8);
    nh.param("manager/dac_sfc/horizontal_drag", dac_engine_options_.horizontal_drag, 0.70);
    nh.param("manager/dac_sfc/vertical_drag", dac_engine_options_.vertical_drag, 0.80);
    nh.param("manager/dac_sfc/parasitic_drag", dac_engine_options_.parasitic_drag, 0.01);
    nh.param("manager/dac_sfc/speed_smoothing", dac_engine_options_.speed_smoothing, 1.0e-4);
    nh.param("manager/dac_sfc/time_weight", dac_engine_options_.time_weight, 20.0);
    nh.param("manager/dac_sfc/position_weight", dac_engine_options_.position_weight, 1.0e4);
    nh.param("manager/dac_sfc/velocity_weight", dac_engine_options_.velocity_weight, 1.0e4);
    nh.param("manager/dac_sfc/body_rate_weight", dac_engine_options_.body_rate_weight, 1.0e4);
    nh.param("manager/dac_sfc/tilt_weight", dac_engine_options_.tilt_weight, 1.0e4);
    nh.param("manager/dac_sfc/thrust_weight", dac_engine_options_.thrust_weight, 1.0e5);
    nh.param("manager/dac_sfc/smoothing_epsilon", dac_engine_options_.smoothing_epsilon, 1.0e-2);
    nh.param("manager/dac_sfc/quadrature_resolution", dac_engine_options_.quadrature_resolution, 16);
    nh.param("manager/dac_sfc/relative_cost_tolerance", dac_engine_options_.relative_cost_tolerance, 1.0e-5);
    nh.param("manager/dac_sfc/guide_reference_speed_ratio", dac_engine_options_.guide_reference_speed_ratio, 0.5);
    nh.param("manager/dac_sfc/csgn_displacement_step", dac_engine_options_.csgn_displacement_step, 0.01);
    nh.param("manager/dac_sfc/csgn_relative_damping", dac_engine_options_.csgn_relative_damping, 1.0e-3);
    nh.param("manager/dac_sfc/csgn_proximity_power", dac_engine_options_.csgn_proximity_power, 4.0);
    nh.param("manager/dac_sfc/max_corridor_anisotropy", dac_engine_options_.max_corridor_anisotropy, 10.0);
    nh.param("manager/dac_sfc/max_extra_radius", dac_engine_options_.max_extra_radius, 1.0);
    nh.param("manager/dac_sfc/min_extra_ratio", dac_engine_options_.min_extra_ratio, 0.25);
    nh.param("manager/dac_sfc/overlap_radius", dac_engine_options_.overlap_radius, 0.04);
    nh.param("manager/dac_sfc/map_boundary_margin", dac_engine_options_.map_boundary_margin, 0.02);
    nh.param("manager/dac_sfc/max_final_corridor_violation", dac_engine_options_.max_final_corridor_violation, 0.02);

    bool dac_log_enabled = true;
    std::string dac_log_directory;
    int astar_pool_xy = 100;
    int astar_pool_z = 60;
    nh.param("manager/dac_sfc/log_enabled", dac_log_enabled, true);
    nh.param<std::string>("manager/dac_sfc/log_directory", dac_log_directory,
                          "/tmp/dac_sfc_deployment");
    nh.param("manager/dac_sfc/astar_pool_xy", astar_pool_xy, 100);
    nh.param("manager/dac_sfc/astar_pool_z", astar_pool_z, 60);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    if (dac_sfc_enabled_)
    {
      dac_route_adapter_.reset(new dac_sfc_deployment::DacRouteAdapter);
      dac_route_adapter_->initialize(
          grid_map_, Eigen::Vector3i(std::max(astar_pool_xy, 10),
                                    std::max(astar_pool_xy, 10),
                                    std::max(astar_pool_z, 10)));
    }
    dac_sfc_logger_.configure(dac_log_enabled, dac_log_directory);
    if (dac_sfc_enabled_ && dac_sfc_visualization_enabled_)
    {
      dac_sfc_polyhedron_pub_ =
          nh.advertise<decomp_ros_msgs::PolyhedronArray>(
              "dac_sfc/polyhedron_array", 1, true);
    }

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {
    if (dac_sfc_enabled_)
    {
      bool trajectory_activated = false;
      const bool dac_success = dacSfcReplan(
          start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
          touch_goal, trajectory_activated);

      if (trajectory_activated)
        return true;

      if (!dac_sfc_shadow_only_ && !dac_success && !dac_sfc_fallback_to_ego_)
        return false;

      if (!dac_sfc_shadow_only_ && !dac_success)
        ROS_WARN("[DAC-SFC] Pipeline failed; using the configured EGO fallback.");
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    static int count = 0;
    cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;
    // if ((start_pt - local_target_pt).norm() < 0.2)
    //   cout << "Close to goal" << endl;

    /*** STEP 1: INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    // ROS_ERROR("BBBB");

    if (pp_.use_multitopology_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;

      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        ploy_traj_opt_->setUseMultitopologyTrajs(true);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(), final_cost))
        {
          success[i] = true;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
      }

      t_opt = ros::Time::now() - t_start;

      if (trajs.size() > 1)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << ",\033[1;0m"
             << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
      }

      visualization_->displayMultiOptimalPathList(vis_trajs, 0.1); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(), final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();

      t_opt = ros::Time::now() - t_start;
    }

    /*** STEP 3: Store and display results ***/
    cout << "Success=" << (flag_success ? "yes" : "no") << endl;
    if (flag_success)
    {
      static double sum_time = 0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).toSec();
      count_success++;
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);
      // cout << "total time:\033[42m" << (t_init + t_opt).toSec()
      //      << "\033[0m,init:" << t_init.toSec()
      //      << ",optimize:" << t_opt.toSec()
      //      << ",avg_time=" << sum_time / count_success << endl;

      setLocalTrajFromOpt(best_MJO, touch_goal);
      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayOptimalList(cstr_pts, 0);

      continous_failures_count_ = 0;
    }
    else
    {
      cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayFailedList(cstr_pts, 0);

      continous_failures_count_++;
    }

    return flag_success;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if (t_to_lc_end < 0)
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    {
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());
    }

    return ret;
  }

  bool EGOPlannerManager::setLocalTraj(const poly_traj::Trajectory &trajectory,
                                       const bool touch_goal)
  {
    if (trajectory.getPieceNum() <= 0)
      return false;

    poly_traj::Trajectory trajectory_copy = trajectory;
    ploy_traj_opt_->setIfTouchGoal(touch_goal);

    const int control_point_count =
        trajectory_copy.getPieceNum() * getCpsNumPrePiece() + 1;
    const int id_end = touch_goal
                           ? control_point_count - 1
                           : control_point_count - 1 - (control_point_count - 2) / 3;

    PtsChk_t points_to_check;
    if (!ploy_traj_opt_->computePointsToCheck(
            trajectory_copy, id_end, points_to_check) ||
        points_to_check.empty())
      return false;

    traj_.setLocalTraj(trajectory_copy, points_to_check, ros::Time::now().toSec());
    return true;
  }

  bool EGOPlannerManager::trajectoryIsCollisionFree(
      const poly_traj::Trajectory &trajectory,
      std::string &failure_stage) const
  {
    failure_stage.clear();

    if (!grid_map_)
    {
      failure_stage = "trajectory_map_unavailable";
      return false;
    }
    if (trajectory.getPieceNum() <= 0)
    {
      failure_stage = "trajectory_empty";
      return false;
    }

    const double duration = trajectory.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0)
    {
      failure_stage = "trajectory_duration_invalid";
      return false;
    }

    const double time_step = std::max(
        0.002, grid_map_->getResolution() /
                   (2.0 * std::max(dac_engine_options_.max_velocity, 0.1)));
    const int samples = std::max(
        1, static_cast<int>(std::ceil(duration / time_step)));
    for (int i = 0; i <= samples; ++i)
    {
      const double time = duration * static_cast<double>(i) /
                          static_cast<double>(samples);
      const Eigen::Vector3d point = trajectory.getPos(time);

      if (!point.allFinite())
      {
        failure_stage = "trajectory_sample_nonfinite";
        ROS_WARN_STREAM("[DAC-SFC] Safety check failed: " << failure_stage
                        << " sample=" << i << "/" << samples
                        << " t=" << time);
        return false;
      }

      if (!grid_map_->isInInflatedMap(point))
      {
        failure_stage = "trajectory_outside_inflated_map";
        ROS_WARN_STREAM("[DAC-SFC] Safety check failed: " << failure_stage
                        << " sample=" << i << "/" << samples
                        << " t=" << time
                        << " point=" << point.transpose());
        return false;
      }

      const int occupancy = grid_map_->getInflateOccupancy(point);
      if (occupancy != 0)
      {
        failure_stage = "trajectory_occupied";
        ROS_WARN_STREAM("[DAC-SFC] Safety check failed: " << failure_stage
                        << " sample=" << i << "/" << samples
                        << " t=" << time
                        << " point=" << point.transpose()
                        << " occupancy=" << occupancy);
        return false;
      }
    }
    return true;
  }

  void EGOPlannerManager::publishDacSfcCorridors(
      const std::vector<Eigen::MatrixX4d> &corridors) const
  {
    if (!dac_sfc_visualization_enabled_ ||
        dac_sfc_polyhedron_pub_.getTopic().empty())
      return;

    decomp_ros_msgs::PolyhedronArray message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = dac_sfc_visualization_frame_;
    message.polyhedrons.reserve(corridors.size());

    for (const Eigen::MatrixX4d &corridor : corridors)
    {
      decomp_ros_msgs::Polyhedron polyhedron;
      polyhedron.points.reserve(corridor.rows());
      polyhedron.normals.reserve(corridor.rows());

      for (int face_id = 0; face_id < corridor.rows(); ++face_id)
      {
        const Eigen::Vector3d coefficients =
            corridor.block<1, 3>(face_id, 0).transpose();
        const double squared_norm = coefficients.squaredNorm();
        const double offset = corridor(face_id, 3);
        if (!coefficients.allFinite() || !std::isfinite(offset) ||
            squared_norm <= 1.0e-18)
          continue;

        const Eigen::Vector3d point =
            (-offset / squared_norm) * coefficients;
        const Eigen::Vector3d normal =
            coefficients / std::sqrt(squared_norm);

        geometry_msgs::Point point_message;
        point_message.x = point.x();
        point_message.y = point.y();
        point_message.z = point.z();
        geometry_msgs::Point normal_message;
        normal_message.x = normal.x();
        normal_message.y = normal.y();
        normal_message.z = normal.z();
        polyhedron.points.push_back(point_message);
        polyhedron.normals.push_back(normal_message);
      }

      if (!polyhedron.points.empty())
        message.polyhedrons.push_back(polyhedron);
    }

    dac_sfc_polyhedron_pub_.publish(message);
  }

  bool EGOPlannerManager::dacSfcReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool touch_goal,
      bool &trajectory_activated)
  {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point total_started = Clock::now();
    trajectory_activated = false;

    dac_sfc_deployment::DeploymentRecord record;
    record.timestamp_s = ros::Time::now().toSec();
    record.run_id = ++dac_sfc_run_id_;
    record.data_source = dac_sfc_data_source_;
    record.execution_mode = dac_sfc_shadow_only_ ? "shadow" : "active";

    std::vector<Eigen::Vector3d> raw_path;
    std::vector<Eigen::Vector3d> route;
    if (!dac_route_adapter_ ||
        !dac_route_adapter_->build(start_pt, local_target_pt, dac_route_options_,
                                   raw_path, route, record.route))
    {
      record.failure_stage = record.route.failure_stage.empty()
                                 ? "route"
                                 : record.route.failure_stage;
      record.ego_fallback_used = !dac_sfc_shadow_only_ && dac_sfc_fallback_to_ego_;
      record.total_ms = std::chrono::duration<double, std::milli>(
                            Clock::now() - total_started)
                            .count();
      dac_sfc_logger_.append(record);
      publishDacSfcCorridors(std::vector<Eigen::MatrixX4d>());
      return false;
    }

    if (visualization_)
    {
      visualization_->displayAStarList({raw_path}, 800);
      visualization_->displayInitPathList(route, 0.12, 90);
    }

    Eigen::Vector3d map_lower, map_upper;
    if (!grid_map_->getInflatedMapBounds(map_lower, map_upper))
    {
      record.failure_stage = "map_bounds";
      record.ego_fallback_used = !dac_sfc_shadow_only_ && dac_sfc_fallback_to_ego_;
      record.total_ms = std::chrono::duration<double, std::milli>(
                            Clock::now() - total_started)
                            .count();
      dac_sfc_logger_.append(record);
      publishDacSfcCorridors(std::vector<Eigen::MatrixX4d>());
      return false;
    }

    Eigen::Vector3d obstacle_lower = route.front();
    Eigen::Vector3d obstacle_upper = route.front();
    for (const Eigen::Vector3d &point : route)
    {
      obstacle_lower = obstacle_lower.cwiseMin(point);
      obstacle_upper = obstacle_upper.cwiseMax(point);
    }
    const double obstacle_padding = dac_engine_options_.max_extra_radius +
                                    dac_engine_options_.overlap_radius +
                                    2.0 * grid_map_->getResolution();
    obstacle_lower.array() -= obstacle_padding;
    obstacle_upper.array() += obstacle_padding;

    std::vector<Eigen::Vector3d> obstacle_surface;
    const Clock::time_point obstacle_started = Clock::now();
    grid_map_->getInflatedSurfacePointsInBox(
        obstacle_lower, obstacle_upper, obstacle_surface);
    record.obstacle_extract_ms = std::chrono::duration<double, std::milli>(
                                     Clock::now() - obstacle_started)
                                     .count();

    Eigen::Matrix3d initial_pva;
    initial_pva.col(0) = start_pt;
    initial_pva.col(1) = start_vel;
    initial_pva.col(2) = start_acc;
    Eigen::Matrix3d terminal_pva = Eigen::Matrix3d::Zero();
    terminal_pva.col(0) = local_target_pt;
    terminal_pva.col(1) = local_target_vel;

    dac_sfc_deployment::EngineResult engine_result;
    const bool engine_success = dac_sfc_engine_.plan(
        route, obstacle_surface, map_lower, map_upper, initial_pva, terminal_pva,
        dac_engine_options_, engine_result);
    record.engine = engine_result.diagnostics;
    if (engine_success)
      publishDacSfcCorridors(engine_result.corridors);
    else
      publishDacSfcCorridors(std::vector<Eigen::MatrixX4d>());

    poly_traj::Trajectory candidate;
    bool sampled_collision_free = false;
    bool dynamic_limits_satisfied = false;
    std::string trajectory_safety_failure;
    if (engine_success)
    {
      std::vector<double> durations(engine_result.durations.size());
      std::vector<poly_traj::CoefficientMat> coefficients;
      coefficients.reserve(engine_result.coefficients.size());
      for (int i = 0; i < engine_result.durations.size(); ++i)
        durations[i] = engine_result.durations(i);
      for (const auto &coefficient : engine_result.coefficients)
        coefficients.push_back(coefficient);
      candidate = poly_traj::Trajectory(durations, coefficients);
      sampled_collision_free =
          trajectoryIsCollisionFree(candidate, trajectory_safety_failure);
      const double tolerance = 1.0 + std::max(0.0, pp_.feasibility_tolerance_);
      dynamic_limits_satisfied =
          (pp_.max_vel_ <= 0.0 ||
           engine_result.diagnostics.max_velocity <= pp_.max_vel_ * tolerance) &&
          (pp_.max_acc_ <= 0.0 ||
           engine_result.diagnostics.max_acceleration <= pp_.max_acc_ * tolerance);
    }

    record.sampled_collision_free = sampled_collision_free;
    record.pipeline_success =
        engine_success && sampled_collision_free && dynamic_limits_satisfied;
    if (!engine_success)
      record.failure_stage = engine_result.diagnostics.failure_stage;
    else if (!sampled_collision_free)
      record.failure_stage = trajectory_safety_failure.empty()
                                 ? "inflated_map_collision_check"
                                 : trajectory_safety_failure;
    else if (!dynamic_limits_satisfied)
      record.failure_stage = "dynamic_limits";

    if (record.pipeline_success && !dac_sfc_shadow_only_)
    {
      trajectory_activated = setLocalTraj(candidate, touch_goal);
      if (!trajectory_activated)
      {
        record.failure_stage = "trajectory_activation";
        record.pipeline_success = false;
      }
      else if (visualization_)
      {
        Eigen::MatrixXd junctions(3, candidate.getPieceNum() + 1);
        for (int i = 0; i <= candidate.getPieceNum(); ++i)
          junctions.col(i) = candidate.getJuncPos(i);
        visualization_->displayOptimalList(junctions, 90);
      }
    }

    record.trajectory_activated = trajectory_activated;
    record.ego_fallback_used = !dac_sfc_shadow_only_ && !trajectory_activated &&
                               dac_sfc_fallback_to_ego_;
    record.total_ms = std::chrono::duration<double, std::milli>(
                          Clock::now() - total_started)
                          .count();
    if (!dac_sfc_logger_.append(record))
      ROS_WARN_THROTTLE(2.0, "[DAC-SFC] Could not append deployment CSV log.");

    ROS_INFO_STREAM("[DAC-SFC] mode=" << record.execution_mode
                    << " success=" << record.pipeline_success
                    << " route=" << record.route.raw_point_count << "->"
                    << record.route.sparse_point_count
                    << " corridors=" << record.engine.corridor_count
                    << " faces=" << record.engine.total_faces
                    << " geo/call=" << record.engine.geometry_evaluations_per_call
                    << " total_ms=" << record.total_ms
                    << (record.failure_stage.empty()
                            ? std::string()
                            : " failure=" + record.failure_stage));

    return record.pipeline_success;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;
    if (traj_.swarm_traj[drone_id].drone_id != drone_id) // The trajectory is invalid
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance) )
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
