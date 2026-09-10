#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <cstdint>
#include <string>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>
#include <dac_sfc/dac_route_adapter.h>
#include <dac_sfc/dac_sfc_engine.h>
#include <dac_sfc/deployment_logger.h>
#include <decomp_ros_msgs/PolyhedronArray.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool checkCollision(int drone_id);
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);
    bool setLocalTraj(const poly_traj::Trajectory &trajectory, const bool touch_goal);
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    bool dac_sfc_enabled_{false};
    bool dac_sfc_shadow_only_{true};
    bool dac_sfc_fallback_to_ego_{true};
    std::string dac_sfc_data_source_{"simulation"};
    bool dac_sfc_visualization_enabled_{true};
    std::string dac_sfc_visualization_frame_{"world"};
    ros::Publisher dac_sfc_polyhedron_pub_;
    dac_sfc_deployment::RouteOptions dac_route_options_;
    dac_sfc_deployment::EngineOptions dac_engine_options_;
    dac_sfc_deployment::DacRouteAdapter::Ptr dac_route_adapter_;
    dac_sfc_deployment::DacSfcEngine dac_sfc_engine_;
    dac_sfc_deployment::DeploymentLogger dac_sfc_logger_;
    std::uint64_t dac_sfc_run_id_{0};

    bool dacSfcReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool touch_goal,
        bool &trajectory_activated);
    bool trajectoryIsCollisionFree(
        const poly_traj::Trajectory &trajectory,
        std::string &failure_stage) const;
    void publishDacSfcCorridors(
        const std::vector<Eigen::MatrixX4d> &corridors) const;

    int continous_failures_count_{0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
