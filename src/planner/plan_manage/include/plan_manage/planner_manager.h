#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <optimizer/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>
#include <chrono>

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
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
        const rclcpp::Time trajectory_start_time, const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, 
        const bool flag_polyInit, const bool flag_randomPolyTraj,
        const bool use_formation, const bool have_local_traj);
    bool computeInitReferenceState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, 
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const double &ts, poly_traj::MinJerkOpt &initMJO,
        const bool flag_polyInit);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, 
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints, 
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel);
    void initPlanModules(rclcpp::Node::SharedPtr nh, PlanningVisualization::Ptr vis = NULL);
    // void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool EmergencyStop(Eigen::Vector3d stop_pos);

    void deliverTrajToOptimizer(void) { ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj); };

    void setDroneIdtoOpt(void) { ploy_traj_opt_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return ploy_traj_opt_->getSwarmClearance(); }

    bool checkCollision(int drone_id);

    void fakeSwarmTrajs(void);

    PlanParameters pp_;
    // LocalTrajData local_data_;
    // GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    // SwarmTrajData swarm_trajs_;
    TrajContainer traj_;
    
    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    bool start_flag_, reach_flag_;
    rclcpp::Time global_start_time_{0L, RCL_SYSTEM_TIME};
    rclcpp::Clock manager_clock;
    double start_time_, reach_time_, average_plan_time_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;
    rclcpp::Node::SharedPtr node;

    int continous_failures_count_{0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
