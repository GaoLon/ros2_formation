#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/msg/poly_traj.hpp>
#include <traj_utils/msg/assignment.hpp>

#include <fstream>
#include <iostream>
using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET ,
      SWARM_MANUAL_TARGET 
    };
    
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    int goal_num_;
    double goalpoints_[50][3];
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;

     // global goal setting for swarm
    Eigen::Vector3d swarm_central_pos_;
    double swarm_relative_pts_[50][3];
    double swarm_scale_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;

    GlobalTrajData frontend_traj_;

    /* rclcpp utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_, trigger_sub_, central_goal;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<traj_utils::msg::PolyTraj>::SharedPtr broadcast_ploytraj_sub_;
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_pub_, broadcast_ploytraj_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_, start_pub_;

    // result file and file name
    string result_fn_;
    fstream result_file_;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init, bool use_formation);
    
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    // void getLocalTarget();

    /* rclcpp functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);
    void triggerCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstPtr &msg);
    void RecvBroadcastPolyTrajCallback(const traj_utils::msg::PolyTraj::ConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::msg::PolyTraj &msg);
    void formationWaypointCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);
    bool frontEndPathSearching();
    bool checkCollision();

    rclcpp::Clock fsm_clock;

  public:
    EGOReplanFSM(rclcpp::Node::SharedPtr nh);
    ~EGOReplanFSM();


    void init();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif