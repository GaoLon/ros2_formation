// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <visualization_msgs/msg/marker.hpp> // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager():manager_clock(RCL_SYSTEM_TIME) {}

  EGOPlannerManager::~EGOPlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    node = nh;
    rclcpp::Parameter max_vel_param, max_acc_param, feasibility_tolerance_param, planning_horizen_param,\
                      ctrl_pt_dist_param, polyTraj_piece_length_param, use_distinctive_trajs_pram, drone_id_s_param; 
    nh->get_parameter_or("manager/max_vel", max_vel_param, rclcpp::Parameter("manager/max_vel", -1.0));
    nh->get_parameter_or("manager/max_acc", max_acc_param, rclcpp::Parameter("manager/max_acc", -1.0));
    nh->get_parameter_or("manager/feasibility_tolerance", feasibility_tolerance_param, rclcpp::Parameter("manager/feasibility_tolerance", 0.0));
    nh->get_parameter_or("manager/control_points_distance", ctrl_pt_dist_param, rclcpp::Parameter("manager/control_points_distance", -1.0));
    nh->get_parameter_or("manager/polyTraj_piece_length", polyTraj_piece_length_param, rclcpp::Parameter("manager/polyTraj_piece_length", -1.0));
    nh->get_parameter_or("manager/planning_horizon", planning_horizen_param, rclcpp::Parameter("manager/planning_horizon", 5.0));
    nh->get_parameter_or("manager/use_distinctive_trajs", use_distinctive_trajs_pram, rclcpp::Parameter("manager/use_distinctive_trajs", false));
    nh->get_parameter_or("manager/drone_id", drone_id_s_param, rclcpp::Parameter("manager/drone_id", "drone_0"));
    pp_.max_vel_ = max_vel_param.as_double();
    pp_.max_acc_ = max_acc_param.as_double();
    pp_.feasibility_tolerance_ = feasibility_tolerance_param.as_double();
    pp_.ctrl_pt_dist = ctrl_pt_dist_param.as_double();
    pp_.polyTraj_piece_length = polyTraj_piece_length_param.as_double();
    pp_.use_distinctive_trajs = use_distinctive_trajs_pram.as_bool();
    pp_.drone_id = std::atoi(&(drone_id_s_param.as_string().back()));
    // RCLCPP_ERROR(nh->get_logger(),"MALLLLLLLLLLAMAMMAMMMAMM");

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);
    // RCLCPP_ERROR(nh->get_logger(),"MMMMMMMMMMMMMMMMMMMAP");

    ploy_traj_opt_.reset(new PolyTrajOptimizer(nh));
    ploy_traj_opt_->setParam();
    // RCLCPP_ERROR(nh->get_logger(),"OOOOOOOOOOOOOOOOOOOPT");
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;
  }

  bool EGOPlannerManager::computeInitReferenceState(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Vector3d &start_vel,
                                                    const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &local_target_pt,
                                                    const Eigen::Vector3d &local_target_vel,
                                                    const double &ts,
                                                    poly_traj::MinJerkOpt &initMJO,
                                                    const bool flag_polyInit)
  {
    static bool flag_first_call = true;

    /*** case 1: use A* initialization ***/
    if (flag_first_call || flag_polyInit)
    {
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      poly_traj::Trajectory traj;
      vector<Eigen::Vector3d> simple_path;
      constexpr double init_of_init_totaldur = 2.0;

      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* step 1: A* search and generate init traj */
      Eigen::MatrixXd ctl_points;

      // traj = ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points);
      ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points, initMJO);
      traj = initMJO.getTraj();

      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      visualization_->displayAStarList(path_view, 0);

      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);
    }

    /*** case 2: initialize from previous optimal trajectory ***/
    else
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt.seconds() < 0.0)
      {
        RCLCPP_ERROR(node->get_logger(), "You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        // rclcpp_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = (manager_clock.now() - traj_.local_traj.start_time).seconds();
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt).seconds();
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
          double glb_t = t - t_to_lc_end + (traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time).seconds();
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          // rclcpp_ERROR("Should not happen! x_x 0x88");
          RCLCPP_ERROR(node->get_logger(), "Should not happen! x_x 0x88");
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
      Eigen::Vector3d &local_target_vel)
  {
    // double t;
    rclcpp::Time t;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    // double t_step = planning_horizen / 20 / pp_.max_vel_;
    // rclcpp::Duration t_step = rclcpp::Duration(planning_horizen / 20 / pp_.max_vel_*1e+9);
    rclcpp::Duration t_step = rclcpp::Duration(static_cast<rcl_duration_value_t>(planning_horizen / 20 / pp_.max_vel_*1e9));
    rclcpp::Duration tgtd = rclcpp::Duration(static_cast<rcl_duration_value_t>(traj_.global_traj.duration*1e9));
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + tgtd);
        //  t < (traj_.global_traj.global_start_time + rclcpp::Duration(traj_.global_traj.duration*1e+9));
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos((t - traj_.global_traj.global_start_time).seconds());
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time).seconds() >= traj_.global_traj.duration) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + tgtd;
      // traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + rclcpp::Duration(traj_.global_traj.duration*1e+9);
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel((t - traj_.global_traj.global_start_time).seconds());
    }
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const rclcpp::Time trajectory_start_time, const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj,
      const bool use_formation, const bool have_local_traj)
  {
    static int count = 0;

    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
    }

    rclcpp::Time t_start = rclcpp::Time(manager_clock.now(), RCL_SYSTEM_TIME);

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

   
    poly_traj::MinJerkOpt initMJO;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc,
                                   local_target_pt, local_target_vel,
                                   ts, initMJO, flag_polyInit))
    {
      return false;
    }


    Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    ploy_traj_opt_->setControlPoints(cstr_pts);

    rclcpp::Duration t_init = manager_clock.now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = rclcpp::Time(manager_clock.now(), RCL_SYSTEM_TIME);

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState,
                                                            innerPts, initTraj.getDurations(),
                                                            cstr_pts, use_formation);
 
    rclcpp::Duration t_opt = manager_clock.now() - t_start;

    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).seconds();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).seconds()
         << "\033[0m,init:" << t_init.seconds()
         << ",optimize:" << t_opt.seconds()
         << ",avg_time=" << sum_time / count_success
         << ",count_success= " << count_success << endl;
    average_plan_time_ = sum_time / count_success;

    if (have_local_traj && use_formation)
    {
      double delta_replan_time = (trajectory_start_time - manager_clock.now()).seconds();
      if (delta_replan_time > 0)
        // rclcpp::Duration(delta_replan_time).sleep();
        rclcpp::sleep_for(std::chrono::milliseconds((int)(delta_replan_time*1000)));
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time);
    }
    else
    {
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), manager_clock.now()); // todo time
    }

    visualization_->displayOptimalList(cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
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

    traj_.setLocalTraj(stopMJO.getTraj(), manager_clock.now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time.seconds() < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time.seconds();
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < ploy_traj_opt_->getSwarmClearance())
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
      for (int i = 0; i < waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }
    else
    {
      if (innerPts.size() != 0)
      {
        // rclcpp_ERROR("innerPts.size() != 0");
        RCLCPP_ERROR(node->get_logger(), "innerPts.size() != 0");
      }
    }
    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_;
    Eigen::VectorXd time_vec(waypoints.size());
    int try_num = 0;
    do
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }
      globalMJO.generate(innerPts, time_vec);
      // cout << "try_num : " << try_num << endl;
      // cout << "max vel : " << globalMJO.getTraj().getMaxVelRate() << endl;
      // cout << "time_vec : " << time_vec.transpose() << endl;

      des_vel /= 1.2;
      try_num++;
    } while (globalMJO.getTraj().getMaxVelRate() > pp_.max_vel_ && try_num <= 5);

    auto time_now = rclcpp::Time(manager_clock.now(), RCL_SYSTEM_TIME);
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now);

    return true;
  }

} // namespace ego_planner
