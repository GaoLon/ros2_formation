

#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  EGOReplanFSM::~EGOReplanFSM()
  {
    // result_file_.close();
  }
  EGOReplanFSM::EGOReplanFSM(rclcpp::Node::SharedPtr nh):fsm_clock(RCL_SYSTEM_TIME)
  {
    node_ = nh;
    init();
  }
  void EGOReplanFSM::init()
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    flag_relan_astar_ = false;
    have_local_traj_ = false;

    /*  fsm param  */
    rclcpp::Parameter target_type_p, replan_thresh_p, no_replan_thresh_p, planning_horizen_p,\
                      planning_horizen_time_p, emergency_time_p, flag_realworld_experiment_p,\
                      enable_fail_safe_p, replan_trajectory_time_p;
    node_->get_parameter_or("fsm/flight_type", target_type_p, rclcpp::Parameter("fsm/flight_type", -1));
    node_->get_parameter_or("fsm/thresh_replan_time", replan_thresh_p, rclcpp::Parameter("fsm/thresh_replan_time", -1.0));
    node_->get_parameter_or("fsm/thresh_no_replan_meter", no_replan_thresh_p, rclcpp::Parameter("fsm/thresh_no_replan_meter", -1.0));
    node_->get_parameter_or("fsm/planning_horizon", planning_horizen_p, rclcpp::Parameter("fsm/planning_horizon", -1.0));
    node_->get_parameter_or("fsm/planning_horizen_time", planning_horizen_time_p, rclcpp::Parameter("fsm/planning_horizen_time", -1.0));
    node_->get_parameter_or("fsm/emergency_time", emergency_time_p, rclcpp::Parameter("fsm/emergency_time", 1.0));
    node_->get_parameter_or("fsm/realworld_experiment", flag_realworld_experiment_p, rclcpp::Parameter("fsm/realworld_experiment", false));
    node_->get_parameter_or("fsm/fail_safe", enable_fail_safe_p, rclcpp::Parameter("fsm/fail_safe", true));
    // node_->get_parameter_or("fsm/result_file", result_fn_, string("/home/zuzu/Documents/Benchmark/21-RSS-ego-swarm/2.24/ego/ego_swarm.txt"));
    node_->get_parameter_or("fsm/replan_trajectory_time", replan_trajectory_time_p, rclcpp::Parameter("fsm/replan_trajectory_time", 0.0));

    target_type_ = target_type_p.as_int();
    replan_thresh_ = replan_thresh_p.as_double();
    no_replan_thresh_ = no_replan_thresh_p.as_double();
    planning_horizen_ = planning_horizen_p.as_double();
    planning_horizen_time_ = planning_horizen_time_p.as_double();
    emergency_time_ = emergency_time_p.as_double();
    flag_realworld_experiment_ = flag_realworld_experiment_p.as_bool();
    enable_fail_safe_ = enable_fail_safe_p.as_bool();
    replan_trajectory_time_ = replan_trajectory_time_p.as_double();

    have_trigger_ = !flag_realworld_experiment_;

    // node_->get_parameter_or("fsm/waypoint_num", waypoint_num_, -1);
    // for (int i = 0; i < waypoint_num_; i++)
    // {
    //   node_->get_parameter_or("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    //   node_->get_parameter_or("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    //   node_->get_parameter_or("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    // }

    // node_->get_parameter_or("fsm/goal_num", goal_num_, -1);
    // for (int i = 0; i < goal_num_; i++)
    // {
    //   node_->get_parameter_or("fsm/target" + to_string(i) + "_x", goalpoints_[i][0], -1.0);
    //   node_->get_parameter_or("fsm/target" + to_string(i) + "_y", goalpoints_[i][1], -1.0);
    //   node_->get_parameter_or("fsm/target" + to_string(i) + "_z", goalpoints_[i][2], -1.0);
    // }

    rclcpp::Parameter swarm_scale_p;
    rclcpp::Parameter swarm_relative_pts_p;
    
    // for (int i = 0; i < 7; i++)
    // {
    //   node_->get_parameter_or("global_goal/relative_pos_" + to_string(i) + "/x", swarm_relative_pts_[i][0], -1.0);
    //   node_->get_parameter_or("global_goal/relative_pos_" + to_string(i) + "/y", swarm_relative_pts_[i][1], -1.0);
    //   node_->get_parameter_or("global_goal/relative_pos_" + to_string(i) + "/z", swarm_relative_pts_[i][2], -1.0);
    // }
    for (int i = 0; i < 7; i++)
    {
      node_->get_parameter_or("global_goal/relative_pos_" + to_string(i), swarm_relative_pts_p, rclcpp::Parameter("global_goal/relative_pos_" + to_string(i), (-1.0, -1.0, -1.0)));
      std::vector<double> a = swarm_relative_pts_p.as_double_array();
      // std::cout<<a[0]<<"  "<<a[1]<<"  "<<a[2]<<"  "<<std::endl;
      swarm_relative_pts_[i][0] = a[0];
      swarm_relative_pts_[i][1] = a[1];
      swarm_relative_pts_[i][2] = a[2];
    }

    node_->get_parameter_or("global_goal/swarm_scale", swarm_scale_p, rclcpp::Parameter("global_goal/swarm_scale", 1.0));
    swarm_scale_ = swarm_scale_p.as_double();
    // RCLCPP_ERROR(node_->get_logger(), "SHJOTTTT");

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    // RCLCPP_ERROR(node_->get_logger(),"VVVVVVVVVVVVVVVVVVVVVV");
    planner_manager_.reset(new EGOPlannerManager);
    // RCLCPP_ERROR(node_->get_logger(),"pPPPPPPPPPPPPPPPPPPPP");
    planner_manager_->initPlanModules(node_, visualization_);
    // RCLCPP_ERROR(node_->get_logger(),"NNNNNNNNNNNNNNNNNNNNN");
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&EGOReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&EGOReplanFSM::checkCollisionCallback, this));
    // exec_timer_ = node_.createTimer(rclcpp::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    // safety_timer_ = node_.createTimer(rclcpp::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
          
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("odometry", 1, \
                      [this](const nav_msgs::msg::Odometry::ConstPtr msg){odometryCallback(msg);}); 
    // odom_sub_ = node_.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    broadcast_ploytraj_pub_ = node_->create_publisher<traj_utils::msg::PolyTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = node_->create_subscription<traj_utils::msg::PolyTraj>("planning/broadcast_traj_recv", 100,\
                                    [this](const traj_utils::msg::PolyTraj::ConstPtr msg){RecvBroadcastPolyTrajCallback(msg);}); 
                                                                //  &EGOReplanFSM::RecvBroadcastPolyTrajCallback,
                                                                //  this,
                                                                //  rclcpp::TransportHints().tcpnode_lay());

    poly_traj_pub_ = node_->create_publisher<traj_utils::msg::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    start_pub_ = node_->create_publisher<std_msgs::msg::Bool>("planning/start", 1);
    reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>("planning/finish", 1);

    // result_file_.open(result_fn_, ios::app);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/goal", 1, \
                            [this](const geometry_msgs::msg::PoseStamped::ConstPtr msg){waypointCallback(msg);});
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    { 
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", 1, \
                            [this](const geometry_msgs::msg::PoseStamped::ConstPtr msg){triggerCallback(msg);});
      RCLCPP_INFO(node_->get_logger(),"Wait for 2 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 2000)
      {
        // rclcpp::spinOnce();
        // rclcpp::Duration(0.001).sleep();
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
      }
      RCLCPP_WARN(node_->get_logger(),"Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        // rclcpp::spinOnce();
        // rclcpp::Duration(0.001).sleep();
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
      }
      std_msgs::msg::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = rclcpp::Time(fsm_clock.now(), RCL_SYSTEM_TIME);
      planner_manager_->start_flag_ = true;
      start_pub_->publish(flag_msg);
      planGlobalTrajbyGivenWps();
    }
    else if (target_type_ == TARGET_TYPE::SWARM_MANUAL_TARGET)
    {
      central_goal = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, \
                            [this](const geometry_msgs::msg::PoseStamped::ConstPtr msg){formationWaypointCallback(msg);});
    }
    cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    // exec_timer_.stop(); // To avoid blockage
    exec_timer_->cancel();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(1);

        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),"Failed to generate the first trajectory!!!");
          changeFSMExecState(SEQUENTIAL_START, "FSM");
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      bool success = planFromGlobalTraj(1);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      bool success;
      if (flag_relan_astar_)
        success = planFromLocalTraj(true, false);
      else
        success = planFromLocalTraj(false, true);

      if (success)
      {
        flag_relan_astar_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        flag_relan_astar_ = true;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      // std::cout<<"begin_exec\n";
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      // std::cout<<"begin_sub\n";
      double t_cur = (fsm_clock.now() - info->start_time).seconds();
      // std::cout<<"end_sub\n";
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);

      if ((local_target_pt_ - end_pt_).norm() < 0.1) // local target close to the global target
      {
        if (t_cur > info->duration - 0.2)
        {
          have_target_ = false;
          have_local_traj_ = false;

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");

          // result_file_ << planner_manager_->pp_.drone_id << "\t" << (fsm_clock.now() - planner_manager_->global_start_time_).seconds() << "\t" << planner_manager_->average_plan_time_ << "\n";

          printf("\033[47;30m\n[drone %d reached goal]==============================================\033[0m\n",
                 planner_manager_->pp_.drone_id);
          std_msgs::msg::Bool msg;
          msg.data = true;
          reached_pub_->publish(msg);
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = rclcpp::Time(fsm_clock.now(), RCL_SYSTEM_TIME);
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    exec_timer_->reset();
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(),"Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (fsm_clock.now() - info->start_time).seconds();
    Eigen::Vector3d p_cur = info->traj.getPos(t_cur);
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    rclcpp::Time t_cur_global = rclcpp::Time(fsm_clock.now(), RCL_SYSTEM_TIME);
    double t_2_3 = planner_manager_->ploy_traj_opt_->getCollisionCheckTimeEnd();
    double t_temp;
    bool occ = false;
    for (double t = t_cur; t < info->duration; t += time_step)
    {
      // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      if (map->getInflateOccupancy(info->traj.getPos(t)) == 1)
      {
        RCLCPP_WARN(node_->get_logger(),"drone %d is too close to the obstacle at relative time %f!",
                 planner_manager_->pp_.drone_id, t / info->duration);
        t_temp = t;
        occ = true;
        break;
      }

      for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
      {
        if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
            (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        // std::cout<<"begin_checkco\n";
        double t_X = (t_cur_global - planner_manager_->traj_.swarm_traj.at(id).start_time).seconds();
        // std::cout<<"end_checkco\n";

        if (t_X > planner_manager_->traj_.swarm_traj.at(id).duration)
          continue;

        Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          RCLCPP_WARN(node_->get_logger(),"swarm distance between drone %d and drone %d is %f, too close!",
                   planner_manager_->pp_.drone_id, id, dist);
          t_temp = t;
          occ = true;
          break;
        }
      }
    }

    if (occ)
    {
      /* Handle the collided case immediately */
      RCLCPP_INFO(node_->get_logger(),"Try to replan a safe trajectory");
      if (planFromLocalTraj(true, false)) // Make a chance
      // if (planFromLocalTraj(false, true))
      {
        RCLCPP_INFO(node_->get_logger(),"Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }
      else
      {
        if (t_temp - t_cur < emergency_time_) // 1.0s of emergency time
        {
          RCLCPP_WARN(node_->get_logger(),"Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(),"current traj in collision, replan.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::msg::PoseStamped::ConstPtr&msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->pose.position.x, msg->pose.position.y, 1.0;

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),"Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    // RCLCPP_ERROR(node_->get_logger(), "GET_ODOODODO");
  }

  void EGOReplanFSM::RecvBroadcastPolyTrajCallback(const traj_utils::msg::PolyTraj::ConstPtr &msg)
  {
    std::cout<<"SHIT_TRAJ_HERE\n";
    if (msg->drone_id < 0)
    {
      RCLCPP_ERROR(node_->get_logger(),"drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      RCLCPP_ERROR(node_->get_logger(),"Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
      RCLCPP_ERROR(node_->get_logger(),"WRONG trajectory parameters.");
      return;
    }
    if (abs((fsm_clock.now() - msg->header.stamp).seconds()) > 0.25)
    {
      RCLCPP_WARN(node_->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs",
               msg->drone_id, (fsm_clock.now() - msg->header.stamp).seconds());
      return;
    }

    const size_t recv_id = (size_t)msg->drone_id;
    std::cout<<"recv_id="<<recv_id<<std::endl;
    if ((int)recv_id == planner_manager_->pp_.drone_id)
      return;

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->header.stamp;

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    poly_traj::Trajectory trajectory(dura, cMats);
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }

          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::msg::PolyTraj &msg)
  {

    auto data = &planner_manager_->traj_.local_traj;

    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.traj_id = data->traj_id;
    // std::cout<<"data_start_time="<<data->start_time<<std::endl;
    // msg.header.stamp.set__sec(static_cast<int32_t>(data->start_time));
    msg.header.stamp = data->start_time;
    msg.order = 5; // todo, only support order = 5 now.

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
  }
  
  void EGOReplanFSM::formationWaypointCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;

    bool success = false;
    swarm_central_pos_(0) = msg->pose.position.x;
    swarm_central_pos_(1) = msg->pose.position.y;
    swarm_central_pos_(2) = 0.5;

    int id = planner_manager_->pp_.drone_id;

    Eigen::Vector3d relative_pos;
    relative_pos << swarm_relative_pts_[id][0],
                    swarm_relative_pts_[id][1],
                    swarm_relative_pts_[id][2];
    end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);


    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(SEQUENTIAL_START, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),"Unable to generate global trajectory!");
    }
  }
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps;
    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      wps.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        wps[i](0) = waypoints_[i][0];
        wps[i](1) = waypoints_[i][1];
        wps[i](2) = waypoints_[i][2];
      }
      end_pt_ = wps.back();
      for (size_t i = 0; i < (size_t)waypoint_num_; i++)
      {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
        // rclcpp::Duration(0.001).sleep();
      }
    }
    else
      return;

    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero(), wps,
                                                             Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero());

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
      // rclcpp::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      // rclcpp::Duration(0.001).sleep();
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),"Unable to generate global trajectory!");
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, false, true))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(bool flag_use_poly_init, bool use_formation)
  {
    double t_debug_start = fsm_clock.now().seconds();
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = (fsm_clock.now() - info->start_time).seconds();

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(flag_use_poly_init, false, use_formation);

    if (!success)
      return false;

    // cout << "planFromLocalTraj : " << fsm_clock.now().seconds() - t_debug_start << endl;

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_);

    Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc;
    rclcpp::Time desired_start_time = fsm_clock.now();
    if (have_local_traj_ && use_formation)
    {
      desired_start_time = rclcpp::Time(fsm_clock.now() + rclcpp::Duration(static_cast<rcl_duration_value_t>(replan_trajectory_time_*1e9)), RCL_SYSTEM_TIME);
      // desired_start_time = rclcpp::Time(fsm_clock.now() + rclcpp::Duration(replan_trajectory_time_*1e+9), RCL_SYSTEM_TIME);
      desired_start_pt =
          planner_manager_->traj_.local_traj.traj.getPos((desired_start_time - planner_manager_->traj_.local_traj.start_time).seconds());
      desired_start_vel =
          planner_manager_->traj_.local_traj.traj.getVel((desired_start_time - planner_manager_->traj_.local_traj.start_time).seconds());
      desired_start_acc =
          planner_manager_->traj_.local_traj.traj.getAcc((desired_start_time - planner_manager_->traj_.local_traj.start_time).seconds());
    }
    else
    {
      desired_start_pt = start_pt_;
      desired_start_vel = start_vel_;
      desired_start_acc = start_acc_;
    }
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, use_formation, have_local_traj_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::msg::PolyTraj msg;
      polyTraj2ROSMsg(msg);
      poly_traj_pub_->publish(msg);
      broadcast_ploytraj_pub_->publish(msg);
      // std::cout<<"publish down\n";
      have_local_traj_ = true;
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::msg::PolyTraj msg;
    polyTraj2ROSMsg(msg);
    poly_traj_pub_->publish(msg);

    return true;
  }

} // namespace ego_planner
