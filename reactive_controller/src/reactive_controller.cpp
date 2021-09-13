/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <reactive_controller/reactive_controller.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdio.h>
#include <random>

namespace reactive_controller {

  ReactiveController::ReactiveController(tf2_ros::Buffer& tf) :
    tf_(tf),
   controller_costmap_ros_(NULL),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), new_global_plan_(false) {

    ros::NodeHandle private_nh("~");
    as_ = new NavigateActionServer(private_nh, "navigate", boost::bind(&ReactiveController::executeCb, this, _1), false);
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("local_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("local_costmap/global_frame", global_frame_, std::string("odom"));
    private_nh.param("planner_frequency", planner_frequency_, 2.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&ReactiveController::planThread, this));


    //for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("reactive_controller");

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);


    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    controller_costmap_ros_->start();

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &ReactiveController::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("reactive_controller","Stopping costmaps initially");
      controller_costmap_ros_->stop();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();
    ROS_ERROR("Server started");
  }


  bool ReactiveController::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    return true;
  }



  ReactiveController::~ReactiveController(){

    delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    tc_.reset();
  }


  void ReactiveController::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

inline double abs_angle_diff(const double x, const double y)
{
  return M_PI - fabs(fmod(fabs(x - y), 2*M_PI) - M_PI);
}


  void ReactiveController::planThread(){
    ROS_DEBUG_NAMED("reactive_controller_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    auto planner_rate = ros::Rate(planner_frequency_);
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    current_angle = 0;
    cost_threshold = 1000;
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_ERROR("Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();
      lock.unlock();
      planner_plan_->clear();
      // ROS_INFO_STREAM("Current angle:" << current_angle);
      const auto size_x = this->controller_costmap_ros_->getCostmap()->getSizeInCellsX();
      const auto size_y = this->controller_costmap_ros_->getCostmap()->getSizeInCellsY();
      const uint mid_x = size_x / 2;
      const uint mid_y = size_y / 2;
      int cheapest_goal_i = -1;
      uint cheapest_cost = -1;
      double best_angle_diff = 2 * M_PI;
      uint steps = 6;
      uint half_steps = steps / 2;
      uint steps3 = steps * 3;
      uint steps4 = steps * 4;
      uint steps32 = steps3 / 2;
      uint x_step = size_x / steps;
      uint y_step = size_y / steps;
      for (int i = 0; i < steps4; i++) {
        int goal_xi = abs((int)((i % steps4) - steps32)) - half_steps;
        int goal_yi = abs((int)(((i + steps) % steps4) - steps32)) - half_steps;
        goal_xi = std::max(0, std::min(goal_xi, (int)steps));
        goal_yi = std::max(0, std::min(goal_yi, (int)steps));
        goal_xi *= x_step;
        goal_yi *= y_step;
        auto line_cost = reactive_controller::CalcCost(this->controller_costmap_ros_->getCostmap()->getCharMap());
        raytraceLine(line_cost, mid_x, mid_y, goal_xi, goal_yi, size_x);
        double angle = atan2(goal_yi - (int)mid_y, goal_xi - (int)mid_x);
        double angle_diff = abs_angle_diff(current_angle, angle);
        //ROS_INFO_STREAM("cost" << line_cost.cost << " angle: " << angle << " diff: " << angle_diff);
        if (line_cost.cost < cheapest_cost) {
          cheapest_cost = line_cost.cost;
          cheapest_goal_i = i;
          best_angle_diff = angle_diff;
        } else if (line_cost.cost == cheapest_cost && angle_diff < best_angle_diff){
          cheapest_cost = line_cost.cost;
          cheapest_goal_i = i;
          best_angle_diff = angle_diff;
        }
      }
      //ROS_INFO_STREAM("Best cost" << cheapest_cost << " Best diff: " << best_angle_diff);
      if (cheapest_cost > cost_threshold) {

        ROS_ERROR_STREAM_THROTTLE(1, "No plan beneath threshold " << cost_threshold);
      } else {
        int goal_xi = abs((int)((cheapest_goal_i % steps4) - steps32)) - half_steps;
        int goal_yi = abs((int)(((cheapest_goal_i + steps) % steps4) - steps32)) - half_steps;
        goal_xi = std::max(0, std::min(goal_xi, (int)steps));
        goal_yi = std::max(0, std::min(goal_yi, (int)steps));
        goal_xi *= x_step;
        goal_yi *= y_step;

        double start_x;
        double start_y;
        this->controller_costmap_ros_->getCostmap()->mapToWorld(mid_x, mid_y, start_x, start_y);

        double goal_x;
        double goal_y;
        this->controller_costmap_ros_->getCostmap()->mapToWorld(goal_xi, goal_yi, goal_x, goal_y);

        double goal_angle = atan2(goal_y - start_y, goal_x - start_x);
        geometry_msgs::PoseStamped current_pose;
        getRobotPose(current_pose, controller_costmap_ros_);
        if (goal_angle != current_angle or distance(current_pose, planner_goal_) < 0.2) {

          current_angle = goal_angle;

          // First get the angle to an odom frame quat
          tf2::Quaternion q;
          q.setRPY(0, 0, goal_angle);
          geometry_msgs::Quaternion goal_angle_in_base_link = tf2::toMsg(q);

          // Now find a point in the direction of the quat.
          // Start with the straight ahead in base_link
          geometry_msgs::PoseStamped goal_pose;
          goal_pose.header.frame_id = global_frame_;
          goal_pose.pose.position.x = goal_x;
          goal_pose.pose.position.y = goal_y;
          goal_pose.pose.orientation = goal_angle_in_base_link;

          current_goal_pub_.publish(goal_pose);

          for (int i = 0; i < 100; i++) {
            geometry_msgs::PoseStamped step;
            step.header.frame_id = global_frame_;
            double p = i * 0.01;
            step.pose.position.x =
                (1 - p) * start_x + p * goal_pose.pose.position.x;
            step.pose.position.y =
                (1 - p) * start_y + p * goal_pose.pose.position.y;
            step.pose.orientation = goal_pose.pose.orientation;
            planner_plan_->push_back(step);
          }

          std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;
          lock.lock();
          planner_plan_ = latest_plan_;
          latest_plan_ = temp_plan;
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          planner_goal_ = goal_pose;
          new_global_plan_ = true;

          lock.unlock();
        }
        ROS_ERROR("Generated a plan from the base_global_planner");

      }
      // Even if the plan doesn't change, we need to break out of the
      // planning state to hit the next recovery behaviors
      if (runPlanner_)
        state_ = CONTROLLING;
      if (planner_frequency_ <= 0)
        runPlanner_ = false;
      planner_rate.sleep();

      lock.lock();
    }
  }

  int ReactiveController::executeCb(const reactive_controller::NavigateGoalConstPtr& goal)
  {
    ROS_ERROR("LOOP START");

    publishZeroVelocity();


    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("reactive_controller","Starting up costmaps that were shut down previously");
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {

      if (as_->isPreemptRequested())
      {
        ROS_INFO("Action Preempted");
        // set the action state to preempted
        runPlanner_ = false;
        publishZeroVelocity();
        as_->setPreempted();
        return -1;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();
      //the real work on pursuing a goal is done here
      int status = executeCycle();

      // Effort -1 is "don't stop unless preempted"
      if (goal->effort != -1) {
        // if we're done, then we'll return from execute
        if (status == 1) {
          as_->setSucceeded(as_result);

          return 0;
        } else if (status == -1) {
          as_->setAborted(as_result);
          return -1;
        }
      }


      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("reactive_controller","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }


    //if the node is killed then we'll abort and return
    as_->setAborted(as_result);

    return -1;
  }

  double ReactiveController::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  int ReactiveController::executeCycle(){
    //boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;


    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, controller_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;
    as_feedback.base_position = global_pose;
    as_->publishFeedback(as_feedback);


    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return 0;
    }


    if(new_global_plan_) {
      new_global_plan_ = false;

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();

      ROS_DEBUG_NAMED("reactive_controller", "Got a new plan...swap pointers");

      if (!tc_->setPlan(*controller_plan_)) {
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        return -1;
      } else
      {
        ROS_ERROR("Plan set, planner still going");
        // HAX: leave the planner running, see if we replan around obstacle areas fast enough
        //runPlanner_ = false;
      }
    }

    //make sure to reset recovery_index_ since we were able to find a valid plan
    if(recovery_trigger_ == PLANNING_R)
      recovery_index_ = 0;


    //ROS_ERROR_STREAM("Cycling " << state_);
    //the reactive_controller state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG("Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
        ROS_DEBUG_NAMED("reactive_controller","In controlling state.");

        //check to see if we've reached our goal
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("reactive_controller","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          return 1;
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          ROS_ERROR("Oscillation detected");
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
          break;
        }

        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        if(tc_->computeVelocityCommands(cmd_vel)){
          //ROS_ERROR("Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
           //                cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {
          ROS_ERROR("The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            ROS_ERROR("CONTROLLER PATIENCE EXPIRED, CLEARING");
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            ROS_ERROR("No valid control, planner on");
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case CLEARING:
        ROS_ERROR("In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if(recovery_behavior_enabled_){
          ROS_ERROR("Executing behavior %u", recovery_index_+1);
          switch (recovery_index_) {
          case 0: {
            controller_costmap_ros_->resetLayers();
            // The planner is going to kick in and blindly make a path through
            // the empty costmap. Turn in place a bit to fill the map, hopefully
            // prevent blind arc motions into actual obstacles
            auto rate = ros::Rate(10);
            int i = 0;
            while (i < 80) {
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = .4;
              vel_pub_.publish(cmd_vel);
              rate.sleep();
              i++;
            }

          }
            break;
          case 1: {
            // Back out
            auto rate = ros::Rate(10);
            int i = 0;
            while (i < 20) {
              cmd_vel.linear.x = -0.1;
              cmd_vel.angular.z = 0;
              vel_pub_.publish(cmd_vel);
              rate.sleep();
              i++;
            }
            controller_costmap_ros_->resetLayers();
            break;
          }
          case 2: {
            // CSE2 curb jump
            auto rate = ros::Rate(10);
            int i = 0;
            while (i < 100) {
              cmd_vel.linear.x = -1;
              cmd_vel.angular.z = sin(8 * (float)i / 100);
              vel_pub_.publish(cmd_vel);
              rate.sleep();
              i++;
            }
            break;
          }
          case 3: {
            // CSE2 curb jump aggressive
            auto rate = ros::Rate(10);
            int i = 0;
            while (i < 100) {
              cmd_vel.linear.x = -3;
              cmd_vel.angular.z = 3 * sin(8 * (float)i / 100);
              vel_pub_.publish(cmd_vel);
              rate.sleep();
              i++;
            }
            break;
          }
          default:
            break;
          }

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("reactive_controller_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{
          ROS_DEBUG_NAMED("reactive_controller_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("reactive_controller_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
          }
          resetState();
          return -1;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        return -1;
    }

    //we aren't done yet
    return 0;
  }

  void ReactiveController::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("reactive_controller","Stopping costmaps");
      controller_costmap_ros_->stop();
    }
  }

  bool ReactiveController::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
