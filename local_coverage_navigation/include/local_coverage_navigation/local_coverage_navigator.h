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
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>


#include <nav_core/base_local_planner.h>

#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <actionlib/server/simple_action_server.h>
#include <local_coverage_navigation/NavigateAction.h>

#include <dynamic_reconfigure/server.h>

namespace local_coverage_navigation {

typedef actionlib::SimpleActionServer<local_coverage_navigation::NavigateAction> NavigateActionServer;


  enum LocalCoverageNavigatorState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class LocalCoverageNavigator
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class LocalCoverageNavigator {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      LocalCoverageNavigator(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~LocalCoverageNavigator();

      /**
       * @brief  Performs a control cycle
       * @return 1 if processing of the goal is done, 0 if processing of the
        goal should continue, and -1 if it aborted
       */
      int executeCycle();

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

    void testThread();

      void planThread();

      int executeCb(const local_coverage_navigation::NavigateGoalConstPtr& move_base_goal);


      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf2_ros::Buffer& tf_;


      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* controller_costmap_ros_;

      std::string robot_base_frame_, global_frame_;


      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      std::vector<std::string> recovery_behavior_names_;
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      double oscillation_timeout_, oscillation_distance_;

      NavigateActionServer* as_;
      local_coverage_navigation::NavigateResult as_result;
      local_coverage_navigation::NavigateFeedback as_feedback;

      LocalCoverageNavigatorState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;
      boost::condition_variable_any planner_cond_;
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;
      boost::thread* test_thread;
      bool run_test_thread_;

      double current_angle;
      double cost_threshold;

      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };


class CalcCost
{
public:
  CalcCost(unsigned char* costmap) :
      costmap_(costmap), cost(0)
  {
  }
  inline void operator()(unsigned int offset)
  {
    auto t = costmap_[offset];
    cost += costmap_[offset];
  }

  unsigned int cost;
private:
  unsigned char* costmap_;

};


inline int sign(int x)
{
  return x > 0 ? 1.0 : -1.0;
}

/**
* @brief  Raytrace a line and apply some action at each step
* @param  at The action to take... a functor
* @param  x0 The starting x coordinate
* @param  y0 The starting y coordinate
* @param  x1 The ending x coordinate
* @param  y1 The ending y coordinate
* @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
*/
template<class ActionType>
inline void raytraceLine(ActionType& at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int size_x,
                         unsigned int max_length = UINT_MAX)
{
  int dx = x1 - x0;
  int dy = y1 - y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * size_x;

  unsigned int offset = y0 * size_x + x0;

  // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
  double dist = hypot(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

  // if x is dominant
  if (abs_dx >= abs_dy)
  {
    int error_y = abs_dx / 2;
    bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
    return;
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;
  bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

/**
 * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
 */
template<class ActionType>
inline void bresenham2D(ActionType& at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length)
{
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i)
  {
    at(offset);
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da)
    {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  at(offset);
}
};
#endif
