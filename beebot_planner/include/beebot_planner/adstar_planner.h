/*
 * adstar_planner.h
*/

#ifndef ADSTAR_PLANNER_H
#define ADSTAR_PLANNER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>
#include <iostream>
#include <set>

/**ROS**/
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace adstar_planner{
	/**
	 * @class adstar_planner
	 * @brief provides AD* motion planner as a global planner plugin.
	 */ 
	 class ADStarPlanner : public nav_core::BaseGlobalPlanner {
		 public: 
		    /**
		     * @brief Constructor for ADStarPlaner
		     */
		     ADStarPlanner () ;
		     
		     /**
		      * @brief Constructor for the ADStarPlanner
		      * @param name The name of this planner
		      * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
		      */ 
		     ADStarPlanner (std::string name, costmap_2d::Costmap2DROS* costmap_ros) ;
		     
		     /**
		      * @function initialize (name, costmap_ros)
		      * @brief Initialization function for the Carrot Planner
		      * @param 
		      */ 
		     void initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) ;
		     
		     /**
		      * @function makePlan(start, goal, plan)
		      * @brief Given a goal pose in the world, compute a plan
		      * @param start The start pose
		      * @param goal The goal pose
		      * @param plan The plan filled by the planner
		      * @return True if a a valid plan was found, false otherwise
		      */ 
		      bool makePlan (const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) ;
		      
		  private:
		      costmap_2d::Costmap2DROS* costmap_ros_ ;
		      double step_size_, min_dist_from_robot_ ;
		      costmap_2d::Costmap2D* costmap_ ;
		      base_local_planner::WorldModel* world_model_ ;    ///@brief The world model that the controller will use
		      
		      /**
		       * @function footprintCost (x_i, y_i, theta_i)
		       * @brief Checks the legality of the robot footprint at a position and orientation using the world model
		       * @param x_i The x position of the robot
		       * @param y_i The y position of the robot
		       * @param theta_i The orientation of the robot
		       * @return
		       */ 
		      double footprintCost(double x_i, double y_i, double theta_i) ;
		      
		      bool initialized_ ;
	 } ;
}

#endif
