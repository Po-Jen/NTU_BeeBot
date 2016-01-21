/*
 * adstar_planner.h
 * 
 * Charly Huang, Jan 21, 2016, National Taiwan University
 * 
 * This is an adaption of SBPLPlanner2D to global planner plugin, but 
 * runs only on AD* motion planning algorithm.
*/

/*
 * Copyright 2013 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/**SBPL elements*/
#include <sbpl/headers.h>

/**grid_map_2D elements*/
#include <gridmap_2d/GridMap2D.h>

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
		      * Destructor
		      */
		     virtual ~ADStarPlanner () ;  
		     
		     /**
		      * @function goalCallback(goal)
		      * @param goal
		      * @brief Set goal and plan when start was already set
		      */
		     void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose) ;
		      
		     /**
		       * @function startCallback(start)
		       * @param start
		       * @brief Set start and plan when goal was already set
		       */
		     void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose) ; 
		     
		     /**
		      * @function mapCallback(map)
		      * @param occupancy_map
		      * @brief Calls updateMap ()
		      */   
		     void mapCallback(nav_msgs::OccupancyGridConstPtr& occupancy_map) ;
		     
		     /**
		      * @function updateMap (map)
		      * @param map The map in grid_map_2D format
		      * @brief Setup the internal map representation and initialize the SBPL planning environment
		      */ 
		     bool updateMap(gridmap_2d::GridMap2DPtr map) ; 
		     
		     /**
		      * @function getMap()
		      * @brief using grid_map_2D API to transform the map from occupancy grid to 2D grid
		      */ 
		     gridmap_2d::GridMap2DPtr getMap() const { return map_;};
		      
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
		     bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan) ;
		     
		     /**
		      * @function plan (start, goal)
		      * @function plan(startX, startY, goalX, goalY)
		      * @param start
		      * @param goal
		      * @param startX, startY
		      * @param goalX, goalY
		      * @brief makePlan calls the plan(.) function to solve the motion planning problem. Here, both functions with arguments load in the value
		      * and the private method plan() carries on the actual work.
		      */  
		     bool plan (const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) ;
		     bool plan (double startX, double startY, double goalX, double goalY) ;
		     
		     /**
		      * @function getPathCosts()
		      * @brief return costs of the path (=length in m), if planning was successful 
		      */ 
		     inline double getPathCosts() const {return path_costs_ ;} ; 
		     
		     inline const nav_msgs::Path& getPath() const {return path_ ;} ;
		     inline double getRobotRadius() const {return robot_radius_ ;} ;
		     
		  private:
		   
		      costmap_2d::Costmap2DROS* costmap_ros_ ;
		      double step_size_, min_dist_from_robot_ ;
		      costmap_2d::Costmap2D* costmap_ ;
		      base_local_planner::WorldModel* world_model_ ;    ///@brief The world model that the controller will use
		      
		      bool plan() ;
		      void setPlanner() ;  /// (re)sets the planner
		      ros::Subscriber goal_sub_, start_sub_, map_sub_ ;
		      ros::Publisher path_pub_ ;
		      boost::shared_ptr<SBPLPlanner> planner_ ;                   
		      boost::shared_ptr<EnvironmentNAV2D> planner_environment_ ;
		      gridmap_2d::GridMap2DPtr map_;
		      
		      std::string planner_type_ ;
		      double allocated_time_ ;
		      double initial_epsilon_ ;
		      bool search_until_first_solution_ ;
		      bool forward_search_ ;
		      double robot_radius_ ;
		      
		      bool start_received_, goal_received_ ;
		      geometry_msgs::Pose start_pose_, goal_pose_ ;
		      nav_msgs::Path path_ ;
		      double path_costs_ ;
		      
		      static const unsigned char OBSTACLE_COST = 20 ;
		      
		      bool initialized_ ;
		      
		      /**
		       * @function footprintCost (x_i, y_i, theta_i)
		       * @brief Checks the legality of the robot footprint at a position and orientation using the world model
		       * @param x_i The x position of the robot
		       * @param y_i The y position of the robot
		       * @param theta_i The orientation of the robot
		       * @return
		       */ 
		      double footprintCost(double x_i, double y_i, double theta_i) ;
	 } ;
}

#endif
