/*
 * Copyright 2016 Charly Huang, National Taiwan University
 * 
 * adstar_planner.h
 * 
 * This software is developed using SBPL to implement Anytime Dynamic A*
 * algorithm. 
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
 
/** include the libraries you need in your planner here */
/** for global path planner interface */

// ROS and the big guys (why are they called the big guys? Can they be called the beasts?)
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h> 
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// Motion planning specific includes
#include <sbpl/headers.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <gridmap_2d/GridMap2D.h>

// messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::string ;

#ifndef _ADSTAR_PLANNER_H_
#define _ADSTAR_PLANNER_H_

namespace ADStarPlanner  {

class ADStarPlanner : public nav_core::BaseGlobalPlanner {
public:
   ADStarPlanner () ;
   ADStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) ;

   // Here I make a slight modification to suit into Global Planner Plugin convention   
   /** overriden classes from interface nav_core::BaseGlbalPlanner **/
   void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) ;
   
   /**
   * @brief Plans from start to goal, assuming that the map was set with updateMap().
   * When successful, you can retrieve the path with getPath().
   *
   * @return success of planning
   */              
   bool makePlan(const geometry_msgs::Pose& start, 
	        	const geometry_msgs::Pose& goal) ;        	

   //bool makePlan(double startX, double startY, double goalX, double goalY);

  // Add my stuff in !!
  /// Set goal and plan when start was already set
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  /// Set start and plan when goal was already set
  void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start);
  /// calls updateMap()
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);
  /// Setup the internal map representation and initialize the SBPL planning environment
  bool updateMap(gridmap_2d::GridMap2DPtr map);

namespace gridmap_2d{ 
    gridmap_2d::GridMap2DPtr getMap() { return map_;};
  } ;
  
  /**
   * @brief Plans from start to goal, assuming that the map was set with updateMap().
   * When successful, you can retrieve the path with getPath().
   *
   * @param start
   * @param goal
   * @return success of planning
   */

  /// @return costs of the path (=length in m), if planning was successful
  inline double getPathCosts() const{return path_costs_;};

  inline const nav_msgs::Path& getPath() const{return path_;};
  inline double getRobotRadius() const{return robot_radius_;};

protected:
  //bool plan();
  bool makePlan() ;
  void setPlanner(); ///< (re)sets the planner
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_, start_sub_, map_sub_;
  ros::Publisher path_pub_;
  boost::shared_ptr<SBPLPlanner> planner_;
  boost::shared_ptr<EnvironmentNAV2D> planner_environment_;
  gridmap_2d::GridMap2DPtr map_;

  std::string planner_type_;
  double allocated_time_;
  double initial_epsilon_;
  bool search_until_first_solution_;
  bool forward_search_;
  double robot_radius_;

  bool start_received_, goal_received_;
  geometry_msgs::Pose start_pose_, goal_pose_;
  nav_msgs::Path path_;
  double path_costs_;
  
  static const unsigned char OBSTACLE_COST = 20;  
  
  bool initialized_ ;
  costmap_2d::Costmap2DROS* costmap_ros_ ;
   
  } ;		
} ;

#endif
