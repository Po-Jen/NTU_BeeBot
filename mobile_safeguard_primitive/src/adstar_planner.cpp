/**
 * Copyright 2016 Charly Huang, National Taiwan University
 * 
 * adstar_planner.cpp
 * 
 * This software is developed using SBPL to implement Anytime Dynamic A*
 * algorithm. 
 **/

#include "mobile_safeguard_primitive/adstar_planner.h"
#include <gridmap_2d/GridMap2D.h>

// Announce that is file is a pluggin
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ADStarPlanner::ADStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std ;

namespace ADStarPlanner {
	// default constructor
	ADStarPlanner::ADStarPlanner()
    : nh_(),
	robot_radius_(0.35),
	start_received_(false), goal_received_(false),
	path_costs_(0.0){
			    // private NodeHandle for parameters:
                ros::NodeHandle nh_private("~");
                nh_private.param("planner_type", planner_type_, std::string("ADPlanner"));
                nh_private.param("search_until_first_solution", search_until_first_solution_, false);
                nh_private.param("allocated_time", allocated_time_, 7.0);
                nh_private.param("forward_search", forward_search_, false);
                nh_private.param("initial_epsilon", initial_epsilon_, 3.0);
                nh_private.param("robot_radius", robot_radius_, robot_radius_);

                path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0);		
	}
	
	ADStarPlanner::ADStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, costmap_ros) ;
	}
	
	void ADStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		if (!initialized_){
		}	
			
		 
	}
	
	// ************************************
    // IMPLEMENT THE CLASS FUNCTIONS here and the planning inside of makePlan ()
    void ADStarPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose){
      // set goal:
      goal_pose_ = goal_pose->pose;
      goal_received_ = true;
      ROS_DEBUG("Received goal: %f %f", goal_pose_.position.x, goal_pose_.position.y);

      if (goal_pose->header.frame_id != map_->getFrameID()){
        ROS_WARN("Goal pose frame id \"%s\" different from map frame id \"%s\"", goal_pose->header.frame_id.c_str(), map_->getFrameID().c_str());
      }
  
       // planning?
       if (start_received_)
          //plan();
          makePlan() ;
    }
    
    void ADStarPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map){
  gridmap_2d::GridMap2DPtr map(new gridmap_2d::GridMap2D(occupancy_map));
  updateMap(map);
}
	
	void ADStarPlanner::setPlanner(){
  if (planner_type_ == "ADPlanner"){
    planner_.reset(new ADPlanner(planner_environment_.get(),forward_search_));
  } else if (planner_type_ == "RSTARPlanner"){
    planner_.reset(new RSTARPlanner(planner_environment_.get(),forward_search_));
  }
}

   void ADStarPlanner::startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose){
	  // set start:
	  start_pose_ = start_pose->pose.pose;
	  start_received_ = true;
	  ROS_DEBUG("Received start: %f %f", start_pose_.position.x, start_pose_.position.y);

	  if (start_pose->header.frame_id != map_->getFrameID()){
		ROS_WARN("Start pose frame id \"%s\" different from map frame id \"%s\"", start_pose->header.frame_id.c_str(), map_->getFrameID().c_str());
	  }
	  
	  // planning?
	  if (goal_received_)
		//plan();
		makePlan() ;
}

   bool ADStarPlanner::updateMap(gridmap_2d::GridMap2DPtr map){
	  planner_environment_.reset(new EnvironmentNAV2D());
	  planner_environment_->InitializeEnv(int(map->getInfo().width), int(map->getInfo().height), 0, OBSTACLE_COST);
	  // environment is set up, reset planner:
	  setPlanner();

	  // store local copy:
	  map_.reset(new gridmap_2d::GridMap2D(*map));
	  map_->inflateMap(robot_radius_);


	  for(unsigned int j = 0; j < map_->getInfo().height; ++j){
		for(unsigned int i = 0; i < map_->getInfo().width; ++i){
		  if (map_->isOccupiedAtCell(i,j))
			planner_environment_->UpdateCost(i, j, OBSTACLE_COST);
		  else
			planner_environment_->UpdateCost(i,j,0);

		}
	  }

	  ROS_DEBUG("Map set");

	  return true;
   }
	// DONE IMPLEMENTING FUNCIONS
	//****************************************
	bool ADStarPlanner::makePlan(const geometry_msgs::Pose& start, 
	                             const geometry_msgs::Pose& goal){	 
		/*
		if(!initialized_) {
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner") ;
			return false ;
		}
				
		ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.position.x, start.position.y,
                   goal.position.x, goal.position.y);
      */ 
	  // Add my own stuff !!
	  /*
	  start_pose_ = start;
      goal_pose_ = goal;

      start_received_ = true;
      goal_received_ = true;

      return plan();
      */ 
   } 
/*   
   bool ADStarPlanner::makePlan(double startX, double startY, double goalX, double goalY){
		if(!initialized_) {
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner") ;
			return false ;
		}
				
		ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                   goal.pose.position.x, goal.pose.position.y);
           
        plan.clear() ;	
        									 
        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
           ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                        costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
           return false ;
	   }
	   
	   // Add my own stuff
      start_pose_.position.x = startX;
      start_pose_.position.y = startY;

     goal_pose_.position.x = goalX;
     goal_pose_.position.y = goalY;

     start_received_ = true;
     goal_received_ = true;

     return plan();	   	   
   }
   
   bool ADStarPlanner::plan(){
	  path_.poses.clear();

	  if (!map_){
		ROS_ERROR("Map not set");
		return false;
	  }

	  unsigned start_x, start_y, goal_x, goal_y;
	  if (!map_->worldToMap(start_pose_.position.x, start_pose_.position.y, start_x, start_y)){
		ROS_ERROR("Start coordinates out of map bounds");
		return false;
	  }
	  if (!map_->worldToMap(goal_pose_.position.x, goal_pose_.position.y, goal_x, goal_y)){
		ROS_ERROR("Goal coordinates out of map bounds");
		return false;
	  }

	  if (map_->isOccupiedAtCell(start_x, start_y)){
		ROS_ERROR("Start coordinate (%f %f) is occupied in map", start_pose_.position.x, start_pose_.position.y);
		return false;
	  }
	  if (map_->isOccupiedAtCell(goal_x, goal_y)){
		ROS_ERROR("Goal coordinate (%f %f) is occupied in map", goal_pose_.position.x, goal_pose_.position.y);
		return false;
	  }

	  int start_id = planner_environment_->SetStart(start_x, start_y);
	  int goal_id = planner_environment_->SetGoal(goal_x, goal_y);

	  if (start_id < 0 || planner_->set_start(start_id) == 0){
		ROS_ERROR("Failed to set start state");
		return false;
	  }

	  if (goal_id < 0 || planner_->set_goal(goal_id) == 0){
		ROS_ERROR("Failed to set goal state");
		return false;
	  }

	  // set planner params:
	  planner_->set_initialsolution_eps(initial_epsilon_);
	  planner_->set_search_mode(search_until_first_solution_);
	  std::vector<int> solution_stateIDs;
	  int solution_cost;


	  if(planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost))
		ROS_DEBUG("Solution found. Costs: %d;  final eps: %f", solution_cost, planner_->get_final_epsilon());
	  else{
		ROS_INFO("Solution not found");
		return false;
	  }

	  // scale costs (SBPL uses mm and does not know map res)
	  path_costs_ = double(solution_cost) / ENVNAV2D_COSTMULT * map_->getResolution();

	  // extract / publish path:
	  path_.poses.reserve(solution_stateIDs.size());
	  path_.header.frame_id = map_->getFrameID();
	  path_.header.stamp = ros::Time::now();

	  geometry_msgs::PoseStamped pose;
	  pose.header = path_.header;
	  for (size_t i = 0; i < solution_stateIDs.size(); i++) {
		int mx, my;
		planner_environment_->GetCoordFromState(solution_stateIDs[i], mx, my);
		//ROS_INFO("p: %d - [%d %d]", solution_stateIDs[i], mx, my);
		double wx,wy;
		map_->mapToWorld(mx,my,wx,wy);


		pose.pose.position.x = wx;
		pose.pose.position.y = wy;
		pose.pose.position.z = 0.0;
		path_.poses.push_back(pose);
	  }

	  path_pub_.publish(path_);

	  return true;   
   }
   */
} ;
