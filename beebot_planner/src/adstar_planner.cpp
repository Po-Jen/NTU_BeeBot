/*
 * adstar_planner.cpp
 */
 #ifndef ADSTAR_PLANNER_CPP
 #define ADSTAR_PLANNER_CPP
 
 
#include <beebot_planner/adstar_planner.h>
#include <pluginlib/class_list_macros.h>
 
 namespace adstar_planner {
	 ADStarPlanner::ADStarPlanner()
	 : costmap_ros_(NULL), initialized_(false) {}
	 
	 ADStarPlanner::ADStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	 : costmap_ros_(NULL), initialized_(false) {
		 initialize(name, costmap_ros) ;
		 }
     
     void ADStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		  if(!initialized_){
			  costmap_ros_ = costmap_ros ;
			  costmap_ = costmap_ros -> getCostmap() ;
			  
			  ros::NodeHandle private_nh ("~/" + name) ;
			  private_nh.param("step_size", step_size_, costmap_ -> getResolution()) ;
			  private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10) ;  // This number can be changed
			  world_model_ = new base_local_planner::CostmapModel(*costmap_) ;

			  private_nh.param("planner_type", planner_type_, std::string("ADPlanner"));
			  private_nh.param("search_until_first_solution", search_until_first_solution_, false);
			  private_nh.param("allocated_time", allocated_time_, 7.0);
			  private_nh.param("forward_search", forward_search_, false);
			  private_nh.param("initial_epsilon", initial_epsilon_, 3.0);
			  private_nh.param("robot_radius", robot_radius_, robot_radius_);

			  path_pub_ = private_nh.advertise<nav_msgs::Path>("planned_path", 0);						  
			  
			  
			  initialized_ = true ;
		  }
		  else{
			  ROS_WARN("This planner has already been initialized... I am therefore not doing anything XDD") ;
		  }
	 }
	 
	 double ADStarPlanner::footprintCost (double x_i, double y_i, double theta_i) {
		 if (!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return -1.0;			 
		 }
		 
		 std::vector<geometry_msgs::Point> footprint = costmap_ros_ -> getRobotFootprint() ;
		 // if we have no footprint....do nothing
		 if (footprint.size() < 3)
		    return -1.0 ;
		 
		 // check if the footprint is legal
		 double footprint_cost = world_model_ -> footprintCost(x_i, y_i, theta_i, footprint) ;
		 return footprint_cost ;
	 }

	void ADStarPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose){
		//set goal
		goal_pose_ = goal_pose->pose;
		goal_received_ = true ;
		ROS_DEBUG("Received goal: %f %f", goal_pose_.position.x, goal_pose_.position.y) ;
		if (goal_pose->header.frame_id != map_->getFrameID()){
			ROS_WARN("Goal pose frame id \"%s\" different from map frame id \"%s\"", goal_pose->header.frame_id.c_str(), map_->getFrameID().c_str());
		}
  
	  // planning?
	  if (start_received_)
		plan();		
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
		plan();		
	}
		     
	void ADStarPlanner::mapCallback(nav_msgs::OccupancyGridConstPtr& occupancy_map){
	  gridmap_2d::GridMap2DPtr map(new gridmap_2d::GridMap2D(occupancy_map));
	  updateMap(map);		
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
	
	// TODO	     	 
	bool ADStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan){
		  if(!initialized_){
			ROS_ERROR("Global planner is not initialized");
			return false;
		  }

		  plan.clear();				
		  
		  ROS_DEBUG("adstar_planner is getting fresh copy of costmap") ;
		  
		  ROS_INFO("adstar_planner getting start point (%g, %g) goal point (%g, %g)",
		           start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y) ;
          
          // TOODO TODO TODO 
          // well...maybe this is it! Every other stuff have been dealt with other member functions
          
          return true ;
          
    }      
   
   bool ADStarPlanner::plan(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal){
	  start_pose_ = start;
	  goal_pose_ = goal;

	  start_received_ = true;
	  goal_received_ = true;

	  return plan();	   
   }
   
   bool ADStarPlanner::plan(double startX, double startY, double goalX, double goalY){
	  start_pose_.position.x = startX;
	  start_pose_.position.y = startY;

	  goal_pose_.position.x = goalX;
	  goal_pose_.position.y = goalY;

	  start_received_ = true;
	  goal_received_ = true;

	  return plan();	   
   } 
   
   bool ADStarPlanner::plan() {
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

		  if (start_id < 0 || planner_-> set_start(start_id) == 0){
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

          // HERE IS THE ACTUAL PLANNING !!
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
	 }
 
 // register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(adstar_planner::ADStarPlanner, nav_core::BaseGlobalPlanner)
 #endif
