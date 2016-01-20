/*
 * adstar_planner.cpp
 */
 #ifndef ADSTAR_PLANNER_CPP
 #define ADSTAR_PLANNER_CPP
 
 
#include <beebot_planner/adstar_planner.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(adstar_planner::ADStarPlanner, nav_core::BaseGlobalPlanner)
 
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
	 
	 bool ADStarPlanner::makePlan (const geometry_msgs::PoseStamped& start,
	                               const geometry_msgs::PoseStamped& goal,
	                               std::vector<geometry_msgs::PoseStamped>& plan){
			if (!initialized_){
				ROS_ERROR("The planner has not been initialized. Please call initialize() to use the planner") ;
				return false ;
			}
			
			ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", 
			          start.pose.position.x, start.pose.position.y,
			          goal.pose.position.x, goal.pose.position.y) ;
			
			plan.clear() ;
			costmap_ = costmap_ros_ -> getCostmap() ;

			if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
				ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
				costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
				return false;
			}
			
			tf::Stamped<tf::Pose> goal_tf ;
			tf::Stamped<tf::Pose> start_tf ;
			
			poseStampedMsgToTF(goal, goal_tf) ;
			poseStampedMsgToTF(start, start_tf) ;			
			
			bool done = false ;
			/**
			 * @TODO 
			 */ 
			 if(!done){
			 }
			 
			return (done);
				
	 }
 }
 #endif
