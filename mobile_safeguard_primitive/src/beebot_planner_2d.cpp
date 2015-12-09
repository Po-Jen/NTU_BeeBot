/*
 * Copyright 2015 Charly Huang, National Taiwan University
 * 
 * This software is develop to actually implement search based path 
 * planning algorithm D* Lite proposed by Sven Koenigh in hish
 * 2012 work. 
 * 
 * It only subscribes to start, goal coordinates and the occupancy grid
 * map. The rest is done within beebot_DStarLite_planner.h
 */
 
# include <ros/ros.h>
#include "mobile_safeguard_primitive/beebot_DStarLite_planner.h" 

class SBPLPlanner2DNode{
public:
  SBPLPlanner2DNode(){
    ros::NodeHandle nh;

    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &SBPLPlanner2D::mapCallback, &planner_);
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &SBPLPlanner2D::goalCallback, &planner_);
    start_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &SBPLPlanner2D::startCallback, &planner_);

  }

  virtual ~SBPLPlanner2DNode(){}

protected:
  SBPLPlanner2D planner_;
  ros::Subscriber map_sub_, goal_sub_, start_sub_;


};

int main(int argc, char** argv){
  ros::init(argc, argv, "beebot_planner_2d");

  SBPLPlanner2DNode planner;

  ros::spin();

  return 0;
}
