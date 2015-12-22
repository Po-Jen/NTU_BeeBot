/**
 * Copyright 2015 Charly Huang, National Taiwan University
 * Dec 21, 2015 16:49
 * 
 * This file is intended to convert /path topic to /traj used
 * exclusively by riskrrt's controller.
 */
 
#include <ros/ros.h>
#include <iostream>

// Subscribe to /path topic
#include "nav_msgs/Path.h"
#include <geometry_msgs/Pose.h>
#include <riskrrt/PoseTwistStamped.h>
#include <tf/tf.h>

// Calling in functions within riskrrt package
#include <riskrrt/Trajectory.h>
#include <riskrrt/riskrrt.hpp>

using namespace std ;

ros::Publisher traj_pub ;

void pathCallback (const nav_msgs::Path::ConstPtr &msg){
	riskrrt::Trajectory traj_msgs ;
	riskrrt::PoseTwistStamped pose_twist ;
	
	traj_msgs.poses.clear() ;
	
	for (int i = 0 ; i < msg -> poses.size(); i++ ) {
		pose_twist.pose.position.x = msg -> poses[i].pose.position.x ;
	    pose_twist.pose.position.y = msg -> poses[i].pose.position.y ;
	    pose_twist.pose.position.z = 0.0 ;                                   /// should be canceled if we were to do 3D planning
	    pose_twist.pose.orientation = msg -> poses[i].pose.orientation ;
	}
	
	traj_msgs.poses.push_back (pose_twist) ;

	// Publish msgs into the topic
	traj_pub.publish (traj_msgs) ;
} 

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "path2traj_conv") ;
	ros::NodeHandle nh ;
	
	// create ROS subscriber
	ros::Subscriber traj_sub = nh.subscribe ("/path", 1, pathCallback ) ;
	
	// create ROS publisher
	traj_pub = nh.advertise<riskrrt::Trajectory> ("/traj", 10) ;
	
	// Spin
	ros::spin() ;
}
