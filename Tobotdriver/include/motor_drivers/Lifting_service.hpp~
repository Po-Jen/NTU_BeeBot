#ifndef _LIFTERMALCOLM_HPP
#define _LIFTERMALCOLM_HPP

#include <string.h>
#include <Lift_platform.hpp>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"

#include "tobotdrivers/lifting.h"

/****SERVICE can TALK BACK !*/
bool lift(tobotdrivers::lifting::Request  &req, tobotdrivers::lifting::Response &res, Lift_platform* p, ros::Publisher* pose_pub){
	ROS_INFO("Request : %s", req.way.c_str());
	if(!strcmp(req.way.c_str(),"up")){
		
		p->up();
		ros::Time time=ros::Time::now();
		while(ros::Time::now()-time<ros::Duration(5) ){
			
		}
		ROS_INFO("End of up movement");
		//Wait for it to go down
		std_msgs::Bool boole;
		boole.data=true;
		pose_pub->publish<std_msgs::Bool>(boole);
		res.answer=true;
		
	}
	else if(!strcmp(req.way.c_str(),"down")){
		
		p->down();
		ros::Time time=ros::Time::now();
		while(ros::Time::now()-time<ros::Duration(5) ){
			
		}
		ROS_INFO("End of down movement");
		std_msgs::Bool boole;
		boole.data=false;
		pose_pub->publish<std_msgs::Bool>(boole);
		res.answer=true;

	}	
	//res.answer=req.order;
	return true;
	
}


class Lifter{
	public : 
	ros::Publisher pose_pub;
	double hightobject;
	double maxdistance;
	ros::ServiceServer service;
	Lift_platform platform;
		
		
	Lifter(ros::NodeHandle node, ros::NodeHandle priv_node) : platform(priv_node, 0 ,0) {
		pose_pub =node.advertise<std_msgs::Bool>("pose_pub", 1000);
		priv_node.param<double>("hightobject", hightobject, 10);
		priv_node.param<double>("maxTick",maxdistance, 5);
		//platform(priv_node, hightobject, maxdistance);
		platform.setHight(hightobject);
		platform.setMaxDistance(maxdistance);
		
		service = node.advertiseService<tobotdrivers::lifting::Request, tobotdrivers::lifting::Response>("lifting_service", boost::bind(lift, _1, _2, &platform, &pose_pub));
	ROS_INFO("Ready to change model");
		
	}

	
	

};

#endif
