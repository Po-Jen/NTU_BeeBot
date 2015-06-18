#ifndef YO_FILTER_HPP  
#define YO_FILTER_HPP
#include <vector>
#include <deque>
#include <queue>
#include "nav_msgs/Odometry.h"


class Filter{

public:
	// push_back and pop_front
	std::list<nav_msgs::Odometry> filter;
	
	Filter(){};
	
	void add(nav_msgs::Odometry& a){
		if (filter.size()>10){
			filter.pop_front();;
		}
		filter.push_back(a);
		
	}
	
	int getSize(){return filter.size();}
	
	nav_msgs::Odometry filtering(){
		nav_msgs::Odometry sum;
		for (std::list<nav_msgs::Odometry>::iterator iter=filter.begin(); iter!=filter.end();){
			//sum=sum+(*iter);
			
			sum.pose.pose.position.x=sum.pose.pose.position.x+(*iter).pose.pose.position.x;
			sum.pose.pose.position.y=sum.pose.pose.position.y+(*iter).pose.pose.position.y;
			
			sum.pose.pose.orientation.x=sum.pose.pose.orientation.x+(*iter).pose.pose.orientation.x;
			sum.pose.pose.orientation.y=sum.pose.pose.orientation.y+(*iter).pose.pose.orientation.y;
			sum.pose.pose.orientation.z=sum.pose.pose.orientation.z+(*iter).pose.pose.orientation.z;
			sum.pose.pose.orientation.w=sum.pose.pose.orientation.w+(*iter).pose.pose.orientation.w;
			iter++;
		}
		sum.header.stamp=ros::Time::now();
		sum.header.frame_id="odom"; //Frame of the pose
		sum.child_frame_id="odom"; //Frame of the twist
		
		sum.pose.pose.position.x=sum.pose.pose.position.x/filter.size();
		sum.pose.pose.position.y=sum.pose.pose.position.y/filter.size();
		sum.pose.pose.orientation.x=sum.pose.pose.orientation.x/filter.size();
		sum.pose.pose.orientation.y=sum.pose.pose.orientation.y/filter.size();
		sum.pose.pose.orientation.z=sum.pose.pose.orientation.z/filter.size();
		sum.pose.pose.orientation.w=sum.pose.pose.orientation.w/filter.size();
		
		//std::cout<<"filtering results "<< sum;

		return sum;
	}


};

#endif
