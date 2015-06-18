#ifndef SCRIBE_MOTOR_DRIVER_H
#define SCRIBE_MOTOR_DRIVER_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/Twist.h>
#include "Robot.hpp"
#include <SerialStream.h>

class Scribe{

	protected : 
	bool _verbose;
	ros::Subscriber _scribe;
	ros::Timer _bomb;
	std::string _topic_name;
	ros::Time _timeStamp;
	geometry_msgs::Twist _cmd;
	bool _needChecking;

	public : 
	
	/****************µCONSTRUCtEUR*************************/
	
	Scribe(ros::NodeHandle ao_nh, std::string topic, Robot& p) : _verbose(false), _topic_name(topic), _timeStamp(ros::Time::now()), _cmd(), _needChecking(false) {
		//Scribe mode
		_scribe=ao_nh.subscribe<geometry_msgs::Twist>( topic,1000, boost::bind(&Scribe::callBackFunc, this, _1, &p));
		//Timer
		_bomb=ao_nh.createTimer(ros::Duration(2), boost::bind(&Scribe::bombCallBack, this, _1, &p));
	};
	
	
	/***************FUNTIONS******************************/
	
	void callBackFunc(const geometry_msgs::Twist::ConstPtr& msg, Robot* plat);
	void bombCallBack(const ros::TimerEvent&, Robot* plat);
	
	geometry_msgs::Twist& getCmd() {return _cmd;}
	void setCmd(geometry_msgs::Twist& newer){_cmd=newer;}
	void setVerbose(){_verbose=true;}
	
	void affiche();
	
};

/*********************************************************************************************************
FONCTIONS
*********************************************************************************************************/

inline void Scribe::callBackFunc(const geometry_msgs::Twist::ConstPtr& msg, Robot* plat){
	if(_verbose==true){
		std::cout << "Command receive by the sribe my general and send through the new Robot :D"<<std::endl;
		std::cout << *msg <<std::endl;
	}
	_cmd=*msg;
	_timeStamp=ros::Time::now();
	//Mise à jour de la Robot =D
	//_platoon.robot2wheels(_cmd);	
	plat->robot2wheels(_cmd);
	std::cout<<"cmd_sent"<<std::endl;
	_needChecking=true;
}

inline void Scribe::bombCallBack(const ros::TimerEvent&, Robot* plat){
	if(_verbose==true){
		std::cout<<"HELLO, my name's security timer and I'm here to same your life =D"<<std::endl;
	}
	if(_needChecking==true && ros::Time::now() - _timeStamp>ros::Duration(3)){
		if(_verbose==true){
			std::cout<<"SAVING YOUUUUU"<<std::endl;
		}
		_timeStamp=ros::Time::now();
		_cmd=geometry_msgs::Twist();
		plat->robot2wheels(_cmd);	
		_needChecking=false;
	}
}


inline void Scribe::affiche(){
	std::cout<< "I'm a scribe check on "<<_topic_name << " my lastest criting were "<<_cmd<<std::endl;
}

#endif







