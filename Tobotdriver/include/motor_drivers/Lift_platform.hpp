#ifndef _PLATFORMLIFTER_HPP
#define _PLATFORMLIFTER_HPP
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <deque>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <SerialStream.h>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "SerialPortControl_stream.hpp"


class Lift_platform{
private : 
	SerialPortControl _stream;
	double _hight; //hight in meter
	double _maxDistance;
	
public : 
	
	/*The lifting platform constructor. priv_node give the parameters to the stream and high_object is the actual hight of the object in meter and max is the maximum distance covered by the mechanism in meter*/
	Lift_platform(ros::NodeHandle priv_node, double hight_object, double max) : _stream(7500, priv_node), _hight(hight_object), _maxDistance(max){
		_stream.writeHomeLifter();
	}
	
	/*Mutators*/
	
	double getHight(){return _hight;}
	double getMaxDistance(){return _maxDistance;}
	SerialPortControl& getSerialPortControl(){return _stream;}
	
	void setHight(double h){_hight=h;}
	void setMaxDistance(double m){_maxDistance=m;}
	void setNumberTick4OneTurn(int n){_stream.setTickNumberLift(n);}
	
	
	void reset();
	void up();
	void down();

};


inline void Lift_platform::reset()
{
	_stream.writeHomeLifter();
}


inline void Lift_platform::up()
{

	//Calcul of the way up.
	double up_value=_hight*_stream.getTickNumberLift()/_maxDistance;
	std::cout<<"the tick value is "<<up_value<< " with "<<_stream.getTickNumberLift()<<std::endl;
	_stream.writePoseAbsoluteLifter(up_value);
	_stream.writeMoveLifter();
}

inline void Lift_platform::down()
{
	_stream.writePoseAbsoluteLifter(0); //Going to home position define at the initialization
	_stream.writeMoveLifter();
}

#endif
