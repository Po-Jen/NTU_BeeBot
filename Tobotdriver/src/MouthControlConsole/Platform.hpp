/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition Modified by kenji : Y.C.Lin , NTU,EE97,master student in 2010.04
//Third edition Modified by W.L.Hsu ,NTU,EE98,PhD student
*/
#ifndef _PLATFORM_HPP  
#define _PLATFORM_HPP

#include <math.h>
#include "process.h"
#include "SerialPortControl.hpp"

#define PI               3.1415926

#define MOTOR_PORT1               8

#define MOTOR_BAUDRATE      9600
#define PULSE_RATIO         135168  // 2048*66 pulses for 1 circle
#define MAXIMUM_SPEED        10000  // RPM
#define MOTION_SPEED          6000  // RPM
#define MOTION_ACCELERATION    1000  // circle/s^2 (changed for little motor)
#define MOTION_DECELERATION    1000  // circle/s^2 (changed for little motor)

class Platform
{
public:
	
	Platform(int motor_port1,int baud_rate);
	
	///////////////////////////////////////////////////////////////////////
	void Mouth(bool bState); //jaw function
	void Mouth_initialize(bool bState); //initialization
	/////////////////////////////////////////////////////////////////////
	
	//encoder detect
	/*void watchMotion();
	static void thrWatchMotion(void*);*/

	//set motor parameter
	void resetMotorEncoder();
	void setSpeed(int speed_rpm);
	void setAcceleration(int acc);
	void setDeceleration(int dec);
	
	//check state
	bool isIdle(){return _isIdle;}
	int speed;
	int acceleration;
	int deceleration;

//protected:

	bool _isIdle;
	bool _isMotionWatching;
	SerialPortControl* _motorControl1;
	
};


#endif