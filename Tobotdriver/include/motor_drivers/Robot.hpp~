/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition Modified by kenji : Y.C.Lin , NTU,EE97,master student in 2010.04
//Third edition Modified by W.L.Hsu ,NTU,EE98,PhD student
*/
#ifndef YO_ROBOT_HPP  
#define YO_ROBOT_HPP

#include <math.h>
//#include "process.h"
#include "SerialPortControl_stream.hpp"
#include "nav_msgs/Odometry.h"
#include "Quaternion.hpp"

#define PI               3.1415926

#define MOTOR_PORT1               8

#define MOTOR_BAUDRATE      9600
#define PULSE_RATIO         135168  // 2048*66 pulses for 1 circle
#define MAXIMUM_SPEED        10000  // RPM
#define MOTION_SPEED          6000  // RPM
#define MOTION_ACCELERATION    1000  // circle/s^2 (changed for little motor)
#define MOTION_DECELERATION    1000  // circle/s^2 (changed for little motor)

class Robot{

	protected : 
	SerialPortControl _motorControl1;
	double _speed; //Speed selon x du robot in m.s
	double _angularSpeed; //angular speed in rad.s
	
	double _x;
	double _y;
	double _heading; //In radian !!
	
	double _old_posL;
	double _old_posR;
	
	//Robot carachetristique
	double _radius;
	double _wheelRadius;
	double _gearRatio;
	
	//Odometry stuff
	nav_msgs::Odometry _odomRead; //Odometry of the robot
	bool _verbose;
	
	ros::NodeHandle _pnode;

	public:
	
	Robot(ros::NodeHandle ao_nh) : _motorControl1(7500, ao_nh), _speed(0), _angularSpeed(0), _x(0), _y(0), _heading(0), _old_posL(0), _old_posR(0), _radius(1), _verbose(false), _pnode(ao_nh){
		_pnode.param<double>("WheelRadius", _wheelRadius, 0.075);
		_pnode.param<double>("Radius", _radius, 0.30);
		_pnode.param<double>("GearRatio", _gearRatio, 0.0454545);
	};
	
	Robot(ros::NodeHandle ao_nh, double r, double wr) : _motorControl1(7500, ao_nh), _speed(0), _angularSpeed(0), _radius(r), _wheelRadius(wr), _verbose(false), _pnode(ao_nh){
		//_pnode.param<double>("WheelRadius", _wheelRadius, 0.08);
		//_pnode.param<double>("WRadius", _radius, 0.30);
		_pnode.param<double>("GearRatio", _gearRatio, 0.0454545);
	};
	
	/*********************************
	Accesseurs
	*********************************/
	double getSpeed(){return _speed;}
	double getAngularSpeed(){return _angularSpeed;}
	double getX(){return _x;}
	double getY(){return _y;}
	double getHeading(){return _heading;}
	double getRadius(){return _radius;}
	double getWheelRadius(){return _wheelRadius;}
	SerialPortControl& getControl(){return _motorControl1;}
	nav_msgs::Odometry& getOdom(){return _odomRead;}
	
	void setRadius(double d){_radius=d;}
	void setWheelRadius(double wd){_wheelRadius=wd;}
	
	/************Set the speed of the two wheel left and right
	Depending on the speed X and the angular speed asked*****/
	
	void setSpeed(double s){_speed=s;}
	void setAngularSpeed(double as){_angularSpeed=as;};
	void setVerbose(){_verbose=true;_motorControl1.setVerbose();}	
	/*********************************************************
	Function conversion for the serial communication.
	We calculate the speed and change the speed to RPM.
	*********************************************************/
	
	int ms2rpm(double speed){
		return speed*60/(3.1415*_wheelRadius);
	}
	double rpm2ms(double rpm){
		return rpm*(3.1415*_wheelRadius)/60;
	}
	double radsec2rpm(double radsec){
		return radsec*9.549296596425384;
	}
	double rpm2radsec(double rpm){
		return rpm/9.549296596425384;
		
	}
	void robot2wheels(geometry_msgs::Twist& _twistDemand);
	void robot2wheels();	
	void odometry();

	
	void affiche(){std::cout<<"I'm the Robot of size "<<_radius<<" and wheels "<<_wheelRadius<<" my speed is "<< _speed<< " and angular speed "<<_angularSpeed<<std::endl;}
	
	double boundAngle(double angle1){
		while(angle1 >= 2*PI){
			angle1=angle1-2*PI;
		}
		while(angle1 <0){
			angle1=angle1+2*PI;
		}		
		return angle1;
	}

};


/*********************************************************************************************************
FONCTIONS
*********************************************************************************************************/


inline void Robot::odometry(){
	//init of all the measured values
	if(_verbose==true){
		std::cout<< "Searching for odometry..."<<std::endl;
	}
	//Prepare de reading by stating that we are in a correct reading mode. if the reading fail for X reason then the state willl be false afterward.
	_motorControl1.setReadState();
	//_motorControl1.updateMeasurement();
	if(_verbose==true){
		std::cout<< "reading SLW..."<<std::endl;
	}
	double SLW=_motorControl1.readRealSLW();
	if(_verbose==true){
		std::cout<< "reading Lencoder..."<<std::endl;
	}
	double posLW=_motorControl1.readLencoder();
	//posLW=posLW/(66*3);
	//TODO take care this come from the inervsion
	posLW=-posLW;
	
	if(_verbose==true){
		std::cout<< "reading TickNumber..."<<std::endl;
	}
	double TICKNUM=_motorControl1.getTickNumber();
	if(_verbose==true){
		std::cout<< "reading SRW..."<<std::endl;
	}
	double SRW=_motorControl1.readRealSRW();
	if(_verbose==true){
		std::cout<< "reading Rencoder..."<<std::endl;
	}
	double posRW=_motorControl1.readRencoder();
	//;posRW=posRW/(66*3);

	//posLW=posLW/1000;
	//posRW=posRW/1000;
	//TICKNUM=TICKNUM/1000;


//	double SRW=SLW; //TESTING!!
//	double posRW=posLW;
	//Need to convert from RPM to m.s
	SRW=rpm2ms(SRW);
	SLW=rpm2ms(SLW);
	ROS_INFO("the speed is");
	std::cout << SRW <<" "<< SLW<<std::endl;
	
	
	if(_motorControl1.getReadState()==true){
		//TODO GearRatio
		_odomRead.twist.twist.linear.x = _odomRead.twist.twist.linear.y = _odomRead.twist.twist.linear.z = _odomRead.twist.twist.angular.x = _odomRead.twist.twist.angular.y = _odomRead.twist.twist.angular.z=0;
		//calcul of speed
		double Speed_x=_wheelRadius*( (SRW + SLW)/2);
		double Angle=(_wheelRadius/_radius)*(SRW-SLW );
		
		Speed_x=Speed_x;
		Angle=Angle;
		
		
		if(Speed_x>0.001 || Speed_x<-0.001){
			_odomRead.twist.twist.linear.x=Speed_x;
		}
		if(Angle>0.001 || Angle<-0.001){
			_odomRead.twist.twist.angular.z=Angle;
		}
	
	
		//Calcul of position
		double angle=2*PI*(_wheelRadius/_radius)*( (posLW-posRW)/TICKNUM);
	
		if(_verbose==true){
			std::cout<< "We measured everything and it's "<<_motorControl1.getReadState()<< " and so posx is "<<posRW<<" and so posy is "<<posLW<< " speedl "<<SLW<< " tick num " << TICKNUM << " rseult " << _wheelRadius*cos(angle)*(posLW+posRW)*(PI/(TICKNUM)) << "whell radius "<< _wheelRadius << " angle " << angle << " PI " << PI<<" tick over tick "<<  (posLW-posRW)/TICKNUM<<std::endl;
		}
	
		// leftDelta and rightDelta = distance that the left and right wheel have moved along
		//  the ground
		
		double leftDelta=2*PI*(_wheelRadius)*( (posLW - _old_posL) /TICKNUM);
		double rightDelta=2*PI*(_wheelRadius)*( (posRW - _old_posR) /TICKNUM);
		
		_old_posL=posLW;
		_old_posR=posRW;

		if (fabs(leftDelta - rightDelta) < 1.0e-6) { // basically going straight
			_x = _x + leftDelta * cos(_heading);
			_y = _y + rightDelta * sin(_heading);
			_heading = _heading;
		} else {
			double R = _radius * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta)),
				wd = (rightDelta - leftDelta) / _radius;

			_x = _x + R * sin(wd + _heading) - R * sin(_heading);
			_y = _y - R * cos(wd + _heading) + R * cos(_heading);
			_heading = boundAngle(_heading + wd);
		}
	
	
		
		//_odomRead.pose.pose.position.x=_wheelRadius*cos(angle)*(posLW+posRW)*(PI/(TICKNUM));
		//_odomRead.pose.pose.position.y=_wheelRadius*sin(angle)*(posLW+posRW)*(PI/(TICKNUM));
	
		_odomRead.pose.pose.position.x=_x;
		_odomRead.pose.pose.position.y=_y;
	
		//Quaternion quat=Quaternion(0,0,angle*180/PI);
		Quaternion quat=Quaternion(0,0,_heading*180/PI);
		_odomRead.pose.pose.orientation.x=quat.getX();
		_odomRead.pose.pose.orientation.y=quat.getY();
		_odomRead.pose.pose.orientation.z=quat.getZ();
		_odomRead.pose.pose.orientation.w=quat.getW();
	
		_odomRead.header.stamp=ros::Time::now();
		_odomRead.header.frame_id="odom"; //Frame of the pose
		_odomRead.child_frame_id="odom"; //Frame of the twist
	}
}


inline void Robot::robot2wheels(geometry_msgs::Twist& _twistDemand){
	_speed=_twistDemand.linear.x;
	_angularSpeed=_twistDemand.angular.z;
	robot2wheels();
}

inline void Robot::robot2wheels(){	
	//the fuck ?
	//double rwheel =  (((2 * _speed)/_wheelRadius) + ((_angularSpeed * _radius)/ _wheelRadius))/2 ;
	//double lwheel =  -(((2 * _speed)/_wheelRadius) - ((_angularSpeed * _radius)/ _wheelRadius))/2 ;
	//double rwheel= (_speed+ (_angularSpeed * _radius) ) / _gearRatio;///_wheelRadius;
	//double lwheel= - (_speed - (_angularSpeed * _radius) ) / _gearRatio;///_wheelRadius;
	
	//Calcule the rpm from the rad sec formula :
	double rwheel=( (_radius*_angularSpeed)/(_wheelRadius*2) ) + ( _speed / _wheelRadius);
	//sign inverted because the motor is inverted
	double lwheel= ( (_radius*_angularSpeed)/(_wheelRadius*2) ) - ( _speed / _wheelRadius);
	
	//Cheat
	//Calcule the rpm from the rad sec formula :
	//double rwheel=( (_radius*_angularSpeed)/(_wheelRadius*2) ) + (3* ( _speed / _wheelRadius));
	//sign inverted because the motor is inverted
	//double lwheel= ( (_radius*_angularSpeed)/(_wheelRadius*2) ) - (3* ( _speed / _wheelRadius));
	
	//To test diameter of the robot => distance in meter per sec
	//66 comes from the reducteur INSIDE the encoder mecanism. Bitch took me forever to find out.
	rwheel=rwheel*66*3;
	lwheel=lwheel*66*3;
	if (_verbose==true){
		std::cout<<"Command envoyée au controlleur mon capitaine. On a "<<_speed<< " "<<_angularSpeed<< " donc Roue droite "<<rwheel<<" roue gauche "<<lwheel<<" avec un gear ratio de : "<< _gearRatio<<std::endl;
		std::cout << "les deux vitesses sont "<< radsec2rpm(rwheel)<<" " <<radsec2rpm(lwheel) << " pour "<< rwheel <<" "<<lwheel<< std::endl;
	}	
	//66 comes from the reducteur INSIDE the encoder mecanism. Bitch took me forever to find out.

	//Sending the new command as rpm
	//Need to add the gear ratio !!!
	double rw=0;
	double lw=0;
	try{
		if (radsec2rpm(rwheel)>7500){
			throw std::invalid_argument("rwheel too big");
		}
		else{
			try{
				if (radsec2rpm(rwheel)<-7500){
					throw std::invalid_argument("rwheel too small");
				}
				else{
					rw=radsec2rpm(rwheel);
				}
			}
			catch(std::exception const& e){
				std::cerr << "Erreur in speed command : " << e.what() << radsec2rpm(rwheel) << std::endl;
				//TODO change by a variablke
				rw=-7500;
			}
		}
	}
	catch(std::exception const& e){
		std::cerr << "Erreur in speed command : " << e.what() << radsec2rpm(rwheel) << std::endl;
		//TODO change by a variablke
		rw=7500;
	}
	
	
	try{
		if (radsec2rpm(lwheel)>7500){
			throw std::invalid_argument("lwheel too big");
		}
		else{
			try{
				if (radsec2rpm(lwheel)<-7500){
					throw std::invalid_argument("lwheel too small");
				}
				else{
					lw=radsec2rpm(lwheel);
				}
			}
			catch(std::exception const& e){
				std::cerr << "Erreur in speed command : " << e.what() << radsec2rpm(lwheel) << std::endl;
				//TODO change by a variablke
				lw=-7500;
			}
		}
	}
	catch(std::exception const& e){
		std::cerr << "Erreur in speed command : " << e.what() << radsec2rpm(lwheel) << std::endl;
		//TODO change by a variablke
		lw=7500;
	}
	
	
	_motorControl1.update(rw, lw);
}



#endif
