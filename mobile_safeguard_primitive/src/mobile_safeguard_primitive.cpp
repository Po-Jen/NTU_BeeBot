#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include <unistd.h>

using namespace std;

#define PI 3.1415926

const double linear_speed = 0.2;
const double angular_speed = 0.6;
const double angular_ratio = 1.23;
const double linear_ratio = 1.010408;
const int rate = 10;
geometry_msgs::Twist msg;
ros::Publisher pub;

void stop(ros::Rate &loop_rate)
{
	msg.linear.x = 0;
	msg.angular.z = 0;
	pub.publish(msg);
	loop_rate.sleep();
}

void moveLinear(ros::Rate &loop_rate, double meter)
{
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	
	if(meter > 0)
	{
		double duration = meter/linear_speed;
		int ticks = (int)(duration*rate*linear_ratio);

		std::cout << "Linear ticks:" << ticks << "\n";

		msg.linear.x = linear_speed;
	
		for(int i=0; i<ticks; i++)
		{
			pub.publish(msg);
			loop_rate.sleep();
		}
		
		stop(loop_rate);
	}
	else if(meter < 0)
	{
		meter *= -1;
		double duration = meter/linear_speed;
		int ticks = (int)(duration*rate*linear_ratio);

		std::cout << "Linear ticks:" << ticks << "\n";

		msg.linear.x = -1*linear_speed;
	
		for(int i=0; i<ticks; i++)
		{
			pub.publish(msg);
			loop_rate.sleep();
		}

		stop(loop_rate);
	}
}

//Counterclockwise->positive angle value
void rotate(ros::Publisher &pub, ros::Rate &loop_rate, double degree_angle)
{
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = angular_speed;

	if(degree_angle < 0)
	{
		degree_angle *= -1;
		msg.angular.z = -1*angular_speed;
	}	

	double rad_angle = (degree_angle/180)*PI;
	double duration = rad_angle/angular_speed;
	int ticks = (int)(duration*rate*1.23);
	
	std::cout << "Rotate ticks:" << ticks << "\n";
	
	for(int i=0; i<ticks; i++)
	{
		pub.publish(msg);
		loop_rate.sleep();
	}
	stop(loop_rate);
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "Mobile_Guard");
	ros::NodeHandle _nh;
	
	pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate loop_rate(rate);
	
#if 1 
	moveLinear(loop_rate, 2.1);
	rotate(pub, loop_rate, 90);
	moveLinear(loop_rate, 0.7);
	rotate(pub, loop_rate, 60);
	moveLinear(loop_rate, 1.8);
	rotate(pub, loop_rate, -90);
	rotate(pub, loop_rate, -45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 50);
	ros::Duration(2.0).sleep();
	moveLinear(loop_rate, 0.1);
	ros::Duration(0.5).sleep();
	system("/home/robot/桌面/fmodstudioapi10509linux/api/lowlevel/examples/make/play_sound");
//jobs over
	rotate(pub, loop_rate, 55);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 50);
	moveLinear(loop_rate, 2.0);
// out from jobs room
	rotate(pub, loop_rate, -80);
	moveLinear(loop_rate, 1.6);
	rotate(pub, loop_rate, -60);
	moveLinear(loop_rate, 1.4);
	ros::Duration(2.0).sleep();
	rotate(pub, loop_rate, 20);
	moveLinear(loop_rate, 0.2);
	moveLinear(loop_rate, -0.2);
	rotate(pub, loop_rate, -20);
	moveLinear(loop_rate, -1.4);
	rotate(pub, loop_rate, -80);
	rotate(pub, loop_rate, -30);
	moveLinear(loop_rate, 0.9);
	rotate(pub, loop_rate, -80);
	//two rooms finished
	moveLinear(loop_rate, 1.8);
	rotate(pub, loop_rate, -90);
	moveLinear(loop_rate, 1.6);
	rotate(pub, loop_rate, -45);
	rotate(pub, loop_rate, -45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(2.0).sleep();
	moveLinear(loop_rate, 0.2);
	ros::Duration(1.0).sleep();
	rotate(pub, loop_rate, 90);
	rotate(pub, loop_rate, 90);
	//room 3 finished
	moveLinear(loop_rate, 2.6);
	ros::Duration(2.0).sleep();
	moveLinear(loop_rate, 0.5);
	rotate(pub, loop_rate, 45);
	ros::Duration(2.0).sleep();
	rotate(pub, loop_rate, 45);
	ros::Duration(2.0).sleep();
	//rotate(pub, loop_rate, -15);
	//rotate(pub, loop_rate, -15);
	moveLinear(loop_rate, 0.30);
	ros::Duration(1.0).sleep();
	moveLinear(loop_rate, -0.4);
	//rotate(pub, loop_rate, 15);
	//rotate(pub, loop_rate, 15);
	rotate(pub, loop_rate, 90);
	moveLinear(loop_rate, 1.5);
	//room 4 finished
	rotate(pub, loop_rate, -90);
	moveLinear(loop_rate, 2.0);
	rotate(pub, loop_rate, -20);
	moveLinear(loop_rate, 2.3);
#endif
	return 0;
}
