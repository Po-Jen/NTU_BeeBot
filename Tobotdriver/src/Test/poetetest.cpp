#include <iostream>
#define BOOST_TEST_DYN_LINK
#include <time.h>
#include <cstdlib>
#include "Poete.hpp"
#include "Robot.hpp"
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>
/*********************

*************************/

BOOST_AUTO_TEST_CASE(trying)
{
	

	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "timer");
	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);
	Poete p(true, n, "/odom");
	
	nav_msgs::Odometry o;
	
	int flag=1;
	
	std_msgs::Time tim;
	tim.data=ros::Time::now();
	ros::Publisher montre = n.advertise<std_msgs::Time>("time", 1000);
	
	
	//Publish only work inside his loop
	while (ros::ok())
  		{
  			//ros::spinOnce(); //PUSH ONLY ONE THING AT A TIME !!!!
  			if(flag==1){
  				//ros::spinOnce();
  				loop_rate.sleep();
  				p.publish(o);
				//ros::spinOnce();
				flag=3;
				std::cout << "HELLOW"<<std::endl;
				montre.publish(tim);

				
			}
			//montre.publish(tim);
			//ros::spinOnce();
  			//loop_rate.sleep();

			

		}
	
}
