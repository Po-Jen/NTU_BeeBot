#include <iostream>
#define BOOST_TEST_DYN_LINK
#include <time.h>
#include <cstdlib>
#include "Scribe.hpp"
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>
/*********************

*************************/

BOOST_AUTO_TEST_CASE(trying)
{
	

	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "timer");
	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);
	Robot r(n);
	Scribe s(n, "/cmd_vel", r);
	
	geometry_msgs::Twist cmd;
	cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x =cmd.angular.y=cmd.angular.z = 0;   
	
	BOOST_CHECK_EQUAL(cmd.linear.x, s.getCmd().linear.x);
	BOOST_CHECK_EQUAL(cmd.linear.y, s.getCmd().linear.y);
	BOOST_CHECK_EQUAL(cmd.linear.z, s.getCmd().linear.z);
	BOOST_CHECK_EQUAL(cmd.angular.x, s.getCmd().angular.x);
	BOOST_CHECK_EQUAL(cmd.angular.y, s.getCmd().angular.y);
	BOOST_CHECK_EQUAL(cmd.angular.z, s.getCmd().angular.z);
	
	cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x =cmd.angular.y=cmd.angular.z = 5;
	
	s.setCmd(cmd);
	
	BOOST_CHECK_EQUAL(cmd.linear.x, s.getCmd().linear.x);
	BOOST_CHECK_EQUAL(cmd.linear.y, s.getCmd().linear.y);
	BOOST_CHECK_EQUAL(cmd.linear.z, s.getCmd().linear.z);
	BOOST_CHECK_EQUAL(cmd.angular.x, s.getCmd().angular.x);
	BOOST_CHECK_EQUAL(cmd.angular.y, s.getCmd().angular.y);
	BOOST_CHECK_EQUAL(cmd.angular.z, s.getCmd().angular.z);
	


}
