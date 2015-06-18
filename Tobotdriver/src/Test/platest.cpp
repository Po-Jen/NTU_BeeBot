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
	Robot p(n);
	
	p.setRadius(10);
	p.setWheelRadius(4);
	p.setSpeed(2);
	p.setAngularSpeed(3);
	
	BOOST_CHECK_EQUAL(10,p.getRadius());
	BOOST_CHECK_EQUAL(4,p.getWheelRadius());
	BOOST_CHECK_EQUAL(2,p.getSpeed());
	BOOST_CHECK_EQUAL(3,p.getAngularSpeed());
	
	p.robot2wheels();
	
	BOOST_CHECK_EQUAL(p.getControl().getTargetSRW(),20);
	BOOST_CHECK_EQUAL(p.getControl().getTargetSLW(),-15);
	
	p.setWheelRadius(1);
	BOOST_CHECK_EQUAL(19,p.ms2rpm(1));
	int i;
	scanf("%d",&i);
}
