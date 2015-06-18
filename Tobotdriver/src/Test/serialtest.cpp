#include <iostream>
#define BOOST_TEST_DYN_LINK
#include <time.h>
#include <cstdlib>
#include "SerialPortControl_stream.hpp"
#include <SerialStream.h>
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>
/*********************

*************************/
#define BOU 'V'

BOOST_AUTO_TEST_CASE(trying)
{
	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "timer");
	ros::NodeHandle n("~");
	std::string yo="yo";
	BOOST_CHECK_EQUAL(BOU+yo, "Vyo");
	SerialPortControl sp(7500, n);
	//sp.setVerbose();
	sp.writeHome();
	sp.writePoseRelativeR(2000);
	sp.writeMoveRightWheel();
}
