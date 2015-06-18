#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/Twist.h>

#include <sstream>

/**
 * This node take a command vel and publish the time. It is the premises of the pid driver.
 */
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher montre, std_msgs::Duration& tim)
{
  //Wat te doen ?
  	montre.publish(tim);
  	std::cout << "YO"<<std::endl;
  	std::cout<< tim.data<<std::endl;
}
 
 
 
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform any ROS arguments and name remapping that were provided at the command line. For programmatic remappings you can use a different version of init() which takes remappings directly, but for most command-line programs, passing argc and argv is the easiest way to do it.  The third argument to init() is the name of the node. You must call one of the versions of ros::init() before using any other part of the ROS system.
   */
  	ros::init(argc, argv, "timer");

  /**
   * NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
   */
  	ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to publish on a given topic name. This invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing. After this advertise() call is made, the master node will notify anyone who is trying to subscribe to this topic name, and they will in turn negotiate a peer-to-peer connection with this node.  advertise() returns a Publisher object which allows you to publish messages on that topic through a call to publish().  Once all copies of the returned Publisher object are destroyed, the topic will be automatically unadvertised.
   * The second parameter to advertise() is the size of the message queue used for publishing messages.  If messages are published more quickly than we can send them, the number here specifies how many messages to buffer up before throwing some away.
   */
   	ros::Time msg;
   	std_msgs::Duration tim;
   
   	int i=1;
	ros::Publisher montre = n.advertise<std_msgs::Duration>("time", 1000);
	//bind is used to send the argument to the call back.
	ros::Subscriber order = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&chatterCallback, _1, montre, tim) );


  	ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create a unique string for each message.
   */
  	std::cout << "GOOOO"<<std::endl;
   
  	ros::Time count = ros::Time::now();
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
	msg=ros::Time::now();
	//std::cout<< msg-count<<std::endl;
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type given as a template parameter to the advertise<>() call, as was done in the constructor above.
     */   
	tim.data = msg-count;
     //std::cout<< tim.data<<std::endl;
    
    // need to actualise tim every turn so we call it here
	order = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&chatterCallback, _1, montre, tim) );
    	ros::spinOnce();
    	loop_rate.sleep();
  }


  return 0;
}
