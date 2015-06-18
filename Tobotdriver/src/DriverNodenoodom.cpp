#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/Twist.h>
#include "Scribe.hpp"
#include "Poete.hpp"
#include "Robot.hpp"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include "Lifting_service.hpp"

void publishTransform(const nav_msgs::Odometry& odomRead, tf::TransformBroadcaster& odom_broadcaster){
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = odomRead.header.stamp;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = odomRead.pose.pose.position.x;
	odom_trans.transform.translation.y = odomRead.pose.pose.position.y;
	odom_trans.transform.translation.z = odomRead.pose.pose.position.z;
	odom_trans.transform.rotation = odomRead.pose.pose.orientation;
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
}


int main(int argc, char **argv)
{
	//ROS declaration
	ros::init(argc, argv, "TobotDriver");
	ros::NodeHandle my_node;
	ros::Rate loop_rate(10);

	ros::NodeHandle priv_node("~");
	std::cout << "init Robot"<<std::endl;
	Robot platform(priv_node);
	
	std::cout << "init Scrible"<<std::endl;
	Scribe scribe(my_node, "/cmd_vel", platform);
	
	std::cout << "init Poete"<<std::endl;
	Poete poete(my_node, "/odom");
	
	std::cout << "init Lift"<<std::endl;
	Lifter lift(my_node, priv_node);
	
	std::cout << "init tf"<<std::endl;
	tf::TransformBroadcaster odom_broadcaster;
	
	int flag=0;
	//Creation de Platform et test de verbose option
	if ( argc > 1 ) {// argc should be 2 for correct execution
     		for(int i =0;i<argc;i++&&flag==0){
     			if (strcmp(argv[i],"--verbose")&&flag==0){
     				std::cout<<"VERBOSE MODE ON"<<std::endl;
     				scribe.setVerbose();
     				poete.setVerbose();
     				platform.setVerbose();
     				flag=1;
     			}
     		}	
	}
	
	while(ros::ok()){
		//platform.odometry();
		//if we read correctly
		if(platform.getControl().getReadState()){
			//poete.publish(platform.getOdom());
			//publishTransform(platform.getOdom(), odom_broadcaster);
		}
		ros::spinOnce();
		loop_rate.sleep();
	
	}
}
