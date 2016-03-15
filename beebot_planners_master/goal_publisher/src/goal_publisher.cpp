#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"goal_pub");
  
  ros::NodeHandle n;
  
  float goal_x = 0.;
  float goal_y = 5.0;
  
  n.getParam ("goal_x",  goal_x) ;
  n.getParam ("goal_x",  goal_y) ;

  ros::Publisher goal_pub;
  std::string goal_topic_name = "/move_base_simple/goal";
  goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic_name.c_str(), 1);
    
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
//  goal.pose.position.x = 0.;
//  goal.pose.position.y = 5.0;

  goal.pose.position.x = goal_x ;
  goal.pose.position.y = goal_y ;

  goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  ros::Rate r(1);
  r.sleep();
  for(int i=0;i<100;i++)
    goal_pub.publish(goal);

    ROS_INFO_ONCE("Goal was published");

  ros::spin();
  return 0;
}
