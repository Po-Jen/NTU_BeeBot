/*
* slammot_y_shaped_tube.cpp
* Charly Huang Aug 29, Friday, 0:21
* This node receives RGB-D sensor's topics and publish the same data
* with different topic names for various namespaces, i.e. RTABMap, Spencer
* and people_filter.
*/

#include <ros/ros.h>
#include <iostream>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;
ros::Publisher rtabmap_rgbImg_pub ;
ros::Publisher spencer_rgbImg_pub ;
ros::Publisher pplfilt_rgbImg_pub ;

ros::Publisher rtabmap_rgbCamInfo_pub ;
ros::Publisher spencer_rgbCamInfo_pub ;
ros::Publisher pplfilt_rgbCamInfo_pub ;

ros::Publisher rtabmap_depImg_pub ;
ros::Publisher spencer_depImg_pub ;
ros::Publisher pplfilt_depImg_pub ;

void 
rgbImg_cb (const sensor_msgs::PointCloud2ConstPtr& rgbImg)
{
  rtabmap_rgbImg_pub.publish(rgbImg) ;
  spencer_rgbImg_pub.publish(rgbImg) ;
  pplfilt_rgbImg_pub.publish(rgbImg) ;
}

void
rgbCamInfo_cb (const sensor_msgs::CameraInfoConstPtr& rgbCamInfo)
{
   rtabmap_rgbCamInfo_pub.publish (rgbCamInfo) ;
   spencer_rgbCamInfo_pub.publish (rgbCamInfo) ;
   pplfilt_rgbCamInfo_pub.publish (rgbCamInfo) ;
}

void
depImg_cb (const sensor_msgs::PointCloud2ConstPtr& depImg)
{
   rtabmap_depImg_pub.publish (depImg) ;
   spencer_depImg_pub.publish (depImg) ;
   pplfilt_depImg_pub.publish (depImg) ;
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "slammot_y_shaped_tube");
  ros::NodeHandle nh;

   // for the subscribing topics
   std::string rgbImg_topic = "camera/data_throttled_image_relay";
   std::string rgbCamInfo_topic = "camera_data_throttled_camera_info_relay";
   std::string depImg_topic = "camera/data_throttled_image_depth_relay";

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber rgbImg_sub = nh.subscribe (rgbImg_topic, 1, rgbImg_cb);
  ros::Subscriber rgbCamInfo_sub = nh.subscribe (rgbCamInfo_topic, 1, rgbCamInfo_cb);
  ros::Subscriber depImg_sub = nh.subscribe (depImg_topic, 1, depImg_cb);


  // Create a ROS publisher for the output point cloud
  //rtabmap_rgbImg_pub = nh.advertise<sensor_msgs::Image> ("", 1) ;
  spencer_rgbImg_pub = nh.advertise<sensor_msgs::Image> ("spencer/sensor/front_top/rgb/image_rect_color", 1) ;  // rwth_upper_body/_detector uses /rgb/image_raw, REMEMBER TO REMAP
  pplfilt_rgbImg_pub = nh.advertise<sensor_msgs::Image> ("rgbImg_in", 1) ;

   //rtabmap_rgbCamInfo_pub = nh.advertise<sensor_msgs::CameraInfo> ("", 1) ;
   spencer_rgbCamInfo_pub = nh.advertise<sensor_msgs::CameraInfo> ("spencer/sensor/front_top/rgb/camera_info", 1) ;
   pplfilt_rgbCamInfo_pub = nh.advertise<sensor_msgs::CameraInfo> ("rgbCamInfo_in", 1) ;

   //rtabmap_depImg_pub = nh.advertise<senosr_msgs::Image> ("", 1) ;
   spencer_depImg_pub = nh.advertise<sensor_msgs::Image> ("spencer/sensor/depth/image_rect_color", 1) ;
   pplfilt_depImg_pub = nh.advertise<sensor_msgs::Image> ("depImg_in", 1) ;

  // Spin
  ros::spin ();
}

