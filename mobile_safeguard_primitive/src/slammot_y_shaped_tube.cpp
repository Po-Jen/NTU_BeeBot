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

// PCL Voxel Grid
#include <pcl/filters/voxel_grid.h>

////////////////////////////////////////////////////////////////////////////////////////
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
////////////////////////////////////////////////////////////////////////////////////////
// camera/rgb/image_raw
// ******************************
ros::Publisher rtabmap_rgbRaw_pub ;
ros::Publisher spencer_rgbRaw_pub ;
//ros::Publisher pplfilt_rgbRaw_pub ;

// camera/rgb/image_rect_color
// ******************************
//ros::Publisher rtabmap_rgbRect_pub ;
ros::Publisher spencer_rgbRect_pub ;
//ros::Publisher pplfilt_rgbRect_pub ;

// camera/rgb/camera_info
// ******************************
ros::Publisher rtabmap_rgbCamInfo_pub ;
ros::Publisher spencer_rgbCamInfo_pub ;
//ros::Publisher pplfilt_rgbCamInfo_pub ;

// camera/depth/image_rect
// ******************************
ros::Publisher rtabmap_depImg_pub ;
ros::Publisher spencer_depImg_pub ;
//ros::Publisher pplfilt_depImg_pub ;

// camera/depth_registered/points
// *******************************
//ros::Publisher rtabmap_pts_pub;
ros::Publisher spencer_pts_pub;
ros::Publisher pplfilt_pts_pub;

//////////////////////////////////////////////////////////////////////////////////////////////////

void 
rgbRaw_cb (const sensor_msgs::ImageConstPtr& rgbRaw)
{
  rtabmap_rgbRaw_pub.publish(rgbRaw) ;
  spencer_rgbRaw_pub.publish(rgbRaw) ;
 // pplfilt_rgbRaw_pub.publish(rgbRaw) ;
}

void
rgbRec_cb (const sensor_msgs::ImageConstPtr& rgbRec){
  //rtabmap_rgbRect_pub.publish(rgbRec) ;
  spencer_rgbRect_pub.publish(rgbRec) ;
  //pplfilt_rgbRect_pub.publish(rgbRec) ;
}

void
rgbCamInfo_cb (const sensor_msgs::CameraInfoConstPtr& rgbCamInfo)
{
   rtabmap_rgbCamInfo_pub.publish (rgbCamInfo) ;
   spencer_rgbCamInfo_pub.publish (rgbCamInfo) ;
   //pplfilt_rgbCamInfo_pub.publish (rgbCamInfo) ;
}

void
depImg_cb (const sensor_msgs::ImageConstPtr& depImg)
{
   rtabmap_depImg_pub.publish (depImg) ;
   spencer_depImg_pub.publish (depImg) ;
   //pplfilt_depImg_pub.publish (depImg) ;
}

void
pts_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  
   //====================================
   // Downsample
   // ===================================
    float leafSize = 0.10 ;
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize ( leafSize, leafSize, leafSize );
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
 
    // ===================================
    // Publish Outputs
    // =================================== 
   //rtabmap_pts_pub.publish ( output ) ;
   spencer_pts_pub.publish ( output ) ;
   pplfilt_pts_pub.publish ( output ) ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "slammot_y_shaped_tube");
  ros::NodeHandle nh;

   // for the subscribing topics
#if 1
   std::string rgbRaw_topic = "camera/data_throttled_image_relay";
   std::string rgbRec_topic = "camera/rgb/image_rect_color";
   std::string rgbCamInfo_topic = "camera/data_throttled_camera_info_relay";
   std::string depImg_topic = "camera/data_throttled_image_depth_relay";
   std::string pts_topic = "camera/depth_registered/points" ;
# endif
  
  //=====================================================
  // Create a ROS subscriber for the input point cloud
  //=====================================================
  ros::Subscriber rgbRaw_sub = nh.subscribe (rgbRaw_topic, 1, rgbRaw_cb);
  ros::Subscriber rgbRec_sub = nh.subscribe (rgbRec_topic, 1, rgbRec_cb);
  ros::Subscriber rgbCamInfo_sub = nh.subscribe (rgbCamInfo_topic, 1, rgbCamInfo_cb);
  ros::Subscriber depImg_sub = nh.subscribe (depImg_topic, 1, depImg_cb);
  ros::Subscriber pts_sub = nh.subscribe (pts_topic, 1, pts_cb) ;


  //=========================================================
  // Create a ROS publisher for the output point cloud
  //=========================================================

   // RTABMAP
   // **************
   rtabmap_rgbRaw_pub = nh.advertise<sensor_msgs::Image> ("camera/data_throttled_relay", 1) ;
   //rtabmap_rgbRect_pub = nh.advertise<sensor_msgs::Image> ("", 1) ;
   rtabmap_rgbCamInfo_pub = nh.advertise<sensor_msgs::CameraInfo> ("camera/data_throttled_camera_info", 1) ;
   rtabmap_depImg_pub = nh.advertise<sensor_msgs::Image> ("camera/data_throttled_image_depth_relay", 1) ;
   //rtabmap_pts_pub = nh.advertise<sensor_msgs::PointCloud2> ("", 1) ;

  // SPENCER PEOPLE TRACKING
  // *************************
  spencer_rgbRaw_pub = nh.advertise<sensor_msgs::Image> ("spencer/sensors/rgbd_front_top/rgb/image_raw", 1) ;  
  // rwth_upper_body/_detector uses /rgb/image_raw, REMEMBER TO REMAP
  spencer_rgbRect_pub = nh.advertise<sensor_msgs::Image> ("spencer/sensors/rgbd_front_top/rgb/image_rect_color", 1) ;
  spencer_rgbCamInfo_pub = nh.advertise<sensor_msgs::CameraInfo> ("spencer/sensors/rgbd_front_top/rgb/camera_info", 1) ;
  spencer_depImg_pub = nh.advertise<sensor_msgs::Image> ("spencer/sensors/rgbd_front_top/depth/image_rect", 1) ;
  spencer_pts_pub = nh.advertise<sensor_msgs::PointCloud2> ("spencer/sensors/rgbd_front_top/depth_registered/points", 1) ;

  // PEOPLE FILTER
  // ****************
  /*
  pplfilt_rgbRaw_pub = nh.advertise<sensor_msgs::Image> ("rgbRaw_in", 1) ;
  pplfilt_rgbRect_pub = nh.advertise<sensor_msgs::Image> ("rgbRec_in", 1) ;
  pplfilt_rgbCamInfo_pub = nh.advertise<sensor_msgs::CameraInfo> ("rgbCamInfo_in", 1) ;
  pplfilt_depImg_pub = nh.advertise<sensor_msgs::Image> ("depImg_in", 1) ;
  */
  pplfilt_pts_pub = nh.advertise<sensor_msgs::PointCloud2> ("raw_cloud", 1) ;

  // Spin
  ros::spin ();
}

