/*
* pre-processing.cpp
* Charly Huang  Aug 29, 2015. 03:25 am
* 
* This node receives incoming Point Cloud 2 and do basic pre-processings. 
*/


//Standard ROS Headers
//#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

//PCL basic headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>

//PCL filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

//PCL Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

//PCL other headers
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include "pcl_ros/transforms.h"

//////////////////////////////////////////////////////////////////////////////////////////////////

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher pub;

void
preProcess (PointCloudT::Ptr cloud_msg, PointCloudT::Ptr cloud_filt){

    PointCloudT::Ptr filtered_cloud (new PointCloudT) ;
    
   // =========================================
   //  Remove NaNs
   // =========================================
   std::vector<int> mapping ;
   pcl::removeNaNFromPointCloud(*cloud_msg, *filtered_cloud, mapping);

   // =========================================
   // Remove Statistical Outlier
   // =========================================
   pcl::StatisticalOutlierRemoval<PointT> rmOutlr;
   rmOutlr.setInputCloud ( filtered_cloud ) ;
   // Set number of neighbors to consider to 50.
   rmOutlr.setMeanK(50);
   // Set standard deviation multiplier to 1.
   // Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
   rmOutlr.setStddevMulThresh(1.0);
 
   rmOutlr.filter(*cloud_filt);     
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a containers for the data.
  sensor_msgs::PointCloud2 output;
  PointCloudT::Ptr cloud_in (new PointCloudT) ;
  PointCloudT::Ptr cloud_filtered (new PointCloudT) ;

  // ========================================
  //  Convert from ROS to PCL data type
  // ========================================
  pcl::fromROSMsg (*input, *cloud_in) ;

  // Do data processing here...
  preProcess (cloud_in, cloud_filtered) ;

  // ========================================
  //  Convert from PCL to ROS data type
  // ========================================
  pcl::toROSMsg (*cloud_filtered, output) ;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pre-processing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
