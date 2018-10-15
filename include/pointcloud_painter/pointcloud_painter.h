
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pointcloud_painter/pointcloud_painter_srv.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#define PAINTER_PROJ_EQUA_STEREO 	1
#define PAINTER_PROJ_POLE_STEREO 	2
#define PAINTER_PROJ_EQUAL_AREA 	3
#define PAINTER_PROJ_FLAT 			4

class PointcloudPainter
{
public:
	PointcloudPainter();
	bool buildImageClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_flat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_spherical_lobed, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_spherical, cv_bridge::CvImagePtr cv_image, std::string camera_frame, std::string target_frame, int projection, float max_angle, int image_hgt, int image_wdt, int image_number);
	bool paintPointcloud(pointcloud_painter::pointcloud_painter_srv::Request &req, pointcloud_painter::pointcloud_painter_srv::Response &res);
	bool projectColorOntoDepth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr spherical_depth_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &depth_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgb_cloud, int ver_res, int hor_res, int k);
	bool projectDepthOntoColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr spherical_depth_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &depth_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgb_cloud, int ver_res, int hor_res, int k);
	bool interpolateColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, pcl::PointCloud<pcl::PointXYZ> &depth_cloud, pcl::PointCloud<pcl::PointXYZRGB> &rgb_cloud, int ver_res, int hor_res);
	bool downsampleImage(cv_bridge::CvImagePtr image_out, cv_bridge::CvImagePtr image_in, int height, int width, int height_mult, int width_mult);

private:
	ros::NodeHandle nh_;
	tf::TransformListener camera_frame_listener_;

};