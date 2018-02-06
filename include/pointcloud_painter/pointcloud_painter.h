
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


class PointcloudPainter
{
public:
	PointcloudPainter();
	bool build_image_clouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_flat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_spherical, cv_bridge::CvImagePtr cv_image, std::string camera_frame, std::string target_frame, int projection, float max_angle, int image_hgt, int image_wdt, bool flip_cloud);
	bool paint_pointcloud(pointcloud_painter::pointcloud_painter_srv::Request &req, pointcloud_painter::pointcloud_painter_srv::Response &res);

private:
	ros::NodeHandle nh_;
	tf::TransformListener camera_frame_listener_;

};