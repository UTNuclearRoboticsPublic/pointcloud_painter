
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pointcloud_painter/pointcloud_painter_srv.h"


class PointcloudPainter
{
public:
	PointcloudPainter();
	bool paint_pointcloud(pointcloud_painter::pointcloud_painter_srv::Request &req, pointcloud_painter::pointcloud_painter_srv::Response &res);

private:

};