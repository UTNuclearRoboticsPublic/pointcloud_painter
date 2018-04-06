
#include <ros/ros.h>
#include "pointcloud_painter/pointcloud_painter.h"
#include <ros/callback_queue.h>

bool found_pointcloud, found_front_image, found_rear_image;
sensor_msgs::Image front_image, rear_image;
sensor_msgs::PointCloud2 pointcloud;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
	if(!found_pointcloud)
	{
		pointcloud = *input_cloud;
		found_pointcloud = true;
	}
}
void front_image_callback(const sensor_msgs::Image::ConstPtr& input_image)
{
	if(!found_front_image)
	{
		front_image = *input_image;
		found_front_image = true;
	}
}
void rear_image_callback(const sensor_msgs::Image::ConstPtr& input_image)
{
	if(!found_rear_image)
	{
		rear_image = *input_image;
		found_rear_image = true;
	}
}

int main(int argc, char** argv)
{
	// ---------------------------------------------------------------------------
	// ----------------------------- Basic ROS Stuff -----------------------------
	// ---------------------------------------------------------------------------

	ros::init(argc, argv, "active_painter_demo");

	ros::NodeHandle nh;
/*
	std::string service_name;
	nh.param<std::string>("/pointcloud_painter/service_name", service_name, "/pointcloud_painter/paint");
	ros::ServiceClient painter_srv = nh.serviceClient<pointcloud_painter::pointcloud_painter_srv>(service_name);

	// Maximum angle viewable through the lens 
	float max_lens_angle;
	nh.param<float>("/pointcloud_painter/max_lens_angle", max_lens_angle, 235);
	// Type of projection (see pointcloud_painter/pointcloud_painter_srv)
	int projection_type;
	nh.param<int>("/pointcloud_painter/projection_type", projection_type, 2);
	int neighbor_search_count;
	nh.param<int>("/pointcloud_painter/neighbor_search_count", neighbor_search_count, 3);
	// Should this process loop? 
	bool should_loop;
	nh.param<bool>("/pointcloud_painter/should_loop", should_loop);
	
	// Compression
	bool compress_image;
	nh.param<bool>("/pointcloud_painter/compress_image", compress_image, true);
	int image_compression_ratio;
	nh.param<int>("/pointcloud_painter/image_compression_ratio", image_compression_ratio, 8);
	// Voxel sizes for image postprocessing
	bool voxelize_rgb_images;
	float flat_voxel_size, spherical_voxel_size;
	nh.param<bool>("/pointcloud_painter/voxelize_rgb_images", voxelize_rgb_images, true);
	nh.param<float>("/pointcloud_painter/flat_voxel_size", flat_voxel_size, 0.005);
	nh.param<float>("/pointcloud_painter/spherical_voxel_size", spherical_voxel_size, 0.005);
	bool voxelize_depth_cloud;
	float depth_voxel_size;
	nh.param<bool>("/pointcloud_painter/voxelize_depth_cloud", voxelize_depth_cloud, true);
	nh.param<float>("/pointcloud_painter/depth_voxel_size", depth_voxel_size, 0.05);
	
	std::string camera_frame_front, camera_frame_rear, target_frame;
	nh.param<std::string>("/pointcloud_painter/camera_frame_front", camera_frame_front, "front_camera_frame");
	nh.param<std::string>("/pointcloud_painter/camera_frame_rear", camera_frame_rear, "rear_camera_frame");
	nh.param<std::string>("/pointcloud_painter/target_frame", target_frame, "target_frame");

	// ---------------------------------------------------------------------------
	// ------------------------ Extract Data from ROSBAGs ------------------------
	// ---------------------------------------------------------------------------

	// Input data topics
	std::string pointcloud_topic = 		"/laser_stitcher/local_dense_cloud";
	std::string front_image_topic = 	"front_camera/image_raw";
	std::string rear_image_topic = 		"rear_camera/image_raw";

	found_pointcloud = false;
	found_front_image = false;
	found_rear_image = false;

	ros::NodeHandle sub_handle;
	ros::CallbackQueue input_callback_queue;
	sub_handle.setCallbackQueue(&input_callback_queue);

	ros::Subscriber pointcloud_sub = sub_handle.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1, pointcloud_callback);
	while(ros::ok() && !found_pointcloud)
	{
		input_callback_queue.callAvailable(ros::WallDuration());
		ros::Duration(0.1).sleep();
		ROS_INFO_STREAM("waiting for pointcloud input message...");
	}
	ros::Subscriber front_image_sub = sub_handle.subscribe<sensor_msgs::Image>(front_image_topic, 1, front_image_callback);
	while(ros::ok() && !found_front_image)
	{
		input_callback_queue.callAvailable(ros::WallDuration());
		ros::Duration(0.1).sleep();
		ROS_INFO_STREAM("waiting for front image input message...");
	}
	ros::Subscriber rear_image_sub = sub_handle.subscribe<sensor_msgs::Image>(rear_image_topic, 1, rear_image_callback);
	while(ros::ok() && !found_rear_image)
	{
		input_callback_queue.callAvailable(ros::WallDuration());
		ros::Duration(0.1).sleep();
		ROS_INFO_STREAM("waiting for rear image input message...");
	}

	// -------- Set up service object --------
	pointcloud_painter::pointcloud_painter_srv srv;
	// -------- Data --------
	srv.request.input_cloud = pointcloud;
	srv.request.input_cloud.header.stamp = ros::Time::now();
	srv.request.image_front = front_image;
	srv.request.image_rear = rear_image;
	// -------- Projection Stuff --------
	srv.request.projection = projection_type;
	srv.request.max_angle = max_lens_angle;
	srv.request.neighbor_search_count = neighbor_search_count;
	// -------- Compression --------
	// Raster-space Compression
	srv.request.compress_image = compress_image;
	srv.request.image_compression_ratio = image_compression_ratio;
	// Pointcloud-space Compression
	srv.request.voxelize_rgb_images = voxelize_rgb_images;
	srv.request.flat_voxel_size = flat_voxel_size;
	srv.request.spherical_voxel_size = spherical_voxel_size;
	srv.request.voxelize_depth_cloud = voxelize_depth_cloud;
	srv.request.depth_voxel_size = depth_voxel_size;
	// -------- Frames --------
	srv.request.camera_frame_front = camera_frame_front;
	srv.request.camera_frame_rear = camera_frame_rear;
	srv.request.target_frame = target_frame;

	ROS_ERROR_STREAM("first: " << front_image.header.frame_id << " " << rear_image.header.frame_id);

	ros::Duration(1.0).sleep();

	// Run Service
	while(ros::ok())
	{

		// Wait a moment to ensure that the service is up...
		ros::Duration(1.0).sleep();

		// Call service
		if( ! painter_srv.call(srv) )
			ROS_WARN_STREAM("[PointcloudPainter] Painting service call failed - prob not up yet");
		else
		{	
			ROS_INFO_STREAM("[PointcloudPainter] Successfully called painting service.");
			ROS_INFO_STREAM("[PointcloudPainter]   Cloud Size: " << srv.response.output_cloud.height*srv.response.output_cloud.width);
			ros::Duration(0.5).sleep();
		}

		// If we shouldn't loop, break the loop
		if(!should_loop)
			break;
	}	
	*/
}	