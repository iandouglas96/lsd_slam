/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "IOWrapper/InputImageStream.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "util/Undistorter.h"
#include "GlobalMapping/g2oTypeSE3Sophus.h"


namespace lsd_slam
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
													sensor_msgs::Image,
													sensor_msgs::Image,
													sensor_msgs::Image> SynchPolicy;

/**
 * Image stream provider using ROS messages.
 */
class ROSImageStreamThread : public InputImageStream
{
public:
	ROSImageStreamThread();
	~ROSImageStreamThread();
	
	/**
	 * Starts the thread.
	 */
	void run();
	
	void setCalibration(std::string file);

	/**
	 * Thread main function.
	 */
	void operator()();

	//Get depth (in meters) for a particular pixel at x,y
    float getDepth(int x, int y, int cam);

	SE3NoX getTransformFromTunnel(int cam);
	SE3 getTransformBetweenCameras(int from, int to);
	float getRadius();

	bool depthReady();
	
	// get called on ros-message callbacks
	void vidCb(const sensor_msgs::ImageConstPtr top_left_img/*,
			   const sensor_msgs::ImageConstPtr top_right_img, 
			   const sensor_msgs::ImageConstPtr bottom_left_img, 
			   const sensor_msgs::ImageConstPtr bottom_right_img*/);
	void infoCb(const sensor_msgs::CameraInfoConstPtr info);
	void radiusCb(const std_msgs::Float64::ConstPtr& msg);
	void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr msg);


private:
	//Image stream stuff
	bool haveCalib, haveDepthMap;
	Undistorter* undistorter[NUM_CAMERAS];

	ros::NodeHandle nh_;

	//Must maintain pointers or c will delete the subscription immediately.
	message_filters::Subscriber<sensor_msgs::Image> *top_left_sub;
	message_filters::Subscriber<sensor_msgs::Image> *top_right_sub;
	message_filters::Subscriber<sensor_msgs::Image> *bottom_left_sub;
	message_filters::Subscriber<sensor_msgs::Image> *bottom_right_sub;
	message_filters::Synchronizer<SynchPolicy> *image_sub;

	int lastSEQ;

	//Depth calc stuff
	int old_width_, old_height_;
	Eigen::Matrix3f cam_intrinsics;

	tf2_ros::Buffer *tf_buffer;
    tf2_ros::TransformListener *tf_listener;

    Eigen::Affine3f cam_pose[NUM_CAMERAS];

	ros::Subscriber radius_sub;
    float tunnel_radius;

	Eigen::Vector3f focal_plane_dir[NUM_CAMERAS];

    Eigen::Vector3f calcProjectionCameraFrame(int x, int y);
    float calcDistance(Eigen::Vector3f &ray_direction, int cam);
	
	//Point Cloud processing
	ros::Subscriber pointcloud_sub;
	Eigen::Affine3f sensor_pose;
	bool have_sensor_pose;
	cv::Mat depth_map;
	bool use_tunnel_estimator; //Toggle to use raw pointcloud or data from tunnel_estimator
};

}
