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
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "util/Undistorter.h"
#include "GlobalMapping/g2oTypeSE3Sophus.h"


namespace lsd_slam
{



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
    float getDepth(int x, int y);

	SE3NoX getTransform();
	float getRadius();

	bool depthReady();
	
	// get called on ros-message callbacks
	void vidCb(const sensor_msgs::ImageConstPtr img);
	void infoCb(const sensor_msgs::CameraInfoConstPtr info);
	void radiusCb(const std_msgs::Float64::ConstPtr& msg);
	void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr msg);


private:
	//Image stream stuff
	bool haveCalib, haveDepthMap;
	Undistorter* undistorter;

	ros::NodeHandle nh_;

	std::string vid_channel;
	ros::Subscriber vid_sub;

	int lastSEQ;

	//Depth calc stuff
	int old_width_, old_height_;
	cv::Matx33d cam_intrinsics;

	tf2_ros::Buffer *tf_buffer;
    tf2_ros::TransformListener *tf_listener;

    tf2::Stamped<tf2::Transform> top_left_transform;

	ros::Subscriber radius_sub;
    float tunnel_radius;

	Eigen::Vector3d focal_plane_dir;

    cv::Point3d calcProjectionCameraFrame(int x, int y);
    void unitVectorToPose(const std::string& frame, cv::Point3f vec, tf2::Stamped<tf2::Transform>& trans);
    float calcDistance(tf2::Stamped<tf2::Transform>& vec, tf2::Stamped<tf2::Transform>& transform);
	
	//Point Cloud processing
	ros::Subscriber pointcloud_sub;
	Eigen::Affine3f sensor_pose;
	bool have_sensor_pose;
	cv::Mat depth_map;
	bool use_tunnel_estimator; //Toggle to use raw pointcloud or data from tunnel_estimator
};

}
