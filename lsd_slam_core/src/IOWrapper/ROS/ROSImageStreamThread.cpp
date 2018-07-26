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

#include "ROSImageStreamThread.h"
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#undef USE_ROS //Make PCL compilable
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>

#include <iostream>
#include <fstream>


namespace lsd_slam
{


using namespace cv;

ROSImageStreamThread::ROSImageStreamThread()
{
	//load params
	nh_.param("/lsd_slam/use_tunnel_estimator", use_tunnel_estimator, false);

	ROS_INFO("Using tunnel estimator: %d", use_tunnel_estimator);

	// wait for cam calib
	width_ = height_ = 0;
	have_sensor_pose = false;

	// subscribe
	vid_channel = nh_.resolveName("image");
	vid_sub     = nh_.subscribe(vid_channel,1, &ROSImageStreamThread::vidCb, this);
	pointcloud_sub = nh_.subscribe(nh_.resolveName("pointcloud"), 1, &ROSImageStreamThread::pointCloudCb, this);
	std::string radius_channel = nh_.resolveName("local_tunnel_radius");
	radius_sub = nh_.subscribe(radius_channel,1, &ROSImageStreamThread::radiusCb, this);

    // tf2 listener and buffer
    this->tf_buffer = new tf2_ros::Buffer();
    this->tf_listener = new tf2_ros::TransformListener(*(this->tf_buffer));

    tunnel_radius = -1;

	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMat>(8);
	undistorter = 0;
	lastSEQ = 0;

	haveCalib = false;
	haveDepthMap = false;
}

bool ROSImageStreamThread::depthReady()
{
	return haveDepthMap;
}

float ROSImageStreamThread::getDepth(int x, int y)
{
    if (this->tunnel_radius > 0 && haveCalib && use_tunnel_estimator) {
		//Deal with cropping
		//x += (old_width_-width_)/2;
		//y += (old_height_-height_)/2;

        tf2::Stamped<tf2::Transform> pose;
        cv::Point3d cam_pt = this->calcProjectionCameraFrame(x, y);
        
		//ROS_INFO("%f, %f, %f", cam_pt.x, cam_pt.y, cam_pt.z);

		this->unitVectorToPose("cam_top_left", cam_pt, pose);
        return this->calcDistance(pose, this->top_left_transform);
    } else if (haveDepthMap && haveCalib) {
		cv::Point3d vec = this->calcProjectionCameraFrame(x,y);
		cv::Point3d vec_c = this->calcProjectionCameraFrame(width_/2,height_/2);
		
		//ROS_INFO("%f, %f, %f", vec.x, vec.y, vec.z);
		return depth_map.at<float>(y,x)*vec.dot(vec_c);
	}
    return -1;
}

SE3NoX ROSImageStreamThread::getTransform()
{
	//Transform FROM tunnel frame TO robot frame
	tf2::Transform tf_transform = this->top_left_transform.inverse();
	Eigen::Quaterniond quat;
	Eigen::Vector2d trans(tf_transform.getOrigin().getY(), tf_transform.getOrigin().getZ());
	tf2::convert(tf_transform.getRotation(), quat);

	return SE3NoX(quat, trans);
	//return SE3NoX(0,0,0,0,0);
}

float ROSImageStreamThread::calcDistance(tf2::Stamped<tf2::Transform>& vec, tf2::Stamped<tf2::Transform>& transform)
{
    //Location of ROBOT, assuming TUNNEL is the global frame
    tf2::Transform robot_pose = transform*vec;
    //ROS_INFO("%f, %f, %f", robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), robot_pose.getOrigin().getZ());
    
    //Get a direction vector from the quaternion
    tf2::Quaternion ray_direction(1,0,0,0);
    ray_direction = robot_pose.getRotation()*ray_direction*robot_pose.getRotation().inverse();
    //ROS_INFO("%f, %f, %f, %f", ray_direction.x(), ray_direction.y(), ray_direction.z(), ray_direction.w());

    //Convert to eigen datatypes, which are much faster
    Eigen::Vector3d pos(robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), robot_pose.getOrigin().getZ());
    Eigen::Vector3d dir(ray_direction.x(), ray_direction.y(), ray_direction.z());

    //project onto y-z plane (cross sectional plane of tunnel), magnitude doesn't matter
    Eigen::Vector3d proj = Eigen::Vector3d::UnitX().cross(dir.cross(Eigen::Vector3d::UnitX()));
    
    //Assume parametrization x=x0+at, y=y0+bt, z=z0+ct where a^2+b^2+c^2=1
    double under_sqrt = proj(1)*proj(1)*(this->tunnel_radius*this->tunnel_radius-pos(2)*pos(2));//a^2*(R^2-y0^2)
    under_sqrt += proj(2)*proj(2)*(this->tunnel_radius*this->tunnel_radius-pos(1)*pos(1));//b^2*(R^2-x0^2)
    under_sqrt += 2*proj(1)*proj(2)*pos(1)*pos(2);//2*a*b*x0*y0
    double t = -proj(1)*pos(1)-proj(2)*pos(2);//-a*x0-b*y0
    t += sqrt(under_sqrt);
    t /= proj(1)*proj(1) + proj(2)*proj(2);

    return t*dir.dot(this->focal_plane_dir);
}

//Convert from image to world coordinates IN CAMERA FRAME
cv::Point3d ROSImageStreamThread::calcProjectionCameraFrame(int x, int y)
{
    cv::Matx31d hom_pt(x, y, 1);

    hom_pt = this->cam_intrinsics.inv()*hom_pt; //put in world coordinates

	//Flip around axes because camera is rotated
    cv::Point3f direction(-hom_pt(1),hom_pt(0),hom_pt(2));

    //Normalize so we have a unit vector
    direction *= 1/cv::norm(direction);

    return direction;
}

//Convert from unit vector to PoseStamped message
void ROSImageStreamThread::unitVectorToPose(const std::string& frame, cv::Point3f vec, tf2::Stamped<tf2::Transform>& trans)
{
    //ROS_INFO("%f, %f, %f", vec.x, vec.y, vec.z);

    //Convert to axis angle
    cv::Point3f rel_vec(1,0,0);
    float angle = -acos(vec.dot(rel_vec));
    //ROS_INFO("%f", angle);
    cv::Point3f axis = vec.cross(rel_vec);

    //Assemble rest of message
    tf2::Quaternion q;
    q.setRotation(tf2::Vector3(axis.x, axis.y, axis.z), angle);
    trans.setRotation(q);
    trans.setOrigin(tf2::Vector3(0,0,0));

    trans.frame_id_ = frame;
    trans.stamp_ = ros::Time::now();
}

ROSImageStreamThread::~ROSImageStreamThread()
{
	delete imageBuffer;
}

void ROSImageStreamThread::setCalibration(std::string file)
{
	if(file == "")
	{
		ros::Subscriber info_sub         = nh_.subscribe(nh_.resolveName("camera_info"),1, &ROSImageStreamThread::infoCb, this);

		printf("WAITING for ROS camera calibration!\n");
		while(width_ == 0)
		{
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		info_sub.shutdown();
	}
	else
	{
		undistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(undistorter==0)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			exit(0);
		}

		width_ = undistorter->getOutputWidth();
		height_ = undistorter->getOutputHeight();
		old_width_ = undistorter->getInputWidth();
		old_height_ = undistorter->getInputHeight();

		fx_ = undistorter->getK().at<double>(0, 0);
		fy_ = undistorter->getK().at<double>(1, 1);
		cx_ = undistorter->getK().at<double>(0, 2);
		cy_ = undistorter->getK().at<double>(1, 2);

    	this->cam_intrinsics = cv::Matx33d(fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);//make Matx33f
		std::cout << this->cam_intrinsics << "\n";
		//ROS_INFO("camera: %f", this->cam_intrinsics(0,0));
	}

	haveCalib = true;
}

void ROSImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void ROSImageStreamThread::operator()()
{
	ros::spin();

	exit(0);
}

void ROSImageStreamThread::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr msg)
{
	//Need camera matrix to do anything useful with data
	//Don't need if we are using the tunnel estimator data
	if (!haveCalib || use_tunnel_estimator) return;

	if (!have_sensor_pose) {
		tf2::Stamped<tf2::Transform> sensor_pose_tf2;
		tf2::convert(tf_buffer->lookupTransform("velodyne", "cam_top_left", ros::Time(0), ros::Duration(1.0)), sensor_pose_tf2);
		Eigen::Quaterniond quat;
		tf2::convert(sensor_pose_tf2.getRotation(), quat); 
		sensor_pose.translation() << sensor_pose_tf2.getOrigin().getX(), sensor_pose_tf2.getOrigin().getY(), sensor_pose_tf2.getOrigin().getZ();
		sensor_pose.linear() = quat.normalized().toRotationMatrix().cast<float>();

		ROS_INFO_STREAM("Camera to LIDAR Pose is: \n" << sensor_pose.matrix() << "\n");

		//Set up lidar and camera calibration
		/*sensor_pose.translation() << -0.0579167, 0.0585683, 0.0; //Translation matrix
		sensor_pose.linear() <<  0.999899,   0.0136208,  0.00416754,
								-0.0138209, 0.998525,   0.0525113,
								-0.00344614,-0.0525636, 0.998612; //Rotation matrix
		sensor_pose.linear() = sensor_pose.linear().transpose().eval(); */
		//sensor_pose.linear().setIdentity();

		have_sensor_pose = true;
	}

	//ROS_INFO("Got Pointcloud...");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc =
		pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *pc);

	//ROS_INFO_STREAM(*pc << "\n");

	pcl::RangeImagePlanar rangeImage;
	
	rangeImage.createFromPointCloudWithFixedSize(*pc, width_/initResScaleWidth, height_/initResScaleHeight,
									cx_/initResScaleWidth, cy_/initResScaleHeight, 
									fx_/initResScaleWidth, fy_/initResScaleHeight, 
									sensor_pose, pcl::RangeImagePlanar::LASER_FRAME);

	cv::Mat image = cv::Mat(rangeImage.height, rangeImage.width, CV_32F, -1.0);

	for (int y=0; y<rangeImage.height; y+=1) {
		for (int x=0; x<rangeImage.width; x+=1) {
			if (rangeImage.getPoint(y*rangeImage.width + x).range > 0) {
				image.at<float>(y,x) = (rangeImage.getPoint(y*rangeImage.width + x).range);
				//ROS_INFO("depth: %f",image.at<float>(y,x));
			}
		}
	}

	//std::cout << rangeImage << "\n";
	cv::resize(image,depth_map,cv::Size(width_, height_),0,0,INTER_AREA);//resize image

	if (fillDepthHoles) {
		cv::Mat inpaint_mask;
		//Create mask to find holes
		inpaint_mask = cv::Mat::zeros(depth_map.size(), CV_8UC1);
		inpaint_mask.setTo(255, depth_map == 0);
		//Don't treat the top and bottom of the image as "holes"
		cv::floodFill(inpaint_mask, cv::Point(1,1), 0);
		cv::floodFill(inpaint_mask, cv::Point(1,depth_map.size().height-1), 0);
		//Fill holes with interpolation
		cv::inpaint(depth_map, inpaint_mask, depth_map, 1, cv::INPAINT_NS);
	}

	haveDepthMap = true;
}

void ROSImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr img)
{
	if(!haveCalib) return;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	if(img->header.seq < (unsigned int)lastSEQ)
	{
		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
		lastSEQ = 0;
		return;
	}
	lastSEQ = img->header.seq;

	TimestampedMat bufferItem;
	if(img->header.stamp.toSec() != 0)
		bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
	else
		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

	if(undistorter != 0)
	{
		assert(undistorter->isValid());
		undistorter->undistort(cv_ptr->image,bufferItem.data);
	}
	else
	{
		//ROS_INFO("%i, %i", this->width_, this->height_);
		//cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(this->width_, this->height_));
		bufferItem.data = cv_ptr->image;
	}

	cv::MatIterator_<uchar> it_grey, end_grey;
	it_grey = bufferItem.data.begin<uchar>();
	end_grey = bufferItem.data.end<uchar>();
	//Convert to CIE L*a*b* (takes 2 steps)
	if (maskBrightnessLimit < 255) {
		cv::Mat img_lab;
		cvtColor(bufferItem.data, img_lab, cv::COLOR_GRAY2RGB);
		cvtColor(img_lab, img_lab, cv::COLOR_RGB2Lab);
		//Scan through image pixels
		cv::MatIterator_<Vec3b> it_lab, end_lab;
		it_lab = img_lab.begin<Vec3b>();
		end_lab = img_lab.end<Vec3b>();
		
		for( ; it_grey != end_grey, it_lab != end_lab; ++it_grey, ++it_lab)
		{
			if ((*it_lab)[0] > maskBrightnessLimit) { //Is brightness above a certain level?
				(*it_grey) = 0;
			}
		}
	}
	
	if (maskRectangle) {
		int cnt = 0;
		for( ; it_grey != end_grey; ++it_grey)
		{
			if (cnt % width_ > maskRectangleLeft && cnt % width_ < maskRectangleRight && cnt / width_ > maskRectangleTop && cnt / width_ < maskRectangleBottom) {
				(*it_grey) = 0;
			}
			cnt ++;
		}
	}

	//printf("Got image");

	imageBuffer->pushBack(bufferItem);
}

void ROSImageStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if(!haveCalib)
	{
		fx_ = info->P[0];
		fy_ = info->P[5];
		cx_ = info->P[2];
		cy_ = info->P[6];

		this->cam_intrinsics = cv::Matx33d(fx_,0,cx_,0,fy_,cy_,0,0,1);

		width_ = info->width;
		height_ = info->height;

		haveCalib = true;

		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
	}
}

void ROSImageStreamThread::radiusCb(const std_msgs::Float64::ConstPtr& msg)
{
    //Grab radius
    this->tunnel_radius = msg->data;

	//ROS_INFO("%f", this->tunnel_radius);

    //Load current camera transform while we are at it
    tf2::convert(tf_buffer->lookupTransform("center_cylinder", "cam_top_left", ros::Time(0), ros::Duration(1.0)), this->top_left_transform);       

	//Find the normal vector of the focal plane
	tf2::Stamped<tf2::Transform> pose;
	cv::Point3d cam_pt = this->calcProjectionCameraFrame((width_)/2, (height_)/2);
    //printf("%f, %f, %f\n", cam_pt.x, cam_pt.y, cam_pt.z);
	this->unitVectorToPose("cam_top_left", cam_pt, pose);
	tf2::Transform robot_pose = this->top_left_transform*pose;
    tf2::Quaternion ray_direction(1,0,0,0);
    ray_direction = robot_pose.getRotation()*ray_direction*robot_pose.getRotation().inverse();
    //Convert to eigen datatypes, which are much faster
    this->focal_plane_dir = Eigen::Vector3d(ray_direction.x(), ray_direction.y(), ray_direction.z());

	haveDepthMap = true;
}

}
