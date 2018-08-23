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
	nh_ = ros::NodeHandle();
	//load params
	nh_.param("/lsd_slam/use_tunnel_estimator", use_tunnel_estimator, false);

	ROS_INFO("Using tunnel estimator: %d", use_tunnel_estimator);

	// wait for cam calib
	width_ = height_ = 0;
	have_sensor_pose = false;

	// subscribe
	top_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, nh_.resolveName("top_left_image"), 1);
	/*top_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, nh_.resolveName("top_right_image"), 1);
	bottom_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, nh_.resolveName("bottom_left_image"), 1);
	bottom_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, nh_.resolveName("bottom_right_image"), 1);

	image_sub = new message_filters::Synchronizer<SynchPolicy>(SynchPolicy(5),
		*top_left_sub, *top_right_sub, *bottom_left_sub, *bottom_right_sub);

	image_sub->registerCallback(boost::bind(&ROSImageStreamThread::vidCb, this, _1, _2, _3, _4));*/
	top_left_sub->registerCallback(boost::bind(&ROSImageStreamThread::vidCb, this, _1));

	pointcloud_sub = nh_.subscribe(nh_.resolveName("pointcloud"), 1, &ROSImageStreamThread::pointCloudCb, this);
	std::string radius_channel = nh_.resolveName("local_tunnel_radius");
	radius_sub = nh_.subscribe(radius_channel,1, &ROSImageStreamThread::radiusCb, this);

    // tf2 listener and buffer
    this->tf_buffer = new tf2_ros::Buffer();
    this->tf_listener = new tf2_ros::TransformListener(*(this->tf_buffer));

    tunnel_radius = -1;

	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMultiMat>(8);
	for (int i=0; i<NUM_CAMERAS; i++) {
		undistorter[i] = 0;
	}
	lastSEQ = 0;

	haveCalib = false;
	haveDepthMap = false;
}

bool ROSImageStreamThread::depthReady()
{
	return haveDepthMap;
}

float ROSImageStreamThread::getDepth(int x, int y, int cam)
{
    if (tunnel_radius > 0 && haveCalib && use_tunnel_estimator) {
		//Deal with cropping
		//x += (old_width_-width_)/2;
		//y += (old_height_-height_)/2;

        Eigen::Vector3f cam_pt = calcProjectionCameraFrame(x, y);
        
		//ROS_INFO("%f, %f, %f", cam_pt.x, cam_pt.y, cam_pt.z);
        return calcDistance(cam_pt, cam);
    } else if (haveDepthMap && haveCalib) {
		Eigen::Vector3f vec = calcProjectionCameraFrame(x,y);
		Eigen::Vector3f vec_c = calcProjectionCameraFrame(width_/2,height_/2);
		
		//ROS_INFO("%f, %f, %f", vec.x, vec.y, vec.z);
		return depth_map.at<float>(y,x)*vec.dot(vec_c);
	}
    return -1;
}

SE3NoX ROSImageStreamThread::getTransformFromTunnel(int cam)
{
	Eigen::Vector2d trans(cam_pose[cam].translation().y(), cam_pose[cam].translation().z());
	Eigen::Matrix3d rot = cam_pose[cam].linear().cast<double>();

	return SE3NoX(rot, trans);
	//return SE3NoX(0,0,0,0,0);
}

SE3 ROSImageStreamThread::getTransformBetweenCameras(int from, int to)
{
	tf2::Stamped<tf2::Transform> tf_transform;
	tf2::convert(tf_buffer->lookupTransform(camera_names[to], camera_names[from], ros::Time(0), ros::Duration(1.0)), tf_transform); 

	Eigen::Quaterniond quat;
	Eigen::Vector3d trans(tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY(), tf_transform.getOrigin().getZ());
	tf2::convert(tf_transform.getRotation(), quat);

	SE3 sophus_trans;
	sophus_trans.setQuaternion(quat);
	sophus_trans.translation() = trans;

	return sophus_trans;
}

float ROSImageStreamThread::getRadius()
{
	return tunnel_radius;
}

float ROSImageStreamThread::calcDistance(Eigen::Vector3f &ray_direction, int cam)
{
    Eigen::Vector3f pos = cam_pose[cam].translation();
    Eigen::Vector3f dir(ray_direction.x(), ray_direction.y(), ray_direction.z());
    dir = cam_pose[cam].linear()*dir;

	//ROS_INFO_STREAM("Ray direction: \n" << ray_direction << "\ndir: \n" << dir << "\ncam pose: \n" << cam_pose[cam].matrix());

    //project onto y-z plane (cross sectional plane of tunnel), magnitude doesn't matter
    Eigen::Vector3f proj = Eigen::Vector3f::UnitX().cross(dir.cross(Eigen::Vector3f::UnitX()));
    
    //Assume parametrization x=x0+at, y=y0+bt, z=z0+ct where a^2+b^2+c^2=1
    double under_sqrt = proj(1)*proj(1)*(tunnel_radius*tunnel_radius-pos(2)*pos(2));//a^2*(R^2-y0^2)
    under_sqrt += proj(2)*proj(2)*(tunnel_radius*tunnel_radius-pos(1)*pos(1));//b^2*(R^2-x0^2)
    under_sqrt += 2*proj(1)*proj(2)*pos(1)*pos(2);//2*a*b*x0*y0
    double t = -proj(1)*pos(1)-proj(2)*pos(2);//-a*x0-b*y0
    t += sqrt(under_sqrt);
    t /= proj(1)*proj(1) + proj(2)*proj(2);

	//ROS_INFO_STREAM("Raw dist: " << t);
    return t*(dir.dot(focal_plane_dir[cam]));
}

//Convert from image to world coordinates IN CAMERA FRAME
Eigen::Vector3f ROSImageStreamThread::calcProjectionCameraFrame(int x, int y)
{
    Eigen::Vector3f hom_pt(x, y, 1);
    hom_pt = cam_intrinsics.inverse()*hom_pt; //put in world coordinates
    hom_pt.normalize();

    return hom_pt;
}

ROSImageStreamThread::~ROSImageStreamThread()
{
	delete image_sub;
	delete top_left_sub;
	delete bottom_left_sub;
	delete top_right_sub;
	delete bottom_right_sub;
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
		std::ifstream infile(file);
		assert(infile.good());

		std::string line;

		for (int i=0; i<NUM_CAMERAS; i++) {
			std::getline(infile,line);
			undistorter[i] = Undistorter::getUndistorterForFile(line.c_str());

			if(undistorter[i]==0)
			{
				printf("Failed to read camera calibration %n from file... wrong syntax?\n", i);
				exit(0);
			}
		}

		//Assume all the cameras are the same
		//Make projection matrices all the same
		width_ = undistorter[0]->getOutputWidth();
		height_ = undistorter[0]->getOutputHeight();
		old_width_ = undistorter[0]->getInputWidth();
		old_height_ = undistorter[0]->getInputHeight();

		fx_ = undistorter[0]->getK().at<double>(0, 0);
		fy_ = undistorter[0]->getK().at<double>(1, 1);
		cx_ = undistorter[0]->getK().at<double>(0, 2);
		cy_ = undistorter[0]->getK().at<double>(1, 2);

    	cam_intrinsics << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;//make Matx33f
		std::cout << cam_intrinsics << "\n";
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

void ROSImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr top_left_img/*,
			   					 const sensor_msgs::ImageConstPtr top_right_img, 
			   					 const sensor_msgs::ImageConstPtr bottom_left_img, 
			   					 const sensor_msgs::ImageConstPtr bottom_right_img*/)
{
	if(!haveCalib) return;
	struct timeval tv_start, tv_end;
	gettimeofday(&tv_start, NULL);

	cv_bridge::CvImagePtr cv_ptr[NUM_CAMERAS];
	cv_ptr[0] = cv_bridge::toCvCopy(top_left_img, sensor_msgs::image_encodings::MONO8);
	/*cv_ptr[1] = cv_bridge::toCvCopy(top_right_img, sensor_msgs::image_encodings::MONO8);
	cv_ptr[2] = cv_bridge::toCvCopy(bottom_left_img, sensor_msgs::image_encodings::MONO8);
	cv_ptr[3] = cv_bridge::toCvCopy(bottom_right_img, sensor_msgs::image_encodings::MONO8);*/

	if(top_left_img->header.seq < (unsigned int)lastSEQ)
	{
		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
		lastSEQ = 0;
		return;
	}
	lastSEQ = top_left_img->header.seq;

	TimestampedMultiMat bufferItem;
	if(top_left_img->header.stamp.toSec() != 0)
		bufferItem.timestamp =  Timestamp(top_left_img->header.stamp.toSec());
	else
		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

	for (int i=0; i<NUM_CAMERAS; i++) {
		if(undistorter[i] != 0)
		{
			assert(undistorter[i]->isValid());
			undistorter[i]->undistort(cv_ptr[i]->image,bufferItem.data[i]);
		}
		else
		{
			//ROS_INFO("%i, %i", this->width_, this->height_);
			//cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(this->width_, this->height_));
			bufferItem.data[i] = cv_ptr[i]->image;
		}

		cv::MatIterator_<uchar> it_grey, end_grey;
		it_grey = bufferItem.data[i].begin<uchar>();
		end_grey = bufferItem.data[i].end<uchar>();
		//Convert to CIE L*a*b* (takes 2 steps)
		if (maskBrightnessLimit < 255) {
			cv::Mat img_lab;
			cvtColor(bufferItem.data[i], img_lab, cv::COLOR_GRAY2RGB);
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
				if (cnt % width_ > 0 && cnt % width_ < width_ && cnt / width_ > 0 && cnt / width_ < 100) {
					(*it_grey) = 0;
				}
				if (cnt % width_ > 0 && cnt % width_ < width_ && cnt / width_ > height_-100 && cnt / width_ < height_) {
					(*it_grey) = 0;
				}
				cnt ++;
			}
		}
	}

	//printf("Got image");

	imageBuffer->pushBack(bufferItem);

	gettimeofday(&tv_end, NULL);
	if(enablePrintDebugInfo && printOverallTiming) {
		float msCb = ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
		printf("Image Callback: %fms\n", msCb);
	}
}

void ROSImageStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if(!haveCalib)
	{
		fx_ = info->P[0];
		fy_ = info->P[5];
		cx_ = info->P[2];
		cy_ = info->P[6];

		cam_intrinsics << fx_,0,cx_,0,fy_,cy_,0,0,1;

		width_ = info->width;
		height_ = info->height;

		haveCalib = true;

		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
	}
}

void ROSImageStreamThread::radiusCb(const std_msgs::Float64::ConstPtr& msg)
{
    //Grab radius
    tunnel_radius = msg->data;

	//ROS_INFO("%f", this->tunnel_radius);

	for (int i=0; i<NUM_CAMERAS; i++) {
		//Load current camera transform while we are at it
		tf2::Stamped<tf2::Transform> transform;
    	tf2::convert(tf_buffer->lookupTransform("center_cylinder", camera_names[i], ros::Time(0), ros::Duration(1.0)), transform);
		//Convert to Eigen
		Eigen::Quaterniond quat;
		Eigen::Vector3f trans(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
		tf2::convert(transform.getRotation(), quat);
		cam_pose[i].linear() = quat.toRotationMatrix().cast<float>();
		cam_pose[i].translation() = trans;

		//Find the normal vector of the focal plane
		Eigen::Vector3f cam_pt = calcProjectionCameraFrame((width_)/2, (height_)/2);
		//printf("%f, %f, %f\n", cam_pt.x, cam_pt.y, cam_pt.z);

		focal_plane_dir[i] = cam_pose[i].linear()*cam_pt;
	}

	haveDepthMap = true;
}

}
