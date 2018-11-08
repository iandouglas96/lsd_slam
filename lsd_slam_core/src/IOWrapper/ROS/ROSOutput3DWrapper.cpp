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

#include "ROSOutput3DWrapper.h"
#include "util/SophusUtil.h"
#include <ros/ros.h>
#include "util/settings.h"
#include "IOWrapper/ImageDisplay.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "std_msgs/Float32MultiArray.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2_eigen/tf2_eigen.h>
#include "GlobalMapping/g2oTypeSE3Sophus.h"
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace lsd_slam
{


ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height)
{
	this->width = width;
	this->height = height;

	liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
	liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

	keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
	keyframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

	graph_channel = nh_.resolveName("lsd_slam/graph");
	graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

	debugInfo_channel = nh_.resolveName("lsd_slam/debug");
	debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

	pose_channel = nh_.resolveName("lsd_slam/pose");
	pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);

	publishLvl=0;
}

ROSOutput3DWrapper::~ROSOutput3DWrapper()
{
}


void ROSOutput3DWrapper::publishKeyframe(Frame* f)
{
	//printf("Outputting PC to visualizer\n");

	lsd_slam_viewer::keyframeMsg fMsg;


	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

	fMsg.id = f->id();
	fMsg.time = f->timestamp();
	fMsg.isKeyframe = true;

	int w = f->width(publishLvl);
	int h = f->height(publishLvl);

	memcpy(fMsg.camToWorld.data(),f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = f->fx(publishLvl);
	fMsg.fy = f->fy(publishLvl);
	fMsg.cx = f->cx(publishLvl);
	fMsg.cy = f->cy(publishLvl);
	fMsg.width = w;
	fMsg.height = h;

	//If we have the pointcloud, publish it
	
	if (f->hasIDepthBeenSet() && publishPointcloud) {
		fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

		InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

		const float* idepth = f->idepth(publishLvl);
		const float* idepthVar = f->idepthVar(publishLvl);
		const float* color = f->image(publishLvl);

		for(int idx=0;idx < w*h; idx++)
		{
			pc[idx].idepth = idepth[idx];
			pc[idx].idepth_var = idepthVar[idx];
			pc[idx].color[0] = color[idx];
			pc[idx].color[1] = color[idx];
			pc[idx].color[2] = color[idx];
			pc[idx].color[3] = color[idx];
		}
	} else {
		fMsg.pointcloud.resize(0);
	}

	//load image and reference data
	if (f->segmentation() != NULL) {
		cv::Mat overlay = Util::renderSegmentation(f->segmentation(), w, h, NUM_SEG_CLASSES);
		//cv::Mat overlay = Util::renderSegmentationOverlay(f->segmentation(), f->image(0), w, h, NUM_SEG_CLASSES);
		fMsg.image.resize(sizeof(char)*w*h*3);
		memcpy(fMsg.image.data(), overlay.data, sizeof(char)*w*h*3); //Copy data

		int cam = f->id() % NUM_CAMERAS;
		int id = f->id() / NUM_CAMERAS;

		std::string path = "/media/ian/ResearchSSD/InspectionData/tunnel_seg/filtered/" + std::to_string(cam) + "/" + std::to_string(id) + ".bmp";
		cv::imwrite(path, overlay);
	}
	tf2::convert(f->tunnelPose().transform().translation(), fMsg.tunnel_pose.position);
	//std::cout << "pos: " << f->tunnelPose().transform().translation() << "\n";
	tf2::convert(f->tunnelPose().transform().unit_quaternion(), fMsg.tunnel_pose.orientation);
	fMsg.tunnel_radius = f->tunnelRadius();

	keyframe_publisher.publish(fMsg);
}

void ROSOutput3DWrapper::publishTrackedFrame(Frame* kf, int cam)
{
	//Regular message publisher (for viewer)
	lsd_slam_viewer::keyframeMsg fMsg;

	fMsg.id = kf->id();
	fMsg.time = kf->timestamp();
	fMsg.isKeyframe = false;


	memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);
	fMsg.width = kf->width(publishLvl);
	fMsg.height = kf->height(publishLvl);

	fMsg.pointcloud.clear();

	liveframe_publisher.publish(fMsg);


	SE3 camToWorld = (kf->getScaledCamToWorld());

	geometry_msgs::PoseStamped pMsg;

	pMsg.pose.position.x = camToWorld.translation()[0];
	pMsg.pose.position.y = camToWorld.translation()[1];
	pMsg.pose.position.z = camToWorld.translation()[2];
	pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	if (pMsg.pose.orientation.w < 0)
	{
		pMsg.pose.orientation.x *= -1;
		pMsg.pose.orientation.y *= -1;
		pMsg.pose.orientation.z *= -1;
		pMsg.pose.orientation.w *= -1;
	}

	pMsg.header.stamp = ros::Time(kf->timestamp());
	pMsg.header.frame_id = "world";
	pose_publisher.publish(pMsg);

	//tf publisher (for rviz)
	SE3 worldToCam = (kf->getScaledCamToWorld().inverse());
	static tf2_ros::TransformBroadcaster tf_br;
	geometry_msgs::TransformStamped transformStamped;
  
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = camera_names[cam];
	transformStamped.child_frame_id = "takeoff_top_left";
	transformStamped.transform.translation.x = worldToCam.translation()[0];
	transformStamped.transform.translation.y = worldToCam.translation()[1];
	transformStamped.transform.translation.z = worldToCam.translation()[2];
	transformStamped.transform.rotation.x = worldToCam.so3().unit_quaternion().x();
	transformStamped.transform.rotation.y = worldToCam.so3().unit_quaternion().y();
	transformStamped.transform.rotation.z = worldToCam.so3().unit_quaternion().z();
	transformStamped.transform.rotation.w = worldToCam.so3().unit_quaternion().w();

	tf_br.sendTransform(transformStamped);
}



void ROSOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
	lsd_slam_viewer::keyframeGraphMsg gMsg;

	graph->edgesListsMutex.lock();
	gMsg.numConstraints = graph->edgesAll.size();
	gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
	for(unsigned int i=0;i<graph->edgesAll.size();i++)
	{
		constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
		constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
		Sophus::Vector6d err;
		if (graph->edgesAll[i]->hasX) {
			err = graph->edgesAll[i]->edge->error();
		} else {
			err = graph->edgesAll[i]->edgeNoX->error();
		}
		constraintData[i].err = sqrt(err.dot(err));
	}
	graph->edgesListsMutex.unlock();

	graph->keyframesAllMutex.lock_shared();
	gMsg.numFrames = graph->keyframesAll.size();
	gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		framePoseData[i].id = graph->keyframesAll[i]->id();
		memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	}
	graph->keyframesAllMutex.unlock_shared();

	graph_publisher.publish(gMsg);
}

void ROSOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	std_msgs::Float32MultiArray msg;
	for(int i=0;i<20;i++)
		msg.data.push_back((float)(data[i]));

	debugInfo_publisher.publish(msg);
}

}
