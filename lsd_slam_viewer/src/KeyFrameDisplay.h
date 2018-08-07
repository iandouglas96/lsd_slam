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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#undef Success
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "boost/thread.hpp"

#include "QGLViewer/qglviewer.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "sophus/se3.hpp"

#include <sstream>
#include <fstream>

struct MyVertex
{
	float point[3];
	uchar color[4];
};

struct InputPointDense
{
	float idepth;
	float idepth_var;
	uchar color[4];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	KeyFrameDisplay();
	~KeyFrameDisplay();


	void setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void drawCam(float lineWidth = 1, float* color = 0);
	void drawPC(float pointSize = 1, float alpha = 1);
	void drawCylinderSegment();
	void refreshPC();

	int flushPC(std::ofstream* f);



	int id;
	double time;

	int totalPoints, displayedPoints;


	// camera pose
	// may be updated by kf-graph.
	Sophus::SE3f camToWorld;

private:
	void setTexture(cv::Mat &color_channel);
	void loadTexture();
	Eigen::Vector3f calcProjectionCameraFrame(float x, float y);
	float calcDistance(Eigen::Vector3f &ray_direction);
	void drawCam(float lineWidth);

	Eigen::Vector2f getLeftIntercept(float dist, float angle);
	Eigen::Vector2f getRightIntercept(float dist, float angle);

	// camera parameter
	// fixed.
	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;
	Eigen::Matrix3f cam_intrinsics;
	int width, height;

	Eigen::Affine3f robot_pose;
	float tunnel_radius;

	float my_scaledTH, my_absTH, my_scale;
	int my_minNearSupport;
	int my_sparsifyFactor;


	// pointcloud data & respective buffer
	InputPointDense* originalInput;

	//Raw image to render
	cv::Mat texture;
	boost::mutex texture_mutex;
	GLuint texture_handle;
	int texture_state;

	// buffer & how many
	GLuint vertexBufferId;
	int vertexBufferNumPoints;


	bool vertexBufferIdValid;	// true if the vertixBufferID is valid (doesnt mean the data in there is still valid)
	bool glBuffersValid;		// true if the vertexBufferID contains valid data

};



