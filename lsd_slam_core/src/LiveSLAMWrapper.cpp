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

#include "LiveSLAMWrapper.h"
#include <vector>
#include "util/SophusUtil.h"

#include "SlamSystem.h"

#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/InputImageStream.h"
#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "util/globalFuncs.h"

#include <iostream>

#include "opencv2/opencv.hpp"

namespace lsd_slam
{


LiveSLAMWrapper::LiveSLAMWrapper(InputImageStream* imageStream, Output3DWrapper* outputWrapper)
{
	this->imageStream = imageStream;
	this->outputWrapper = outputWrapper;
	imageStream->getImageBuffer()->setReceiver(this);
	imageStream->getSegBuffer()->setReceiver(this);

	fx = imageStream->fx();
	fy = imageStream->fy();
	cx = imageStream->cx();
	cy = imageStream->cy();
	width = imageStream->width();
	height = imageStream->height();

	outFileName = packagePath+"estimated_poses.txt";


	isInitialized = false;


	Sophus::Matrix3f K_sophus;
	K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	outFile = nullptr;


	// make Odometry
	initialCamera = 0;
	monoOdometry = new SlamSystem(width, height, K_sophus, doSlam, initialCamera);

	monoOdometry->setVisualization(outputWrapper);
	monoOdometry->setLidarDepth(((ROSImageStreamThread*)imageStream));

	imageSeqNumber = 0;
}


LiveSLAMWrapper::~LiveSLAMWrapper()
{
	if(monoOdometry != 0)
		delete monoOdometry;
	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}
}

void LiveSLAMWrapper::Loop()
{
	while (true) {
		boost::unique_lock<boost::recursive_mutex> waitImageLock(imageStream->getImageBuffer()->getMutex());
		while (!fullResetRequested && !(imageStream->getImageBuffer()->size() > 0)) {
			notifyCondition.wait(waitImageLock);
		}
		waitImageLock.unlock();

		boost::unique_lock<boost::recursive_mutex> waitSegLock(imageStream->getSegBuffer()->getMutex());
		while (!fullResetRequested && !(imageStream->getSegBuffer()->size() > 0)) {
			notifyCondition.wait(waitSegLock);
		}
		waitSegLock.unlock();
		
		if(fullResetRequested)
		{
			resetAll();
			fullResetRequested = false;
			if (!(imageStream->getImageBuffer()->size() > 0))
				continue;
		}
		
		TimestampedMultiMat image = imageStream->getImageBuffer()->first();
		TimestampedSegmentation conf_mat = imageStream->getSegBuffer()->first();

		//If we are synched properly
		if (image.id_num == conf_mat.id_num) {
			std::cout << "img: " << image.timestamp.toSec() << "\n";
			std::cout << "conf: " << conf_mat.timestamp.toSec() << "\n";
			// process image
			imageStream->getImageBuffer()->popFront();
			imageStream->getSegBuffer()->popFront();
			
			newImageCallback(image.data, conf_mat.data, image.timestamp);
		} else {
			if (image.id_num < conf_mat.id_num)
				imageStream->getImageBuffer()->popFront();
			else
				imageStream->getSegBuffer()->popFront();
			ROS_WARN_STREAM("Out of synch.  Cam: " << image.id_num << " Seg: " << conf_mat.id_num << "\n");
		}

		Util::displayThreadLoop();

		int key = cv::waitKey(1) & 255; // key is an integer here
		if (key == 27) {
			printf("Quitting...");
			break;            // break when `esc' key is pressed
		}
	}
}


void LiveSLAMWrapper::newImageCallback(const cv::Mat img[NUM_CAMERAS], const cv::Mat seg[NUM_CAMERAS], Timestamp imgTime)
{
	++ imageSeqNumber;

	cv::Mat grayImg[NUM_CAMERAS];
	uchar* imagePtrs[NUM_CAMERAS];
	float* segPtrs[NUM_CAMERAS];
	for (int i=0; i<NUM_CAMERAS; i++) {
		// Convert image to grayscale, if necessary
		if (img[i].channels() == 1)
			grayImg[i] = img[i];
		else
			cvtColor(img[i], grayImg[i], CV_RGB2GRAY);
		

		// Assert that we work with 8 bit images
		assert(grayImg[i].elemSize() == 1);
		assert(fx != 0 || fy != 0);

		imagePtrs[i] = grayImg[i].data;
		segPtrs[i] = (float*)(seg[i].data);
	}

	// need to initialize
	if(!isInitialized)
	{
		//Spin until depth map loaded
		while (((ROSImageStreamThread*)imageStream)->depthReady() == false) {
			//ROS_INFO("Waiting for depth map...");
		}

		float depth[width*height];
		for (int x=0; x<width; x++) {
			for (int y=0; y<height; y++) {
				depth[x+y*width] = ((ROSImageStreamThread*)imageStream)->getDepth(x,y,initialCamera);
			}
		}
		//monoOdometry->randomInit(grayImg.data, imgTime.toSec(), 1);
		monoOdometry->gtDepthInit(imagePtrs, depth, imgTime.toSec(), 1);
		isInitialized = true;
	}
	else if(isInitialized && monoOdometry != nullptr)
	{
		struct timeval tv_start, tv_end;
		gettimeofday(&tv_start, NULL);
		monoOdometry->trackFrame(imagePtrs,segPtrs,imageSeqNumber,false,imgTime.toSec());
		gettimeofday(&tv_end, NULL);
		if(enablePrintDebugInfo && printOverallTiming) {
			float msTrack = ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
			printf("Tracking function: %fms\n", msTrack);
		}
		/*if (imageSeqNumber == 40) {
			std::cout << "Switiching Cameras!\n";
			monoOdometry->switchCameras(2);
		}*/
	}
}

void LiveSLAMWrapper::logCameraPose(const SE3& camToWorld, double time)
{
	Sophus::Quaternionf quat = camToWorld.unit_quaternion().cast<float>();
	Eigen::Vector3f trans = camToWorld.translation().cast<float>();

	char buffer[1000];
	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
			time,
			trans[0],
			trans[1],
			trans[2],
			quat.x(),
			quat.y(),
			quat.z(),
			quat.w());

	if(outFile == 0)
		outFile = new std::ofstream(outFileName.c_str());
	outFile->write(buffer,num);
	outFile->flush();
}

void LiveSLAMWrapper::requestReset()
{
	fullResetRequested = true;
	notifyCondition.notify_all();
}

void LiveSLAMWrapper::resetAll()
{
	if(monoOdometry != nullptr)
	{
		delete monoOdometry;
		printf("Deleted SlamSystem Object!\n");

		Sophus::Matrix3f K;
		K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		monoOdometry = new SlamSystem(width,height,K, doSlam);
		monoOdometry->setVisualization(outputWrapper);
		monoOdometry->setLidarDepth(((ROSImageStreamThread*)imageStream));
	}
	imageSeqNumber = 0;
	isInitialized = false;

	Util::closeAllWindows();

}

}
