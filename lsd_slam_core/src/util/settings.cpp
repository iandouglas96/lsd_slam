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

#include "util/settings.h"
#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>



namespace lsd_slam
{
RunningStats runningStats;


bool autoRun = true;
bool autoRunWithinFrame = true;

int debugDisplay = 0;

bool onSceenInfoDisplay = true;
bool displayDepthMap = true;
bool dumpMap = false;
bool doFullReConstraintTrack = false;

// dyn config
bool enablePrintDebugInfo = true;

bool printPropagationStatistics = true;
bool printFillHolesStatistics = false;
bool printObserveStatistics = false;
bool printObservePurgeStatistics = false;
bool printRegularizeStatistics = false;
bool printLineStereoStatistics = false;
bool printLineStereoFails = false;

bool printTrackingIterationInfoSE3 = false;
bool printTrackingIterationInfoDepth = false;

bool printFrameBuildDebugInfo = false;
bool printMemoryDebugInfo = false;

bool printKeyframeSelectionInfo = false;
bool printConstraintSearchInfo = true;
bool printOptimizationInfo = false;
bool printRelocalizationInfo = false;

bool printThreadingInfo = false;
bool printMappingTiming = false;
bool printOverallTiming = false;

bool plotTrackingIterationInfo = true;
bool plotSim3TrackingIterationInfo = false;
bool plotStereoImages = false;
bool plotTracking = true;


float freeDebugParam1 = 1;
float freeDebugParam2 = 1;
float freeDebugParam3 = 1;
float freeDebugParam4 = 1;
float freeDebugParam5 = 1;

float KFDistWeight = 10;
float KFUsageWeight = 10;

float minUseGrad = 20;
float cameraPixelNoise2 = 5*5;
float depthSmoothingFactor = 1;

int maskBrightnessLimit = 256; //256 is off
bool maskRectangle = false;
int maskRectangleLeft = 880;
int maskRectangleRight = 1280;
int maskRectangleTop = 0;
int maskRectangleBottom = 1024;

bool useCLAHE = true;
float claheClipLimit = 0.5;

int initResScaleHeight = 16;
int initResScaleWidth = 4;
bool fillDepthHoles = false;

bool allowNegativeIdepths = true;
bool useMotionModel = false;
bool useSubpixelStereo = true;
bool multiThreading = true;
bool useAffineLightningEstimation = true;



bool useFabMap = false;
bool doSlam = true;
bool doKFReActivation = true;
bool doMapping = true;

int maxLoopClosureCandidates = 0;
bool doLoopClosure = false;
int maxOptimizationIterations = 100;
int propagateKeyFrameDepthCount = 0;
float loopclosureStrictness = 1.5;
float relocalizationTH = 0.7;


bool saveKeyframes =  false;
bool saveAllTracked =  false;
bool saveLoopClosureImages =  false;
bool saveAllTrackingStages = false;
bool saveAllTrackingStagesInternal = false;

bool continuousPCOutput = true;


bool fullResetRequested = false;
bool manualTrackingLossIndicated = false;

bool doConstrainCrossSection = false;
float lidarDepthCovariance = 0.01;//1cm seems reasonable (but probably check this)
bool useLidarDepthUpdate = true;

std::string packagePath = "";


void handleKey(char k)
{
	char kkk = k;
	switch(kkk)
	{
	case 'a': case 'A':
//		autoRun = !autoRun;		// disabled... only use for debugging & if you really, really know what you are doing
		break;
	case 's': case 'S':
//		autoRunWithinFrame = !autoRunWithinFrame; 	// disabled... only use for debugging & if you really, really know what you are doing
		break;
	case 'd': case 'D':
		debugDisplay = (debugDisplay+1)%6;
		printf("debugDisplay is now: %d\n", debugDisplay);
		break;
	case 'e': case 'E':
		debugDisplay = (debugDisplay-1+6)%6;
		printf("debugDisplay is now: %d\n", debugDisplay);
		break;
	case 'o': case 'O':
		onSceenInfoDisplay = !onSceenInfoDisplay;
		break;
	case 'r': case 'R':
		printf("requested full reset!\n");
		fullResetRequested = true;
		break;
	case 'm': case 'M':
		printf("Dumping Map!\n");
		dumpMap = true;
		break;
	case 'p': case 'P':
		printf("Tracking all Map-Frames again!\n");
		doFullReConstraintTrack = true;
		break;
	case 'l': case 'L':
		printf("Manual Tracking Loss Indicated!\n");
		manualTrackingLossIndicated = true;
		break;
	}

}

}
