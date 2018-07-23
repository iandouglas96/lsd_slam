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

#include "SE3DepthTracker.h"
#include <opencv2/highgui/highgui.hpp>
#include "DataStructures/Frame.h"
#include "Tracking/TrackingReference.h"
#include "util/globalFuncs.h"
#include "IOWrapper/ImageDisplay.h"
#include "Tracking/LGSX.h"

namespace lsd_slam
{


#if defined(ENABLE_NEON)
	#define callOptimized(function, arguments) function##NEON arguments
#else
	#if defined(ENABLE_SSE)
		#define callOptimized(function, arguments) (USESSE ? function##SSE arguments : function arguments)
	#else
		#define callOptimized(function, arguments) function arguments
	#endif
#endif



SE3DepthTracker::SE3DepthTracker(int w, int h, Eigen::Matrix3f K)
{
	width = w;
	height = h;

	this->K = K;
	fx = K(0,0);
	fy = K(1,1);
	cx = K(0,2);
	cy = K(1,2);

	settings = DenseDepthTrackerSettings();


	KInv = K.inverse();
	fxi = KInv(0,0);
	fyi = KInv(1,1);
	cxi = KInv(0,2);
	cyi = KInv(1,2);


	buf_warped_residual = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_weights = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_dx = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_dy = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_x = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_y = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_z = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));

	buf_d = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_residual_d = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_idepthVar = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_warped_idepthVar = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_weight_p = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_weight_d = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));

	buf_weight_Huber = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_weight_VarP = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));
	buf_weight_VarD = (float*)Eigen::internal::aligned_malloc(w*h*sizeof(float));

	buf_warped_size = 0;

	debugImageWeights = cv::Mat(height,width,CV_8UC3);
	debugImageResiduals = cv::Mat(height,width,CV_8UC3);
	debugImageSecondFrame = cv::Mat(height,width,CV_8UC3);
	debugImageOldImageWarped = cv::Mat(height,width,CV_8UC3);
	debugImageOldImageSource = cv::Mat(height,width,CV_8UC3);
	debugImageExternalWeights = cv::Mat(height,width,CV_8UC3);
	debugImageDepthResiduals = cv::Mat(height,width,CV_8UC3);
	debugImageScaleEstimation = cv::Mat(height,width,CV_8UC3);

	debugImageHuberWeight = cv::Mat(height,width,CV_8UC3);
	debugImageWeightD = cv::Mat(height,width,CV_8UC3);
	debugImageWeightP = cv::Mat(height,width,CV_8UC3);
	debugImageWeightedResP = cv::Mat(height,width,CV_8UC3);
	debugImageWeightedResD = cv::Mat(height,width,CV_8UC3);

	
	lastResidual = 0;
	iterationNumber = 0;
	lastDepthResidual = lastPhotometricResidual = lastDepthResidualUnweighted = lastPhotometricResidualUnweighted = lastResidualUnweighted = 0;
	pointUsage = 0;

}

SE3DepthTracker::~SE3DepthTracker()
{
	debugImageResiduals.release();
	debugImageWeights.release();
	debugImageSecondFrame.release();
	debugImageOldImageSource.release();
	debugImageOldImageWarped.release();
	debugImageExternalWeights.release();
	debugImageDepthResiduals.release();
	debugImageScaleEstimation.release();

	debugImageHuberWeight.release();
	debugImageWeightD.release();
	debugImageWeightP.release();
	debugImageWeightedResP.release();
	debugImageWeightedResD.release();


	Eigen::internal::aligned_free((void*)buf_warped_residual);
	Eigen::internal::aligned_free((void*)buf_warped_weights);
	Eigen::internal::aligned_free((void*)buf_warped_dx);
	Eigen::internal::aligned_free((void*)buf_warped_dy);
	Eigen::internal::aligned_free((void*)buf_warped_x);
	Eigen::internal::aligned_free((void*)buf_warped_y);
	Eigen::internal::aligned_free((void*)buf_warped_z);

	Eigen::internal::aligned_free((void*)buf_d);
	Eigen::internal::aligned_free((void*)buf_residual_d);
	Eigen::internal::aligned_free((void*)buf_idepthVar);
	Eigen::internal::aligned_free((void*)buf_warped_idepthVar);
	Eigen::internal::aligned_free((void*)buf_weight_p);
	Eigen::internal::aligned_free((void*)buf_weight_d);

	Eigen::internal::aligned_free((void*)buf_weight_Huber);
	Eigen::internal::aligned_free((void*)buf_weight_VarP);
	Eigen::internal::aligned_free((void*)buf_weight_VarD);
}


SE3 SE3DepthTracker::trackFrameSE3Depth(
		TrackingReference* reference,
		Frame* frame,
		const SE3& frameToReference_initialEstimate,
		int startLevel, int finalLevel)
{
	boost::shared_lock<boost::shared_mutex> lock = frame->getActiveLock();

	diverged = false;


	affineEstimation_a = 1; affineEstimation_b = 0;


	// ============ track frame ============
    SE3 referenceToFrame = frameToReference_initialEstimate.inverse();
	LGS6 ls6;


	int numCalcResidualCalls[PYRAMID_LEVELS];
	int numCalcWarpUpdateCalls[PYRAMID_LEVELS];

	SE3DepthResidualStruct finalResidual;

	bool warp_update_up_to_date = false;

	for(int lvl=startLevel;lvl >= finalLevel;lvl--)
	{
		numCalcResidualCalls[lvl] = 0;
		numCalcWarpUpdateCalls[lvl] = 0;

		if(settings.maxItsPerLvl[lvl] == 0)
			continue;

		reference->makePointCloud(lvl);

		// evaluate baseline-residual.
		callOptimized(calcSE3DepthBuffers, (reference, frame, referenceToFrame, lvl));
		if(buf_warped_size < 0.5 * MIN_GOODPERALL_PIXEL_ABSMIN * (width>>lvl)*(height>>lvl) || buf_warped_size < 10)
		{
			diverged = true;
			return SE3();
		}

		SE3DepthResidualStruct lastErr = callOptimized(calcSE3DepthWeightsAndResidual,(referenceToFrame));
		numCalcResidualCalls[lvl]++;

		if(useAffineLightningEstimation)
		{
			affineEstimation_a = affineEstimation_a_lastIt;
			affineEstimation_b = affineEstimation_b_lastIt;
		}

		float LM_lambda = settings.lambdaInitial[lvl];

		warp_update_up_to_date = false;
		for(int iteration=0; iteration < settings.maxItsPerLvl[lvl]; iteration++)
		{

			// calculate LS System, result is saved in ls.
			callOptimized(calcSE3DepthLGS,(ls6));
			warp_update_up_to_date = true;
			numCalcWarpUpdateCalls[lvl]++;

			iterationNumber = iteration;


			int incTry=0;
			while(true)
			{
				// solve LS system with current lambda
				Vector6 b = - ls6.b / ls6.num_constraints;
				Matrix6x6 A = ls6.A / ls6.num_constraints;
				for(int i=0;i<6;i++) A(i,i) *= 1+LM_lambda;
				Vector6 inc = A.ldlt().solve(b);
				incTry++;

				float absInc = inc.dot(inc);
				if(!(absInc >= 0 && absInc < 1))
				{
					// ERROR tracking diverged.
					lastSE3DepthHessian.setZero();
					return SE3();
				}

				// apply increment. pretty sure this way round is correct, but hard to test.
				SE3 new_referenceToFrame =SE3::exp(inc.cast<sophusType>()) * referenceToFrame;
				//Sim3 new_referenceToFrame = referenceToFrame * SE3::exp((inc));


				// re-evaluate residual
				callOptimized(calcSE3DepthBuffers,(reference, frame, new_referenceToFrame, lvl));
				if(buf_warped_size < 0.5 * MIN_GOODPERALL_PIXEL_ABSMIN * (width>>lvl)*(height>>lvl) || buf_warped_size < 10)
				{
					diverged = true;
					return SE3();
				}

				SE3DepthResidualStruct error = callOptimized(calcSE3DepthWeightsAndResidual,(new_referenceToFrame));
				numCalcResidualCalls[lvl]++;


				// accept inc?
				if(error.mean < lastErr.mean)
				{
					// accept inc
					referenceToFrame = new_referenceToFrame;
					warp_update_up_to_date = false;

					if(useAffineLightningEstimation)
					{
						affineEstimation_a = affineEstimation_a_lastIt;
						affineEstimation_b = affineEstimation_b_lastIt;
					}

					if(enablePrintDebugInfo && printTrackingIterationInfo)
					{
						// debug output
						printf("(%d-%d): ACCEPTED increment of %f with lambda %.1f, residual: %f -> %f\n",
								lvl,iteration, sqrt(inc.dot(inc)), LM_lambda, lastErr.mean, error.mean);

						printf("         p=%.4f %.4f %.4f %.4f %.4f %.4f\n",
								referenceToFrame.log()[0],referenceToFrame.log()[1],referenceToFrame.log()[2],
								referenceToFrame.log()[3],referenceToFrame.log()[4],referenceToFrame.log()[5]);
					}

					// converged?
					if(error.mean / lastErr.mean > settings.convergenceEps[lvl])
					{
						if(enablePrintDebugInfo && printTrackingIterationInfo)
						{
							printf("(%d-%d): FINISHED pyramid level (last residual reduction too small).\n",
									lvl,iteration);
						}
						iteration = settings.maxItsPerLvl[lvl];
					}

					finalResidual = lastErr = error;

					if(LM_lambda <= 0.2)
						LM_lambda = 0;
					else
						LM_lambda *= settings.lambdaSuccessFac;

					break;
				}
				else
				{
					if(enablePrintDebugInfo && printTrackingIterationInfo)
					{
						printf("(%d-%d): REJECTED increment of %f with lambda %.1f, (residual: %f -> %f)\n",
								lvl,iteration, sqrt(inc.dot(inc)), LM_lambda, lastErr.mean, error.mean);
					}

					if(!(inc.dot(inc) > settings.stepSizeMin[lvl]))
					{
						if(enablePrintDebugInfo && printTrackingIterationInfo)
						{
							printf("(%d-%d): FINISHED pyramid level (stepsize too small).\n",
									lvl,iteration);
						}
						iteration = settings.maxItsPerLvl[lvl];
						break;
					}

					if(LM_lambda == 0)
						LM_lambda = 0.2;
					else
						LM_lambda *= std::pow(settings.lambdaFailFac, incTry);
				}
			}
		}
	}



	if(enablePrintDebugInfo && printTrackingIterationInfo)
	{
		printf("Tracking: ");
			for(int lvl=PYRAMID_LEVELS-1;lvl >= 0;lvl--)
			{
				printf("lvl %d: %d (%d); ",
					lvl,
					numCalcResidualCalls[lvl],
					numCalcWarpUpdateCalls[lvl]);
			}

		printf("\n");


		printf("pOld = %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n",
				frameToReference_initialEstimate.inverse().log()[0],frameToReference_initialEstimate.inverse().log()[1],frameToReference_initialEstimate.inverse().log()[2],
				frameToReference_initialEstimate.inverse().log()[3],frameToReference_initialEstimate.inverse().log()[4],frameToReference_initialEstimate.inverse().log()[5],
				frameToReference_initialEstimate.inverse().log()[6]);
		printf("pNew = %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n",
				referenceToFrame.log()[0],referenceToFrame.log()[1],referenceToFrame.log()[2],
				referenceToFrame.log()[3],referenceToFrame.log()[4],referenceToFrame.log()[5],
				referenceToFrame.log()[6]);
		printf("final res mean: %f meanD %f, meanP %f\n", finalResidual.mean, finalResidual.meanD, finalResidual.meanP);
	}


	// Make sure that there is a warp update at the final position to get the correct information matrix
	if (!warp_update_up_to_date)
	{
		reference->makePointCloud(finalLevel);
		callOptimized(calcSE3DepthBuffers,(reference, frame, referenceToFrame, finalLevel));
	    finalResidual = callOptimized(calcSE3DepthWeightsAndResidual,(referenceToFrame));
	    callOptimized(calcSE3DepthLGS,(ls6));
	}

	lastSE3DepthHessian = ls6.A;

	lastResidual = finalResidual.mean;
	lastDepthResidual = finalResidual.meanD;
	lastPhotometricResidual = finalResidual.meanP;


	return referenceToFrame.inverse();
}




#if defined(ENABLE_SSE)
void SE3DepthTracker::calcSE3DepthBuffersSSE(
		const TrackingReference* reference,
		Frame* frame,
		const SE3& referenceToFrame,
		int level, bool plotWeights)
{
	calcSE3DepthBuffers(
			reference,
			frame,
			referenceToFrame,
			level, plotWeights);
}
#endif

#if defined(ENABLE_NEON)
void SE3DepthTracker::calcSim3BuffersNEON(
		const TrackingReference* reference,
		Frame* frame,
		const Sim3& referenceToFrame,
		int level, bool plotWeights)
{
	calcSim3Buffers(
			reference,
			frame,
			referenceToFrame,
			level, plotWeights);
}
#endif


void SE3DepthTracker::calcSE3DepthBuffers(
		const TrackingReference* reference,
		Frame* frame,
		const SE3& referenceToFrame,
		int level, bool plotWeights)
{

	// get static values
	int w = frame->width(level);
	int h = frame->height(level);
	Eigen::Matrix3f KLvl = frame->K(level);
	float fx_l = KLvl(0,0);
	float fy_l = KLvl(1,1);
	float cx_l = KLvl(0,2);
	float cy_l = KLvl(1,2);

	Eigen::Matrix3f rotMat = referenceToFrame.so3().matrix().cast<float>();
	Eigen::Matrix3f rotMatUnscaled = referenceToFrame.rotationMatrix().cast<float>();
	Eigen::Vector3f transVec = referenceToFrame.translation().cast<float>();

	// Calculate rotation around optical axis for rotating source frame gradients
	Eigen::Vector3f forwardVector(0, 0, -1);
	Eigen::Vector3f rotatedForwardVector = rotMatUnscaled * forwardVector;
	Eigen::Quaternionf shortestBackRotation;
	shortestBackRotation.setFromTwoVectors(rotatedForwardVector, forwardVector);
	Eigen::Matrix3f rollMat = shortestBackRotation.toRotationMatrix() * rotMatUnscaled;
	float xRoll0 = rollMat(0, 0);
	float xRoll1 = rollMat(0, 1);
	float yRoll0 = rollMat(1, 0);
	float yRoll1 = rollMat(1, 1);


	const Eigen::Vector3f* refPoint_max = reference->posData[level] + reference->numData[level];
	const Eigen::Vector3f* refPoint = reference->posData[level];
	const Eigen::Vector2f* refColVar = reference->colorAndVarData[level];
	const Eigen::Vector2f* refGrad = reference->gradData[level];

	const float* 			frame_idepth = frame->idepth(level);
	const float* 			frame_idepthVar = frame->idepthVar(level);
	const Eigen::Vector4f* 	frame_intensityAndGradients = frame->gradients(level);
	const float* frame_image = frame->image(level);


	float sxx=0,syy=0,sx=0,sy=0,sw=0;

	float usageCount = 0;

	int idx=0;
	for(;refPoint<refPoint_max; refPoint++, refGrad++, refColVar++)
	{
		//Ignore black pixels (mask out)
		if ((*refColVar)[0] <= 0) {
			//std::cout << "skipping...\n";
			continue;
		}

		Eigen::Vector3f Wxp = rotMat * (*refPoint) + transVec;
		float u_new = (Wxp[0]/Wxp[2])*fx_l + cx_l;
		float v_new = (Wxp[1]/Wxp[2])*fy_l + cy_l;

		// step 1a: coordinates have to be in image:
		// (inverse test to exclude NANs)
		if(!(u_new > 1 && v_new > 1 && u_new < w-2 && v_new < h-2))
			continue;

		int uv_new = int(u_new)+int(v_new)*w;
		//Ignore black pixels on new frame as well
		if (frame_image[uv_new] <= 0 || frame_image[uv_new+1] <= 0 || frame_image[uv_new+1+w] <= 0 || frame_image[uv_new+w] <= 0) {
			//std::cout << "skipping...\n";
			continue;
		}

		*(buf_warped_x+idx) = Wxp(0);
		*(buf_warped_y+idx) = Wxp(1);
		*(buf_warped_z+idx) = Wxp(2);

		Eigen::Vector3f resInterp = getInterpolatedElement43(frame_intensityAndGradients, u_new, v_new, w);


		// save values
#if USE_ESM_TRACKING == 1
		// get rotated gradient of point
		float rotatedGradX = xRoll0 * (*refGrad)[0] + xRoll1 * (*refGrad)[1];
		float rotatedGradY = yRoll0 * (*refGrad)[0] + yRoll1 * (*refGrad)[1];

		*(buf_warped_dx+idx) = fx_l * 0.5f * (resInterp[0] + rotatedGradX);
		*(buf_warped_dy+idx) = fy_l * 0.5f * (resInterp[1] + rotatedGradY);
#else
		*(buf_warped_dx+idx) = fx_l * resInterp[0];
		*(buf_warped_dy+idx) = fy_l * resInterp[1];
#endif


		float c1 = affineEstimation_a * (*refColVar)[0] + affineEstimation_b;
		float c2 = resInterp[2];
		float residual_p = c1 - c2;

		float weight = fabsf(residual_p) < 2.0f ? 1 : 2.0f / fabsf(residual_p);
		sxx += c1*c1*weight;
		syy += c2*c2*weight;
		sx += c1*weight;
		sy += c2*weight;
		sw += weight;


		*(buf_warped_residual+idx) = residual_p;
		*(buf_idepthVar+idx) = (*refColVar)[1];


		// new (only for depth):
		int idx_rounded = (int)(u_new+0.5f) + w*(int)(v_new+0.5f);
		float var_frameDepth = frame_idepthVar[idx_rounded];
		float ref_idepth = 1.0f / Wxp[2];
		*(buf_d+idx) = 1.0f / (*refPoint)[2];
		if(var_frameDepth > 0)
		{
			float residual_d = ref_idepth - frame_idepth[idx_rounded];
			*(buf_residual_d+idx) = residual_d;
			*(buf_warped_idepthVar+idx) = var_frameDepth;
		}
		else
		{
			*(buf_residual_d+idx) = -1;
			*(buf_warped_idepthVar+idx) = -1;
		}

		idx++;

		float depthChange = (*refPoint)[2] / Wxp[2];
		usageCount += depthChange < 1 ? depthChange : 1;
	}
	buf_warped_size = idx;


	pointUsage = usageCount / (float)reference->numData[level];

	affineEstimation_a_lastIt = sqrtf((syy - sy*sy/sw) / (sxx - sx*sx/sw));
	affineEstimation_b_lastIt = (sy - affineEstimation_a_lastIt*sx)/sw;

}


#if defined(ENABLE_SSE)
SE3DepthResidualStruct SE3DepthTracker::calcSE3DepthWeightsAndResidualSSE(
		const SE3& referenceToFrame)
{

	const __m128 txs = _mm_set1_ps((float)(referenceToFrame.translation()[0]));
	const __m128 tys = _mm_set1_ps((float)(referenceToFrame.translation()[1]));
	const __m128 tzs = _mm_set1_ps((float)(referenceToFrame.translation()[2]));

	const __m128 zeros = _mm_set1_ps(0.0f);
	const __m128 ones = _mm_set1_ps(1.0f);


	const __m128 depthVarFacs = _mm_set1_ps((float)settings.var_weight);// float depthVarFac = var_weight;	// the depth var is over-confident. this is a constant multiplier to remedy that.... HACK
	const __m128 sigma_i2s = _mm_set1_ps((float)cameraPixelNoise2);


	const __m128 huber_ress = _mm_set1_ps((float)(settings.huber_d));

	__m128 sumResP = zeros;
	__m128 sumResD = zeros;
	__m128 numTermsD = zeros;



	SE3DepthResidualStruct sumRes;
	memset(&sumRes, 0, sizeof(SE3DepthResidualStruct));


	for(int i=0;i<buf_warped_size-3;i+=4)
	{

		// calc dw/dd:
		 //float g0 = (tx * pz - tz * px) / (pz*pz*d);
		__m128 pzs = _mm_load_ps(buf_warped_z+i);	// z'
		__m128 pz2ds = _mm_rcp_ps(_mm_mul_ps(_mm_mul_ps(pzs, pzs), _mm_load_ps(buf_d+i)));  // 1 / (z' * z' * d)
		__m128 g0s = _mm_sub_ps(_mm_mul_ps(pzs, txs), _mm_mul_ps(_mm_load_ps(buf_warped_x+i), tzs));
		g0s = _mm_mul_ps(g0s,pz2ds);

		 //float g1 = (ty * pz - tz * py) / (pz*pz*d);
		__m128 g1s = _mm_sub_ps(_mm_mul_ps(pzs, tys), _mm_mul_ps(_mm_load_ps(buf_warped_y+i), tzs));
		g1s = _mm_mul_ps(g1s,pz2ds);

		 //float g2 = (pz - tz) / (pz*pz*d);
		__m128 g2s = _mm_sub_ps(pzs, tzs);
		g2s = _mm_mul_ps(g2s,pz2ds);

		// calc w_p
		 // float drpdd = gx * g0 + gy * g1;	// ommitting the minus
		__m128 drpdds = _mm_add_ps(
				_mm_mul_ps(g0s, _mm_load_ps(buf_warped_dx+i)),
				_mm_mul_ps(g1s, _mm_load_ps(buf_warped_dy+i)));

		 //float w_p = 1.0f / (sigma_i2 + s * drpdd * drpdd);
		__m128 w_ps = _mm_rcp_ps(_mm_add_ps(sigma_i2s,
				_mm_mul_ps(drpdds,
						_mm_mul_ps(drpdds,
								_mm_mul_ps(depthVarFacs,
										_mm_load_ps(buf_idepthVar+i))))));


		//float w_d = 1.0f / (sv + g2*g2*s);
		__m128 w_ds = _mm_rcp_ps(_mm_add_ps(_mm_load_ps(buf_warped_idepthVar+i),
				_mm_mul_ps(g2s,
						_mm_mul_ps(g2s,
								_mm_mul_ps(depthVarFacs,
										_mm_load_ps(buf_idepthVar+i))))));

		//float weighted_rp = fabs(rp*sqrtf(w_p));
		__m128 weighted_rps = _mm_mul_ps(_mm_load_ps(buf_warped_residual+i),
				_mm_sqrt_ps(w_ps));
		weighted_rps = _mm_max_ps(weighted_rps, _mm_sub_ps(zeros,weighted_rps));

		//float weighted_rd = fabs(rd*sqrtf(w_d));
		__m128 weighted_rds = _mm_mul_ps(_mm_load_ps(buf_residual_d+i),
				_mm_sqrt_ps(w_ds));
		weighted_rds = _mm_max_ps(weighted_rds, _mm_sub_ps(zeros,weighted_rds));

		// depthValid = sv > 0
		__m128 depthValid = _mm_cmplt_ps(zeros, _mm_load_ps(buf_warped_idepthVar+i));	// bitmask 0xFFFFFFFF for idepth valid, 0x000000 otherwise


		// float weighted_abs_res = sv > 0 ? weighted_rd+weighted_rp : weighted_rp;
		__m128 weighted_abs_ress = _mm_add_ps(_mm_and_ps(weighted_rds,depthValid), weighted_rps);

		//float wh = fabs(weighted_abs_res < huber_res ? 1 : huber_res / weighted_abs_res);
		__m128 whs = _mm_cmplt_ps(weighted_abs_ress, huber_ress);	// bitmask 0xFFFFFFFF for 1, 0x000000 for huber_res_ponly / weighted_rp
		whs = _mm_or_ps(
				_mm_and_ps(whs, ones),
				_mm_andnot_ps(whs, _mm_mul_ps(huber_ress, _mm_rcp_ps(weighted_abs_ress))));


		if(i+3 < buf_warped_size)
		{
			//if(sv > 0) sumRes.numTermsD++;
			numTermsD = _mm_add_ps(numTermsD,
					_mm_and_ps(depthValid, ones));

			//if(sv > 0) sumRes.sumResD += wh * w_d * rd*rd;
			sumResD = _mm_add_ps(sumResD,
					_mm_and_ps(depthValid, _mm_mul_ps(whs, _mm_mul_ps(weighted_rds, weighted_rds))));

			// sumRes.sumResP += wh * w_p * rp*rp;
			sumResP = _mm_add_ps(sumResP,
					_mm_mul_ps(whs, _mm_mul_ps(weighted_rps, weighted_rps)));
		}

		//*(buf_weight_p+i) = wh * w_p;
		_mm_store_ps(buf_weight_p+i, _mm_mul_ps(whs, w_ps) );

		//if(sv > 0) *(buf_weight_d+i) = wh * w_d; else *(buf_weight_d+i) = 0;
		_mm_store_ps(buf_weight_d+i, _mm_and_ps(depthValid, _mm_mul_ps(whs, w_ds)) );

	}
	sumRes.sumResP = SSEE(sumResP,0) + SSEE(sumResP,1) + SSEE(sumResP,2) + SSEE(sumResP,3);
	sumRes.numTermsP = (buf_warped_size >> 2) << 2;

	sumRes.sumResD = SSEE(sumResD,0) + SSEE(sumResD,1) + SSEE(sumResD,2) + SSEE(sumResD,3);
	sumRes.numTermsD = SSEE(numTermsD,0) + SSEE(numTermsD,1) + SSEE(numTermsD,2) + SSEE(numTermsD,3);

	sumRes.mean = (sumRes.sumResD + sumRes.sumResP) / (sumRes.numTermsD + sumRes.numTermsP);
	sumRes.meanD = (sumRes.sumResD) / (sumRes.numTermsD);
	sumRes.meanP = (sumRes.sumResP) / (sumRes.numTermsP);

	return sumRes;
}
#endif

#if defined(ENABLE_NEON)
Sim3ResidualStruct SE3DepthTracker::calcSim3WeightsAndResidualNEON(
		const Sim3& referenceToFrame)
{
	return calcSim3WeightsAndResidual(
			referenceToFrame);
}
#endif


SE3DepthResidualStruct SE3DepthTracker::calcSE3DepthWeightsAndResidual(
		const SE3& referenceToFrame)
{
	float tx = referenceToFrame.translation()[0];
	float ty = referenceToFrame.translation()[1];
	float tz = referenceToFrame.translation()[2];

	SE3DepthResidualStruct sumRes;
	memset(&sumRes, 0, sizeof(SE3DepthResidualStruct));


	float sum_rd=0, sum_rp=0, sum_wrd=0, sum_wrp=0, sum_wp=0, sum_wd=0, sum_num_d=0, sum_num_p=0;

	for(int i=0;i<buf_warped_size;i++)
	{
		float px = *(buf_warped_x+i);	// x'
		float py = *(buf_warped_y+i);	// y'
		float pz = *(buf_warped_z+i);	// z'

		float d = *(buf_d+i);	// d

		float rp = *(buf_warped_residual+i); // r_p, (photometric residual)
		float rd = *(buf_residual_d+i);	 // r_d (depth residual)

		float gx = *(buf_warped_dx+i);	// \delta_x I
		float gy = *(buf_warped_dy+i);  // \delta_y I

		float s = settings.var_weight * *(buf_idepthVar+i);	// \sigma_d^2
		float sv = settings.var_weight * *(buf_warped_idepthVar+i);	// \sigma_d^2'


		// calc dw/dd (first 2 components):
		float g0 = (tx * pz - tz * px) / (pz*pz*d);
		float g1 = (ty * pz - tz * py) / (pz*pz*d);
		float g2 = (pz - tz) / (pz*pz*d);

		// calc w_p
		float drpdd = gx * g0 + gy * g1;	// ommitting the minus
		float w_p = 1.0f / (cameraPixelNoise2 + s * drpdd * drpdd);

		float w_d = 1.0f / (sv + g2*g2*s);

		float weighted_rd = fabs(rd*sqrtf(w_d));
		float weighted_rp = fabs(rp*sqrtf(w_p));


		float weighted_abs_res = sv > 0 ? weighted_rd+weighted_rp : weighted_rp;
		float wh = fabs(weighted_abs_res < settings.huber_d ? 1 : settings.huber_d / weighted_abs_res);

		if(sv > 0)
		{
			sumRes.sumResD += wh * w_d * rd*rd;
			sumRes.numTermsD++;
		}

		sumRes.sumResP += wh * w_p * rp*rp;
		sumRes.numTermsP++;

		*(buf_weight_p+i) = wh * w_p;

		if(sv > 0)
			*(buf_weight_d+i) = wh * w_d;
		else
			*(buf_weight_d+i) = 0;

	}

	sumRes.mean = (sumRes.sumResD + sumRes.sumResP) / (sumRes.numTermsD + sumRes.numTermsP);
	sumRes.meanD = (sumRes.sumResD) / (sumRes.numTermsD);
	sumRes.meanP = (sumRes.sumResP) / (sumRes.numTermsP);

	return sumRes;
}



#if defined(ENABLE_SSE)
void SE3DepthTracker::calcSE3DepthLGSSSE(LGS6 &ls)
{
	LGS4 ls4;
	LGS6 ls6;
	ls6.initialize(width*height);
	ls4.initialize(width*height);

	const __m128 zeros = _mm_set1_ps(0.0f);

	for(int i=0;i<buf_warped_size-3;i+=4)
	{
		__m128 val1, val2, val3, val4;

		__m128 J41, J42, J43, J44;
		__m128 J61, J62, J63, J64, J65, J66;


		// redefine pz
		__m128 pz = _mm_load_ps(buf_warped_z+i);
		pz = _mm_rcp_ps(pz);						// pz := 1/z

		//v4[3] = z;
		J44 = pz;

		__m128 gx = _mm_load_ps(buf_warped_dx+i);
		val1 = _mm_mul_ps(pz, gx);			// gx / z => SET [0]
		//v[0] = z*gx;
		J61 = val1;



		__m128 gy = _mm_load_ps(buf_warped_dy+i);
		val1 = _mm_mul_ps(pz, gy);					// gy / z => SET [1]
		//v[1] = z*gy;
		J62 = val1;


		__m128 px = _mm_load_ps(buf_warped_x+i);
		val1 = _mm_mul_ps(px, gy);
		val1 = _mm_mul_ps(val1, pz);	//  px * gy * z
		__m128 py = _mm_load_ps(buf_warped_y+i);
		val2 = _mm_mul_ps(py, gx);
		val2 = _mm_mul_ps(val2, pz);	//  py * gx * z
		val1 = _mm_sub_ps(val1, val2);  // px * gy * z - py * gx * z => SET [5]
		//v[5] = -py * z * gx +  px * z * gy;
		J66 = val1;


		// redefine pz
		pz = _mm_mul_ps(pz,pz); 		// pz := 1/(z*z)

		//v4[0] = z_sqr;
		J41 = pz;

		//v4[1] = z_sqr * py;
		__m128 pypz = _mm_mul_ps(pz, py);
		J42 = pypz;

		//v4[2] = -z_sqr * px;
		__m128 mpxpz = _mm_sub_ps(zeros,_mm_mul_ps(pz, px));
		J43 = mpxpz;



		// will use these for the following calculations a lot.
		val1 = _mm_mul_ps(px, gx);
		val1 = _mm_mul_ps(val1, pz);		// px * z_sqr * gx
		val2 = _mm_mul_ps(py, gy);
		val2 = _mm_mul_ps(val2, pz);		// py * z_sqr * gy


		val3 = _mm_add_ps(val1, val2);
		val3 = _mm_sub_ps(zeros,val3);	//-px * z_sqr * gx -py * z_sqr * gy
		//v[2] = -px * z_sqr * gx -py * z_sqr * gy;	=> SET [2]
		J63 = val3;


		val3 = _mm_mul_ps(val1, py); // px * z_sqr * gx * py
		val4 = _mm_add_ps(gy, val3); // gy + px * z_sqr * gx * py
		val3 = _mm_mul_ps(val2, py); // py * py * z_sqr * gy
		val4 = _mm_add_ps(val3, val4); // gy + px * z_sqr * gx * py + py * py * z_sqr * gy
		val4 = _mm_sub_ps(zeros,val4); //val4 = -val4.
		//v[3] = -px * py * z_sqr * gx +
		//       -py * py * z_sqr * gy +
		//       -gy;		=> SET [3]
		J64 = val4;


		val3 = _mm_mul_ps(val1, px); // px * px * z_sqr * gx
		val4 = _mm_add_ps(gx, val3); // gx + px * px * z_sqr * gx
		val3 = _mm_mul_ps(val2, px); // px * py * z_sqr * gy
		val4 = _mm_add_ps(val4, val3); // gx + px * px * z_sqr * gx + px * py * z_sqr * gy
		//v[4] = px * px * z_sqr * gx +
		//	   px * py * z_sqr * gy +
		//	   gx;				=> SET [4]
		J65 = val4;


		if(i+3<buf_warped_size)
		{
			ls4.updateSSE(J41, J42, J43, J44, _mm_load_ps(buf_residual_d+i), _mm_load_ps(buf_weight_d+i));
			ls6.updateSSE(J61, J62, J63, J64, J65, J66, _mm_load_ps(buf_warped_residual+i), _mm_load_ps(buf_weight_p+i));
		}
		else
		{
			for(int k=0;i+k<buf_warped_size;k++)
			{
				Vector6 v6;
				v6 << SSEE(J61,k),SSEE(J62,k),SSEE(J63,k),SSEE(J64,k),SSEE(J65,k),SSEE(J66,k);
				Vector4 v4;
				v4 << SSEE(J41,k),SSEE(J42,k),SSEE(J43,k),SSEE(J44,k);

				ls4.update(v4, *(buf_residual_d+i+k), *(buf_weight_d+i+k));
				ls6.update(v6, *(buf_warped_residual+i+k), *(buf_weight_p+i+k));
			}
		}
	}

	ls4.finishNoDivide();
	ls6.finishNoDivide();
	ls.initializeFrom(ls6, ls4);


}
#endif

#if defined(ENABLE_NEON)
void SE3DepthTracker::calcSim3LGSNEON(LGS7 &ls7)
{
	calcSim3LGS(ls7);
}
#endif


void SE3DepthTracker::calcSE3DepthLGS(LGS6 &ls)
{
    LGS4 ls4;
    LGS6 ls6;
	ls6.initialize(width*height);
	ls4.initialize(width*height);

	for(int i=0;i<buf_warped_size;i++)
	{
		float px = *(buf_warped_x+i);	// x'
		float py = *(buf_warped_y+i);	// y'
		float pz = *(buf_warped_z+i);	// z'

		float wp = *(buf_weight_p+i);	// wr/wp
		float wd = *(buf_weight_d+i);	// wr/wd

		float rp = *(buf_warped_residual+i); // r_p
		float rd = *(buf_residual_d+i);	 // r_d (depth residual)

		float gx = *(buf_warped_dx+i);	// \delta_x I
		float gy = *(buf_warped_dy+i);  // \delta_y I


		float z = 1.0f / pz;
		float z_sqr = 1.0f / (pz*pz);
		Vector6 v;
		Vector4 v4;
		v[0] = z*gx + 0;
		v[1] = 0 +         z*gy;
		v[2] = (-px * z_sqr) * gx +
			  (-py * z_sqr) * gy;
		v[3] = (-px * py * z_sqr) * gx +
			  (-(1.0 + py * py * z_sqr)) * gy;
		v[4] = (1.0 + px * px * z_sqr) * gx +
			  (px * py * z_sqr) * gy;
		v[5] = (-py * z) * gx +
			  (px * z) * gy;

		// new:
		v4[0] = z_sqr;
		v4[1] = z_sqr * py;
		v4[2] = -z_sqr * px;
		v4[3] = z;

		ls6.update(v, rp, wp);		// Jac = - v
		ls4.update(v4, rd, wd);	// Jac = v4

	}

	ls4.finishNoDivide();
	ls6.finishNoDivide();


	ls.initializeFrom(ls6, ls4);

}

void SE3DepthTracker::calcResidualAndBuffers_debugStart()
{
	if(plotTrackingIterationInfo || saveAllTrackingStagesInternal)
	{
		int other = saveAllTrackingStagesInternal ? 255 : 0;
		fillCvMat(&debugImageResiduals,cv::Vec3b(other,other,255));
		fillCvMat(&debugImageExternalWeights,cv::Vec3b(other,other,255));
		fillCvMat(&debugImageWeights,cv::Vec3b(other,other,255));
		fillCvMat(&debugImageOldImageSource,cv::Vec3b(other,other,255));
		fillCvMat(&debugImageOldImageWarped,cv::Vec3b(other,other,255));
		fillCvMat(&debugImageScaleEstimation,cv::Vec3b(255,other,other));
		fillCvMat(&debugImageDepthResiduals,cv::Vec3b(other,other,255));
	}
}

void SE3DepthTracker::calcResidualAndBuffers_debugFinish(int w)
{
	if(plotTrackingIterationInfo)
	{
		Util::displayImage( "Weights", debugImageWeights );
		Util::displayImage( "second_frame", debugImageSecondFrame );
		Util::displayImage( "Intensities of second_frame at transformed positions", debugImageOldImageSource );
		Util::displayImage( "Intensities of second_frame at pointcloud in first_frame", debugImageOldImageWarped );
		Util::displayImage( "Residuals", debugImageResiduals );
		Util::displayImage( "DepthVar Weights", debugImageExternalWeights );
		Util::displayImage( "Depth Residuals", debugImageDepthResiduals );

		// wait for key and handle it
		bool looping = true;
		while(looping)
		{
			int k = Util::waitKey(1);
			if(k == -1)
			{
				if(autoRunWithinFrame)
					break;
				else
					continue;
			}

			char key = k;
			if(key == ' ')
				looping = false;
			else
				handleKey(k);
		}
	}

	if(saveAllTrackingStagesInternal)
	{
		char charbuf[500];

		snprintf(charbuf,500,"save/%sresidual-%d-%d.png",packagePath.c_str(),w,iterationNumber);
		cv::imwrite(charbuf,debugImageResiduals);

		snprintf(charbuf,500,"save/%swarped-%d-%d.png",packagePath.c_str(),w,iterationNumber);
		cv::imwrite(charbuf,debugImageOldImageWarped);

		snprintf(charbuf,500,"save/%sweights-%d-%d.png",packagePath.c_str(),w,iterationNumber);
		cv::imwrite(charbuf,debugImageWeights);

		printf("saved three images for lvl %d, iteration %d\n",w,iterationNumber);
	}
}
}
