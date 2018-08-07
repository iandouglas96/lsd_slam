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

#define GL_GLEXT_PROTOTYPES 1

#include "KeyFrameDisplay.h"
#include <stdio.h>
#include "settings.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "geometry_msgs/Pose.h"
#include <tf2_eigen/tf2_eigen.h>

#include "ros/package.h"

KeyFrameDisplay::KeyFrameDisplay()
{
	originalInput = 0;
	id = 0;
	vertexBufferIdValid = false;
	glBuffersValid = false;


	camToWorld = Sophus::SE3f();
	width=height=0;

	my_scaledTH = my_absTH = 0;

	totalPoints = displayedPoints = 0;

	//Init stuff to reasonable starting values
	robot_pose.translation() << 0, 0, 0;
    robot_pose.linear() <<  1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;
	tunnel_radius = 0;

	texture_state = 0;
}


KeyFrameDisplay::~KeyFrameDisplay()
{
	if(vertexBufferIdValid)
	{
		glDeleteBuffers(1, &vertexBufferId);
		vertexBufferIdValid = false;
	}

	if(originalInput != 0)
		delete[] originalInput;
}


void KeyFrameDisplay::setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	//std::cout << "set from!\n";
	// copy over campose.
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

	fx = msg->fx;
	fy = msg->fy;
	cx = msg->cx;
	cy = msg->cy;

	cam_intrinsics << fx, 0, cx, 0, fy, cy, 0, 0, 1;

	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;

	width = msg->width;
	height = msg->height;
	id = msg->id;
	time = msg->time;

	//Load in position w.r.t tunnel center
	Eigen::Vector3d pos;
	tf2::convert(msg->tunnel_pose.position, pos);
	Eigen::Quaterniond rot;
	tf2::convert(msg->tunnel_pose.orientation, rot);
	robot_pose.linear() = rot.normalized().toRotationMatrix().cast<float>();
	robot_pose.translation() = pos.cast<float>();

	//std::cout << "vis pos: " << robot_pose.translation() << "\n";
	//std::cout << "vis rot: " << robot_pose.linear() << "\n";

	//Rotate 90 degrees to reconcile coord frames.
	Eigen::Vector3f euler = robot_pose.rotation().eulerAngles(2, 1, 0);
	//std::cout << "vis eul: " << euler << "\n";
	robot_pose.linear() = (Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX())*
						   Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitY())*
						   Eigen::AngleAxisf(euler(1)+M_PI/2, Eigen::Vector3f::UnitZ())).matrix();
	robot_pose.translation() = Eigen::Vector3f(robot_pose.translation()(1), robot_pose.translation()(0), robot_pose.translation()(2));

	tunnel_radius = msg->tunnel_radius;

	

	// copy over image
	if (msg->image.size() == sizeof(float)*width*height) {
		cv::Mat image = cv::Mat(height, width, CV_32F, cv::Scalar(0));
		memcpy(image.data, msg->image.data(), sizeof(float)*width*height);
		image.convertTo(image, CV_8UC1);
		//std::cout << image << "\n";
		setTexture(image);
		//std::cout << "img disp...\n";
	} else {
		//std::cout << "bad image size: " << msg->image.size() << "\n";
	}

	if(originalInput != 0)
		delete[] originalInput;
	originalInput=0;

	if(msg->pointcloud.size() != width*height*sizeof(InputPointDense))
	{
		if(msg->pointcloud.size() != 0)
		{
			printf("WARNING: PC with points, but number of points not right! (is %zu, should be %u*%dx%d=%u)\n",
					msg->pointcloud.size(), sizeof(InputPointDense), width, height, width*height*sizeof(InputPointDense));
		}
	}
	else
	{
		originalInput = new InputPointDense[width*height];
		memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
	}

	glBuffersValid = false;
}

void KeyFrameDisplay::refreshPC()
{
//	minNearSupport = 9;
	bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
			my_absTH == absDepthVarTH &&
			my_minNearSupport == minNearSupport &&
			my_sparsifyFactor == sparsifyFactor;



	if(glBuffersValid && (paramsStillGood || numRefreshedAlready > 10)) return;
	numRefreshedAlready++;

	glBuffersValid = true;


	// delete old vertex buffer
	if(vertexBufferIdValid)
	{
		glDeleteBuffers(1, &vertexBufferId);
		vertexBufferIdValid = false;
	}



	// if there are no vertices, done!
	if(originalInput == 0)
		return;


	// make data
	MyVertex* tmpBuffer = new MyVertex[width*height];

	my_scaledTH =scaledDepthVarTH;
	my_absTH = absDepthVarTH;
	my_scale = 1;
	my_minNearSupport = minNearSupport;
	my_sparsifyFactor = sparsifyFactor;
	// data is directly in ros message, in correct format.
	vertexBufferNumPoints = 0;

	int total = 0, displayed = 0;
	for(int y=1;y<height-1;y++)
		for(int x=1;x<width-1;x++)
		{
			if(originalInput[x+y*width].idepth <= 0) continue;
			total++;


			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;


			/*if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;*/

			if(my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for(int dx=-1;dx<2;dx++)
					for(int dy=-1;dy<2;dy++)
					{
						int idx = x+dx+(y+dy)*width;
						if(originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if(diff*diff < 2*originalInput[x+y*width].idepth_var)
								nearSupport++;
						}
					}

				if(nearSupport < my_minNearSupport)
					continue;
			}

			//printf("%i %i %f\n", x, y, depth);

			tmpBuffer[vertexBufferNumPoints].point[0] = (x*fxi + cxi) * depth;
			tmpBuffer[vertexBufferNumPoints].point[1] = (y*fyi + cyi) * depth;
			tmpBuffer[vertexBufferNumPoints].point[2] = depth;

			tmpBuffer[vertexBufferNumPoints].color[3] = 100;
			tmpBuffer[vertexBufferNumPoints].color[2] = originalInput[x+y*width].color[0];
			tmpBuffer[vertexBufferNumPoints].color[1] = originalInput[x+y*width].color[1];
			tmpBuffer[vertexBufferNumPoints].color[0] = originalInput[x+y*width].color[2];

			vertexBufferNumPoints++;
			displayed++;
		}
	totalPoints = total;
	displayedPoints = displayed;

	// create new ones, static
	vertexBufferId=0;
	glGenBuffers(1, &vertexBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);
	vertexBufferIdValid = true;



	if(!keepInMemory)
	{
		delete[] originalInput;
		originalInput = 0;
	}




	delete[] tmpBuffer;
}

void KeyFrameDisplay::setTexture(cv::Mat &color_channel)
{
    color_channel.convertTo(texture, CV_8UC1);
	texture_mutex.lock();
	cv::cvtColor(color_channel, texture, cv::COLOR_GRAY2RGBA);

	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x) {
			cv::Vec4b & pixel = texture.at<cv::Vec4b>(y, x);
			if (pixel[0] == 0) pixel[3] = 0;
			else pixel[3] = std::max((height/2-imageWidth*(abs(height/2-y)))*255/(height/2),0);
		}

	texture_state = 1;
	texture_mutex.unlock();
}

void KeyFrameDisplay::loadTexture()
{
	if (texture_state == 1) {
		//std::cout << texture << "\n";
		glGenTextures(1, &texture_handle);
		glBindTexture(GL_TEXTURE_2D, texture_handle);

		//Set interpolation technique
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		texture_mutex.lock();
		glTexImage2D(GL_TEXTURE_2D,     // Type of texture
					0,                  // Pyramid level (for mip-mapping) - 0 is the top level
					GL_RGBA,             // Internal colour format to convert to
					texture.cols,       // Image width  
					texture.rows,       // Image height 
					0,                  // Border width in pixels (can either be 1 or 0)
					GL_RGBA,      		// Input image format (OpenCV uses BGR by default)
					GL_UNSIGNED_BYTE,   // Image data type
					texture.ptr());     // The actual image data itself
		texture_mutex.unlock();

		//Turn on texture mapping
		glEnable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		texture_state = 2;

		//std::cout << texture_handle << "\n";

		//cv::imshow("Texture", texture);
		//cv::waitKey(1);
	}
}

//Convert from image to world coordinates IN CAMERA FRAME
Eigen::Vector3f KeyFrameDisplay::calcProjectionCameraFrame(float x, float y)
{
    Eigen::Vector3f hom_pt(x, y, 1);
    hom_pt = cam_intrinsics.inverse()*hom_pt; //put in world coordinates

    Eigen::Vector3f direction(hom_pt(0),hom_pt(1),hom_pt(2));

    //Normalize so we have a unit vector
    direction *= 1/direction.norm();

    return direction;
}

float KeyFrameDisplay::calcDistance(Eigen::Vector3f &ray_direction)
{
    //ROS_INFO("%f, %f, %f, %f", ray_direction.x(), ray_direction.y(), ray_direction.z(), ray_direction.w());

    //Convert to eigen datatypes, which are much faster
    Eigen::Vector3f pos = robot_pose.translation();
    Eigen::Vector3f dir(ray_direction.x(), ray_direction.y(), ray_direction.z());
    dir = robot_pose.linear()*dir;

    //project onto y-z plane (cross sectional plane of tunnel), magnitude doesn't matter
    Eigen::Vector3f proj = Eigen::Vector3f::UnitX().cross(dir.cross(Eigen::Vector3f::UnitX()));
    
    //Assume parametrization x=x0+at, y=y0+bt, z=z0+ct where a^2+b^2+c^2=1
    double under_sqrt = proj(1)*proj(1)*(tunnel_radius*tunnel_radius-pos(2)*pos(2));//a^2*(R^2-y0^2)
    under_sqrt += proj(2)*proj(2)*(tunnel_radius*tunnel_radius-pos(1)*pos(1));//b^2*(R^2-x0^2)
    under_sqrt += 2*proj(1)*proj(2)*pos(1)*pos(2);//2*a*b*x0*y0
    double t = -proj(1)*pos(1)-proj(2)*pos(2);//-a*x0-b*y0
    t += sqrt(under_sqrt);
    t /= proj(1)*proj(1) + proj(2)*proj(2);

    return t;
}

Eigen::Vector2f KeyFrameDisplay::getLeftIntercept(float dist, float angle)
{
    float x = dist*sin(abs(angle));
    float y = dist*cos(abs(angle));
    float slope = -tan(angle);
    if (slope > 0)
        x = width - x;
    float cept = -slope*x + y;
    /*std::cout << "cept: " << cept << "\n";
    std::cout << "x: " << x << "\n";
    std::cout << "y: " << y << "\n";*/
    //special case
    
    if (abs(tan(angle)) < 0.0001) {
        return Eigen::Vector2f(0, y);
    }
    if (abs(slope) < 0.0001) {
        return Eigen::Vector2f(x, 0);
    }

    if (cept <= height && cept >= 0) {
        return Eigen::Vector2f(0, cept);
    } else if (slope < 0) {
        cept = (height - y)/slope + x;
        if (cept > -1 && cept < 0) cept = 0; //If we are just a bit negative, fix rounding error
        if (cept <= width)
            return Eigen::Vector2f(cept, height);
        else {
            printf("ERROR: left image coord out of bounds: %f\n", cept);
            return Eigen::Vector2f(0,0);
        }
    } else if (slope > 0) {
        cept = -y/slope + x;
        if (cept > -1 && cept < 0) cept = 0; //If we are just a bit negative, fix rounding error
        if (cept <= width)
            return Eigen::Vector2f(cept, 0);
        else {
            printf("ERROR: left image coord out of bounds: %f\n", cept);
            return Eigen::Vector2f(0,0);
        }
    }
}

Eigen::Vector2f KeyFrameDisplay::getRightIntercept(float dist, float angle)
{
    float x = dist*sin(abs(angle));
    float y = dist*cos(abs(angle));
    float slope = -tan(angle);
    if (slope > 0)
        x = width - x;
    float cept = slope*(width-x) + y;
    //std::cout << "slope: " << slope << "\n";

    if (abs(tan(angle)) < 0.0001) {
        return Eigen::Vector2f(width, y);
    }
    if (abs(slope) < 0.0001) {
        return Eigen::Vector2f(x, height);
    }

    if (cept <= height && cept >= 0)
        return Eigen::Vector2f(width, cept);
    else if (slope < 0) {
        cept = -y/slope + x;
        if (cept <= width)
            return Eigen::Vector2f(cept, 0);
        else {
            printf("ERROR: right image coord out of bounds: %f\n", cept);
            return Eigen::Vector2f(0,0);
        }
    } else if (slope > 0) {
        cept = (height - y)/slope + x;
        if (cept > -1 && cept < 0) cept = 0; //If we are just a bit negative, fix rounding error
        if (cept <= width)
            return Eigen::Vector2f(cept, height);
        else {
            printf("ERROR: right image coord out of bounds: %f\n", cept);
            return Eigen::Vector2f(0,0);
        }
    }
}

void KeyFrameDisplay::drawCylinderSegment()
{
	if (texture_state  == 0) {
		std::cout << "No image yet\n";
		return;
	}
	loadTexture();

	const float nbSteps = 200.0;

    //Get angle b/w camera and tunnel in axis
    Eigen::Vector3f robot_vec = robot_pose.linear()*Eigen::Vector3f(1, 0, 0);
    float angle;

    //std::cout << "vec:\n" << robot_vec << "\n";

    if (robot_vec(0) != 0) {
        angle = atan(robot_vec(1)/robot_vec(0));
    } else if (robot_vec(1) != 0) {
        angle = M_PI/2;
    } else {
        angle = 0;
    }

    //std::cout << angle << "\n";

    float dist = height*cos(abs(angle)) + (width - height*cos(abs(angle))*sin(abs(angle)))*sin(abs(angle));

    Eigen::Vector2f img_pt;

    /*img_pt = getLeftIntercept(0, angle);
    std::cout << "dist: " << dist << "\n";
    std::cout << "lt pt:\n" << img_pt << "\n";
    img_pt = getRightIntercept(0, angle);
    std::cout << "rt pt:\n" << img_pt << "\n";
    img_pt = getLeftIntercept(dist, angle);
    std::cout << "lb pt:\n" << img_pt << "\n";
    img_pt = getRightIntercept(dist, angle);
    std::cout << "tb pt:\n" << img_pt << "\n";
    img_pt = getLeftIntercept(dist/2, angle);
    std::cout << "lm pt:\n" << img_pt << "\n";
    img_pt = getRightIntercept(dist/2, angle);
    std::cout << "tm pt:\n" << img_pt << "\n";*/


    //Set the image to texture with
	glColor3f(1,1,1);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindTexture(GL_TEXTURE_2D, texture_handle);  
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= nbSteps; ++i) {
        const float ratio = i / nbSteps;
        const float pt = ratio*dist;
        //glColor3f(1.0 - ratio, 0.2f, ratio);
        //Give sequential texture and 3d points to set up mapping equivalencies
        img_pt = getLeftIntercept(pt, angle);
        Eigen::Vector3f global_pt = calcProjectionCameraFrame(img_pt(0), img_pt(1));
        //glNormal3f(0, -global_pt.y(), -global_pt.z());
        global_pt *= calcDistance(global_pt);
        glTexCoord2f(img_pt(0)/width, img_pt(1)/height);
		global_pt += camToWorld.translation(); //Translate globally
        glVertex3f(global_pt.x(), global_pt.y(), global_pt.z());
        
        img_pt = getRightIntercept(pt, angle);
        global_pt = calcProjectionCameraFrame(img_pt(0), img_pt(1));
        global_pt *= calcDistance(global_pt);
        //glNormal3f(0, -global_pt.y(), -global_pt.z());
        glTexCoord2f(img_pt(0)/width, img_pt(1)/height);
		global_pt += camToWorld.translation(); //Translate globally
        glVertex3f(global_pt.x(), global_pt.y(), global_pt.z());
    }
    glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);
}

void KeyFrameDisplay::drawCam(float lineWidth, float* color)
{
	if(width == 0)
		return;


	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix();
		//std::cout << m << "\n";
		glMultMatrixf((GLfloat*)m.data());

		if(color == 0)
			glColor3f(1,0,0);
		else
			glColor3f(color[0],color[1],color[2]);

		glLineWidth(lineWidth);
		glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);

		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);

		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);

		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

		glEnd();
	glPopMatrix();
}

int KeyFrameDisplay::flushPC(std::ofstream* f)
{

	MyVertex* tmpBuffer = new MyVertex[width*height];
	int num = 0;
	for(int y=1;y<height-1;y++)
		for(int x=1;x<width-1;x++)
		{
			if(originalInput[x+y*width].idepth <= 0) continue;

			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;

			if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;

			if(my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for(int dx=-1;dx<2;dx++)
					for(int dy=-1;dy<2;dy++)
					{
						int idx = x+dx+(y+dy)*width;
						if(originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if(diff*diff < 2*originalInput[x+y*width].idepth_var)
								nearSupport++;
						}
					}

				if(nearSupport < my_minNearSupport)
					continue;
			}


			Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
			tmpBuffer[num].point[0] = pt[0];
			tmpBuffer[num].point[1] = pt[1];
			tmpBuffer[num].point[2] = pt[2];



			tmpBuffer[num].color[3] = 100;
			tmpBuffer[num].color[2] = originalInput[x+y*width].color[0];
			tmpBuffer[num].color[1] = originalInput[x+y*width].color[1];
			tmpBuffer[num].color[0] = originalInput[x+y*width].color[2];

			num++;
		}




	for(int i=0;i<num;i++)
	{
		f->write((const char *)tmpBuffer[i].point,3*sizeof(float));
		float color = tmpBuffer[i].color[0] / 255.0;
		f->write((const char *)&color,sizeof(float));
	}
	//	*f << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " " << tmpBuffer[i].point[2] << " " << (tmpBuffer[i].color[0] / 255.0) << "\n";

	delete tmpBuffer;

	printf("Done flushing frame %d (%d points)!\n", this->id, num);
	return num;
}

void KeyFrameDisplay::drawPC(float pointSize, float alpha)
{
	refreshPC();

	if(!vertexBufferIdValid)
	{
		return;
	}

	GLfloat LightColor[] = {1, 1, 1, 1};
	if(alpha < 1)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		LightColor[0] = LightColor[1] = 0;
		glEnable(GL_LIGHTING);
		glDisable(GL_LIGHT1);

		glLightfv (GL_LIGHT0, GL_AMBIENT, LightColor);
	}
	else
	{
		glDisable(GL_LIGHTING);
	}


	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix();
		glMultMatrixf((GLfloat*)m.data());

		glPointSize(pointSize);

		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);

		glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);

		glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();




	if(alpha < 1)
	{
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);
		LightColor[2] = LightColor[1] = LightColor[0] = 1;
		glLightfv (GL_LIGHT0, GL_AMBIENT_AND_DIFFUSE, LightColor);
	}
}

