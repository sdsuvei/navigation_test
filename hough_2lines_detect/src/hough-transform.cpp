//============================================================================
// Name        : Hough.cpp
// Author      : Exchizz
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "hough-transform.h"

using namespace std;

int y(int x){
	int retval = 5*x;
	return retval;
}


void HoughTransform::SetParams(int angle_res, int height, double r_res, double min_row_distance,double max_point_distance){
	this->tRes = angle_res;
	this->w = 180/tRes;
	this->h = height;
	this->min_row_distance = min_row_distance;
	this->max_point_distance = max_point_distance;
	this->RRESOLUTION = r_res;
}

HoughTransform::HoughTransform(){

	x1 = y1 = x2 = y2 = 0;

	max.current = 0;
	max.r = 0;
	max.t = 0;
}

OutputLines HoughTransform::Transform2(const sensor_msgs::PointCloud::ConstPtr & cloud_in, ros::Publisher* hough_img_){

	OutputLines returnVal;
	returnVal.line1.x1 = -1;
	returnVal.line1.y1 = -1;
	returnVal.line1.x2 = -2;
	returnVal.line1.y1 = -2;
	returnVal.line2.x1 = -1;
	returnVal.line2.y1 = -2;
	returnVal.line2.x2 = -2;
	returnVal.line2.y1 = -3;

	if(w > 180)
	{		
		std::cout << "ERROR w is been set too big: " << w << ", decrease! w can max be 180." << std::endl;
		return returnVal;
	} else if(w < 1)
	{
		std::cout << "ERROR w is been set too small: " << w << ", increase! w can min be 1." << std::endl;
		return returnVal;
	}

	std::vector<std::vector<int> > houghMatrix(h, std::vector<int>(w));	 //initialisation of the Hugh Matrix

	//Hough Matrix Generation
	for(int i = 0; i < cloud_in->points.size(); i++)//iterate through the data
	{
		//count++;
		for(int t = 0; t < w; t++){
			//intert requirements on the distance of the line.
			double r = ( ((double)cloud_in->points[i].x) * cos((double)t * tRes * DEG2RAD)) + (((double)cloud_in->points[i].y) * sin((double)t * tRes * DEG2RAD));
			double distanceSquared = std::pow(cloud_in->points[i].x,2) + std::pow(cloud_in->points[i].y,2);
			if (distanceSquared < max_point_distance*max_point_distance) //&& cloud_in->points[i].x > 0 && cloud_in->points[i].y > 0)
			{
				double rtmp = r/RRESOLUTION;
				if(std::abs(rtmp) <= (double)h/2)
				{
					//Increment Hough space
					houghMatrix[std::floor<int>(rtmp)+std::floor<int>(h/2)][t]++;
				}
			}
		}
	}

	// Publish hough image
	cv::Mat houghImg = cv::Mat(h, w, CV_16UC1);

	// Calculate maximum hough value
	double maxV = 0;
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			//ROS_INFO("i = %d, j = %d",i,j);
			if ((double)houghMatrix[i][j] > maxV)
				maxV = (double)houghMatrix[i][j];
		}
	}

	// Fill in pixel values and normalize image
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			houghImg.at<uint16_t>(i,j) = (uint16_t)((double)houghMatrix[i][j]/maxV*(double)std::numeric_limits<uint16_t>::max());
		}
	}

	//		double minVal;
	//		double maxVal;
	//		cv::Point minLoc;
	//		cv::Point maxLoc;
	//		cv::minMaxLoc( houghImg, &minVal, &maxVal, &minLoc, &maxLoc );
	//		max.t = (double)maxLoc.x;
	//		max.r = (double)maxLoc.y;

	sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::TYPE_16UC1, houghImg).toImageMsg();
	msg_out->header.stamp = ros::Time::now();


	hough_img_->publish(msg_out);

	// Evaluating parallel lines
	std::vector<int> DiscardedRs;
	std::vector<int> squaredValues(w);

	//iterate through Hough matrix to Find interesting Rows
	for(int t = 0; t < w; t++)
	{
		squaredValues[t] = 0;
	}

	for(int t = 0; t < w; t++)
	{
		for(int r = 0; r < h; r++)
		{
			squaredValues[t] += houghMatrix[r][t]*houghMatrix[r][t];
		}
	}

	// Find the biggest angle
	max.current = 0;
	max.t = 0;
	for(int t = 0; t < w; t++)
	{
		if(max.current < squaredValues[t]){
			max.current = squaredValues[t];
			max.t = t;
		}
	}
	max.current = 0;
	max.r = 0;
	bool linesFound = true;
	for(int r = 0; r < h; r++)
	{
		if(max.current < houghMatrix[r][max.t]){
			max.current = houghMatrix[r][max.t];
			max.r = r;
		}
	}

	max.r_second = 0;
	max.current = 0;
	double rMinDist = min_row_distance/RRESOLUTION;
	for (int r=0;r < max.r-rMinDist;r++)
	{
		if(max.current < houghMatrix[r][max.t]){
			max.current = houghMatrix[r][max.t];
			max.r_second = r;
		}
	}
	for (int r=max.r+rMinDist;r < h;r++)
	{
		if(max.current < houghMatrix[r][max.t]){
			max.current = houghMatrix[r][max.t];
			max.r_second = r;
		}
	}

	//ROS_INFO("t = %f, r = %f, minVal = %f, maxVal = %f, %d",max.t,max.r,minVal,maxVal, houghImg.at<uint16_t>(0,0));

	// Handle the first line (found as the maximum of the Hough image)
	y1 = -1000;
	x1 = ((double)(max.r-(double)std::floor<int>(h/2))*RRESOLUTION - (double)y1  * sin(max.t*tRes * DEG2RAD)) / cos(max.t*tRes * DEG2RAD);
	y2 = 1000;
	x2 = ((double)(max.r-(double)std::floor<int>(h/2))*RRESOLUTION - (double)y2  * sin(max.t*tRes * DEG2RAD)) / cos(max.t*tRes * DEG2RAD);

	Line line;
	line.x1 = x1;
	line.y1 = y1;
	line.x2 = x2;
	line.y2 = y2;

	returnVal.line1 = line;

	// Handle the second line (found as the next maximum along the column)
	y1 = -1000;
	x1 = ((double)(max.r_second-(double)std::floor<int>(h/2))*RRESOLUTION - (double)y1  * sin(max.t*tRes * DEG2RAD)) / cos(max.t*tRes * DEG2RAD);
	y2 = 1000;
	x2 = ((double)(max.r_second-(double)std::floor<int>(h/2))*RRESOLUTION - (double)y2  * sin(max.t*tRes * DEG2RAD)) / cos(max.t*tRes * DEG2RAD);

	line.x1 = x1;
	line.y1 = y1;
	line.x2 = x2;
	line.y2 = y2;

	returnVal.line2 = line;


	returnVal.theta = 90-max.t*tRes;
	returnVal.r1 = (double)(max.r-(double)std::floor<int>(h/2))*RRESOLUTION;
	returnVal.r2 = (double)(max.r_second-(double)std::floor<int>(h/2))*RRESOLUTION;
	returnVal.variance = (double)(houghMatrix[max.r][max.t]+houghMatrix[max.r_second][max.t])/((double)cloud_in->points.size());

	return returnVal;
}

