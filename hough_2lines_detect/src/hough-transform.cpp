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


void HoughTransform::SetSize(int tres, int h){
	this->tRes = tres;
	this->w = 180/tRes;
	this->h = h;
	//hough_h = ((sqrt(2.0) * (double)(h>w?h:w)) / 2.0); //maximum height for the accumulator
	//_accu_h = hough_h * 2.0; // -r -> +r
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
				//Using Center
				//double r = ( ((double)x - x_center) * cos((double)t * DEG2RAD)) + (((double)y - y_center) * sin((double)t * DEG2RAD));
				//Not using center

				//intert requirements on the distance of the line.				
				double r = ( ((double)cloud_in->points[i].x) * cos((double)t * tRes * DEG2RAD)) + (((double)cloud_in->points[i].y) * sin((double)t * tRes * DEG2RAD));
				double distanceSquared = std::pow(cloud_in->points[i].x,2) + std::pow(cloud_in->points[i].y,2);
				if (distanceSquared < 2*2) //&& cloud_in->points[i].x > 0 && cloud_in->points[i].y > 0)
				{	
					//std::cout << "Is it 10o in first iteration ?: " << houghMatrix.size() << std::endl;
					double rtmp = r/RRESOLUTION;
					if(std::abs(rtmp) <= (double)h/2)
					{
	//					std::cout<< "Resizing Matrix before: " << houghMatrix.size();
	//					houghMatrix.resize(houghMatrix.size()*2);// resize outer matrix

						//for (int i = 0; i < n; ++i)// resize inner matrix
						//	houghMatrix[i].resize(m);

						//Increment Hough space
						houghMatrix[std::floor<int>(rtmp)+std::floor<int>(h/2)][t]++;
					}
				}
			}
		}

		// Publish hough image
		//cv::Mat houghImg = cv::Mat(houghMatrix);
		cv::Mat houghImg = cv::Mat(h, w, CV_16UC1);

		ROS_INFO("test1");

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

		ROS_INFO("test2");

		ROS_INFO("test1 = %f",maxV);

		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{

				houghImg.at<uint16_t>(i,j) = (uint16_t)((double)houghMatrix[i][j]/maxV*(double)std::numeric_limits<uint16_t>::max());
				//houghImg.setTo(257);
				//houghImg.set
				//houghImg[i][j] = 1;
			}
		}

		ROS_INFO("test3");

		double minVal;
		double maxVal;
		cv::Point minLoc;
		cv::Point maxLoc;

		cv::minMaxLoc( houghImg, &minVal, &maxVal, &minLoc, &maxLoc );

		max.t = (double)maxLoc.x;
		max.r = (double)maxLoc.y;



//		cv_bridge::CvImage out_msg;
//
//		out_msg.header.frame_id = "/hough"; // Same timestamp and tf frame as input image
//		out_msg.header.stamp = ros::Time::now();
//		out_msg.header.seq = 1;
//		out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
//		out_msg.image    = houghImg;

		sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::TYPE_16UC1, houghImg).toImageMsg();
		msg_out->header.stamp = ros::Time::now();


//		cv_bridge::CvImagePtr cv_ptr;
//		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		//cv_ptr = cv_bridge::toCvCopy(houghImg, sensor_msgs::image_encodings::BGR8);

		hough_img_->publish(msg_out);

		// Evaluating parallel lines
//		bool parallelLinesFound = false;
//		max.current = 0;
//		max.t = 0;
//		max.r = 0;
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
//
//
//		//Test Find the biggest angle
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
//		max.r_second = 0;
		bool linesFound = true;
//		std::vector<int> oldr;
		for(int r = 0; r < h; r++)
		{
			if(max.current < houghMatrix[r][max.t]){
				max.current = houghMatrix[r][max.t];
				max.r = r;
//				//Imposing that there has to be a certain distance between the rows.
///*oldr.push_back(max.r);
//std::cout << "line distance = " << (((double)max.r-std::floor<int>(h/2))-((double)oldr.front()-std::floor<int>(h/2)))*RRESOLUTION << std::endl;
//std::cout << std::endl;
//
//				max.r = r;
//				while((((double)max.r-std::floor<int>(h/2))-((double)oldr.front()-std::floor<int>(h/2)))*RRESOLUTION > 0.5 && oldr.size() > 0)
//				{
//
//					max.r_second = oldr.front();
//    					oldr.erase(oldr.begin());
//
//					linesFound = true;
//
//				}
//std::cout << "r = " << max.r << " and the second biggest: r_second = " << max.r_second << std::endl;*/
//			//ros::Duration(1).sleep();
//
			}

		}

		max.r_second = 0;
		max.current = 0;
		double rMinDist = 0.5/RRESOLUTION;
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

		//Find lines
//		while(parallelLinesFound)
//		{
//			//iterate through Hough matrix
//			for(int t = 0; t < houghMatrix[0].size(); t++){
//				for(int r = 0; r < houghMatrix.size(); r++){
//					if(max.current < accumulator[r*180+t]){
//						max.current = accumulator[r*180+t];
//						max.t = t;
//						max.r = r;
//					}
//				}
//			}
//			parallelLinesFound= true;
//		}

		//Output Generation

		//
		//OBS!!!
		//
		//Assumes that 0 is along the x-axis when y is 0!!!
		/*if((max.t >=45 && max.t <= 135) || (max.t >=225 && max.t <= 315)){
			//y = (r - x cos(t)) / sin(t);
			x1 = 1;
//(double)(max.r-(h/2))*RRESOLUTION
			y1 = (2-x1*cos(max.t * DEG2RAD))/sin(max.t * DEG2RAD);
			x2 = 10;
			y2 = (2-x2*cos(max.t * DEG2RAD))/sin(max.t * DEG2RAD);
		} else {*/
			//x = (r - y sin(t)) / cos(t);
		if(linesFound)
		{
			//std::cout<< "size of w = " << w << std::endl;
			//std::cout << "Max.r = " << max.r << " Max.t = " << max.t  << " Max.r_second = " << max.r_second << std::endl;
			//std::cout << "sin = " << sin((double)max.t * DEG2RAD) << " cos = " << cos((double)max.t * DEG2RAD) << std::endl;
			//ros::Duration(0.5).sleep();
			ROS_INFO("t = %f, r = %f, minVal = %f, maxVal = %f, %d",max.t,max.r,minVal,maxVal, houghImg.at<uint16_t>(0,0));

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


			y1 = -1000;
			x1 = ((double)(max.r_second-(double)std::floor<int>(h/2))*RRESOLUTION - (double)y1  * sin(max.t*tRes * DEG2RAD)) / cos(max.t*tRes * DEG2RAD);
			y2 = 1000;
			x2 = ((double)(max.r_second-(double)std::floor<int>(h/2))*RRESOLUTION - (double)y2  * sin(max.t*tRes * DEG2RAD)) / cos(max.t*tRes * DEG2RAD);

			line.x1 = x1;
			line.y1 = y1;
			line.x2 = x2;
			line.y2 = y2;

			returnVal.line2 = line;

//			y1 = -h*RRESOLUTION;
//			x1 = ((double)(max.r_second-std::floor<int>(h/2))*RRESOLUTION - (double)y1  * sin((double)max.t*tRes * DEG2RAD)) / cos((double)max.t*tRes * DEG2RAD);
//			y2 = h*RRESOLUTION;
//			x2 = ((double)(max.r_second-std::floor<int>(h/2))*RRESOLUTION - (double)y2  * sin((double)max.t*tRes * DEG2RAD)) / cos((double)max.t*tRes * DEG2RAD);
//			//}
//			line.x1 = x1;
//			line.y1 = y1;
//			line.x2 = x2;
//			line.y2 = y2;
//			returnVal.line2 = line;
			//ros::Duration(0.5).sleep();
			return returnVal;
		}
	return returnVal;
}

/*
int main() {
	int x1,y1,x2,y2;

	const int w = 70;
	const int h = y(w);


	std::vector<std::vector<int> > img;
	img.reserve(w);
	for(int i = 0; i < w; ++i){
		std::vector<int> temp(h);
		img.push_back(temp);
	}

	for(int x = 0; x < w; ++x){
		int y_ = y(x);
		std::cout << "x: " << x << " y: " << y_ << "\n";
		//making binary image - pixel = 1 else 0
		img[x][y_] = 1;
		//std::cout << " value from vector: "<< img[x][y_];
	}


	HoughTransform houghtransform(w, h);
	Line result = houghtransform.Transform(img);

	x1 = result.x1;
	y1 = result.y1;
	x2 = result.x2;
	y2 = result.y2;

	std::cout << "x1: " << x1 << " y1: " << y1 << "\n";
	std::cout << "x2: " << x2 << " y2: " << y2 << "\n";
	//cout << "x1: " << x1 << " y1: " << y1 << "\nx2: " << x2 << " y2: " << y2 << "\n";
	if((x2-x1) != 0 ){
		double a = (y2-y1)/(x2-x1);
		std::cout << "a: " << a << "\n";
	}else {
		cout << "Divide by 0\n";
	}
	cout << "Hello World" << endl; // prints !!!Hello World!!!
	return 0;
}

*/
