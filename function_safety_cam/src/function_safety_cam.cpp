//============================================================================
// Name        : PedestrianDetection.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

// Setting up: C++11
// http://stackoverflow.com/questions/9131763/eclipse-cdt-c11-c0x-support
// Setting up opencv:
// http://docs.opencv.org/doc/tutorials/introduction/linux_eclipse/linux_eclipse.html
// Matio is required to read and write mat-files.
// http://sourceforge.const double PI  =3.141592653589793238463;net/projects/matio/

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <algorithm>
#include <vector>
//#include "PedestrianDetector.hpp"
#include <dirent.h>
#include <string.h>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

class MyNode {
public:
	MyNode() :
			nh("~"), it(nh) {
		//ROS_ERROR("NodeBeingCreated");
		//nh.param<double>("FOV_verticalDeg",FOV_verticalDeg,47.0);
		//nh.param<double>("FOV_horizontal",FOV_horizontal,83.0);
		//nh.param<double>("angleTiltDegrees",angleTiltDegrees,7.0);
		//nh.param<double>("cameraHeight",cameraHeight,1.9);
		//nh.param<double>("imageResize",imageResize,0.5);

		//nh.param<std::string>("model_dir",model_dir,"model");
		//ROS_ERROR("%s\n",model_dir.c_str());
		//cam_pub = it.advertiseCamera("imageWithBBox", 1);

		//cinfor_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh, "test", ""));

		//array_pub = nh.advertise<std_msgs::Float64MultiArray>("DistanceAngle", 1);
		//array_pub2 = nh.advertise<std_msgs::Float64MultiArray>("BBox", 1);
		//cam_sub = it.subscribeCamera("image_raw", 1, &MyNode::onImage, this);
		cam_sub = it.subscribeCamera("/usb_cam/image_raw", 1, &MyNode::onImage, this);
		


	}
	;

	~MyNode() {

	}
	;

	void onImage(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& p) {
		// do all the stuff here
		//ROS_ERROR("GOT Image");
		//convert  image to opencv
		//ROS_ERROR("ImageHasBeenReceived"); 
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		//resize(cv_ptr->image, image, Size(), imageResize, imageResize);

	}
private:
	cv::Mat image;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::CameraSubscriber cam_sub;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfor_;


};

int main(int argc, char** argv) {

	ros::init(argc, argv, "function_safety_cam");

	MyNode node;

	ros::spin();


	return 0;
}

