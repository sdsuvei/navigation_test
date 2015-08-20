// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <msgs/nmea.h>
#include <fstream>
#include <math.h>
#include <algorithm>  
#include <stdlib.h>
#include <std_msgs/Float64MultiArray.h>
#include <obstacle_detection/boundingbox.h>
#include <obstacle_detection/boundingboxes.h>

#define M_PI2   2*M_PI
#define DEG2RAD 0.0174533f

class Frobit {
private:
	// variables
	int count; 
	ros::NodeHandle n; 
	std_msgs::Bool deadman_button; //Deadman button message

	double z_axis; 
	double x_axis;
    float v_max;
    float d_max;
    int stopCounter;
	geometry_msgs::TwistStamped twist; // cmd_vel message type
	//ros initializers
	ros::Subscriber sub_BB;
	ros::Subscriber sub_lines;
	ros::Publisher twist_pub_;
	ros::Publisher deadman_pub_;
	ros::Publisher error_pub_;

	//callbacks
	void houghCallback(const std_msgs::Float64MultiArray& houghInfo);
	void bboxesCallback(const obstacle_detection::boundingboxes& bboxes);
	
	// controller function
	void updateVel(const std_msgs::Float64MultiArray houghInfo,const obstacle_detection::boundingboxes bboxes);

public:

	Frobit();
};

