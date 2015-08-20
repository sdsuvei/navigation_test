#include "navigation.h"
#include <sensor_msgs/LaserScan.h>
#include <sstream>

#define PI 3.14159265359
#define RAD2DEG 180/PI
#define ANGLE 20*(PI/180)

std_msgs::Float64MultiArray hi;
obstacle_detection::boundingboxes b_boxes;

// constructor for the Frobit
Frobit::Frobit(){	
	stopCounter=0;
	deadman_button.data = false;
	z_axis = 0;
	x_axis = 0;
    v_max = 0.3;
    d_max = 4;

	sub_BB = n.subscribe("/obstacle_detection/boundingboxes", 5, &Frobit::bboxesCallback, this);
	sub_lines = n.subscribe("/measHoughKalman", 5, &Frobit::houghCallback,this);
	twist_pub_ = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel2", 1000);
	deadman_pub_ = n.advertise<std_msgs::Bool>("/fmCommand/deadman", 1000);
	error_pub_ = n.advertise<std_msgs::Float64>("/frobyte/error", 1000);

}

void Frobit::houghCallback(const std_msgs::Float64MultiArray& houghInfo){
	hi = houghInfo;	
	updateVel(hi,b_boxes);
}

void Frobit::bboxesCallback(const obstacle_detection::boundingboxes& bboxes){
	b_boxes = bboxes;	
	updateVel(hi,b_boxes);
}

void Frobit::updateVel(const std_msgs::Float64MultiArray houghInfo, const obstacle_detection::boundingboxes bboxes){
	//get bounding boxes
	float Start_x = 0;
	float Start_y = 0;	
	float Width_x = 0;
	float Width_y = 0;
	float distance = 99;
	// search through bounding boxes list, delete all the ones that are behind the robot (x is negative) and save the one that is closest to the robot (smallest x)
	for (size_t i=0;i<bboxes.boundingboxes.size();i++)
	{
		if(bboxes.boundingboxes[i].start.x>0 && bboxes.boundingboxes[i].start.x < distance)
		{
			distance = bboxes.boundingboxes[i].start.x;
			Start_x = bboxes.boundingboxes[i].start.x;
			Start_y = bboxes.boundingboxes[i].start.y;	
			Width_x = bboxes.boundingboxes[i].width.x;
			Width_x = bboxes.boundingboxes[i].width.y;
		}

	}
		
	twist.header.stamp = ros::Time::now();
	twist.twist.angular.z = z_axis;

	// Get values from the hough2line node (theta,r1,r2)
	float theta = houghInfo.data[0]; // theta angle
	float r1_ = houghInfo.data[1]; //left line
	float r2_ = houghInfo.data[2]; //right line
	
	// check that r1_ is always smallest and on the left side
	if (r2_ < r1_)
	{
		float tmp = r1_;
		r1_ = r2_;
		r2_ = tmp;
	}

	// compute distances to the two lines and the desired "center lane"
	float r_dist_ = r1_+r2_;  
	float center_range = fabs(r1_)+fabs(r2_);  
	

	// compute the angle error	
	double err_ = 0;
	if(fabs(r_dist_)>center_range/7)
	{
		err_ = 0.35*r_dist_;
	}
	// adjust the angle with the error
	twist.twist.angular.z = -0.4*theta*DEG2RAD+err_;
	
	//check if the BBox is close to the robot	
	if(Start_x < d_max)
	{	
		if(Start_y > 0)
		{
			r1_ = -Start_y; // because r1_ should be negative
		}
		else
		{
			r2_ = fabs(Start_y+Width_y);
		}
	}

	// handle the velocity, in case robot is running close to an obstacle 
	float R=std::min(fabs(r1_),fabs(r2_)); //smallest r distance

	// determine the frontal distance to the obstacle	
	float phi = fabs(90-fabs(theta))*DEG2RAD;
	float front_dist_ = R/cos(phi);

	// change the velocity
	twist.twist.linear.x = std::min((float)fabs(front_dist_/d_max), 1.0f)*v_max;	
	//std::cout << "theta: "<< theta <<  " and phi: = " << phi << std::endl;
	//std::cout << "err: "<< err_ << std::endl;
	//std::cout << "r1: "<< r1_ <<  " and r2: = " << r2_ << std::endl;
	//std::cout << "R: "<< R << std::endl;
	//std::cout << front_dist_ <<  " and division = " << front_dist_/d_max << std::endl;
	//twist.twist.linear.x = 0.2; //constant speed
	// send command to motors
	twist_pub_.publish(twist);
	//deadman_button.data = true;	
	//deadman_pub_.publish(deadman_button);
}
int main(int argc, char** argv){	
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle nh;
	
	Frobit *frobo = new Frobit();
    ros::Rate loop_rate(10); // publish 10 messages per second
	// while (ros::ok()){			
		//frobo->updateVel(); //activate motors
		ros::spin();
		//loop_rate.sleep();
        //}*/
 return 0;
}
