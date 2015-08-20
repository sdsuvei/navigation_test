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

#define M_PI2   2*M_PI
#define DEG2RAD 0.0174533f

//create PID controller
struct PID {
	float Kp;
	float Ki;
	float Kd;

	float error;
	float last_error;

	float perror;
	float ierror;
	float derror;

	float dt;
	float integrator;
};

class angleIntegrator{
   private:
	double imu_yaw;
	sensor_msgs::Imu msg_imu;
	double g_z;
	ros::Publisher pub_imu_;
        ros::Time imu_time_latest, imu_time_prev;
	int buffer[10000];
	int i;
	double angle_limit (double angle) // keep angle within [0;2*pi[
	{
		while (angle >= M_PI2)
			angle -= M_PI2;
		while (angle < 0)
			angle += M_PI2;
		return angle;
	}
   public:
	void newMsgCallback(const msgs::nmea::ConstPtr& msg){
	        if ((msg->type).compare("SFIMU") == 0){
	                if(msg->length == 6){
	                        if(msg->valid == false){
	                                ROS_WARN("IMU checksum did not match, skipping measurement");
	                        } else {
					bool result;
	                                try {
	                                        g_z = boost::lexical_cast<int>(msg->data[5]) * 1/14.375 * DEG2RAD;
											result = true;
	                                }
	                                catch(boost::bad_lexical_cast &) {
	                                        ROS_ERROR("Could not convert Sparkfun Razor 9dof IMU accelerometer and gyro readings");
						result = false;
	                                }

					if(result){
						imu_time_latest = ros::Time::now();
						double dt = (imu_time_latest - imu_time_prev).toSec();
						imu_time_prev = imu_time_latest;

						imu_yaw += (g_z*dt);
	                                        imu_yaw = angle_limit (imu_yaw);
					}
	                        }
        	        } else {
        	                ROS_ERROR("Unexpected number of tokens in $SPIMU message");
        	        }
        	}
	}


	void publishAngle(const ros::TimerEvent& e){

	        ++msg_imu.header.seq;
	        msg_imu.header.frame_id = "imu_link";
	        msg_imu.header.stamp = ros::Time::now();

		// ENU orientation
		msg_imu.orientation_covariance[0] = 0; // As instructed here: http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html
		msg_imu.orientation.x = 0;
		msg_imu.orientation.y = 0;
		msg_imu.orientation.z = imu_yaw;
		msg_imu.orientation.w = 0;

		// acceleration
		msg_imu.linear_acceleration.x = 0;
		msg_imu.linear_acceleration.y = 0;
		msg_imu.linear_acceleration.z = 0;

		// angular rates
		msg_imu.angular_velocity.x = 0;
		msg_imu.angular_velocity.y = 0;
		msg_imu.angular_velocity.z = 0;
		pub_imu_.publish(msg_imu);
	}

	angleIntegrator(ros::Publisher pub){
		pub_imu_ = pub;
	}

};

class Frobit {
private:
	//std::ofstream myfile;
	int count; // counter
	ros::NodeHandle n; 
	std_msgs::Bool deadman_button; //Deadman button message
	std_msgs::Float64 error_msg; //Error message
	double z_axis; 
	double x_axis;
        float v_max;
        float d_max;

	geometry_msgs::TwistStamped twist;
	ros::Subscriber sub;
	ros::Subscriber subScan;
	ros::Subscriber sub_lines;
	ros::Publisher twist_pub_;
	ros::Publisher deadman_pub_;
	ros::Publisher error_pub_;
	PID pid;
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void updateVel(const std_msgs::Float64MultiArray& houghInfo);
	void IMUCallback(const msgs::nmea::ConstPtr& imu);
	int stopCounter;
	int followLeftWall;
	float angelTurned;
public:

	Frobit();
};

