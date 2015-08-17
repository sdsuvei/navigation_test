

#include <iostream>
#include <sstream>
#include <dirent.h>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/rawdata.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

union four_bytes
  {
    float floatV;
    uint8_t  bytes[4];
  };

int ring_no;
std::string pointcloud_in,pointcloud_out;
int n;
ros::Publisher output_;



void rawCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
{
	int NPackets = laserCloud->data.size();
	int n;
	int ring;

	sensor_msgs::PointCloud cloud; //create empty pointcloud

	sensor_msgs::ChannelFloat32 intensities;
	sensor_msgs::ChannelFloat32 index;

	index.name = "index";
	intensities.name = "intensities";
	n = 0;

	for (int p=0;p<NPackets;p+=32) // jump in steps of 32
	{
		ring = laserCloud->data[p+20]+laserCloud->data[p+21]*256; // (read the ring number)

		if (ring == ring_no)
		{
			n++;
			//laserCloudRing->
			//velodyne_rawdata::VPoint test;
			geometry_msgs::Point32 point;
			union four_bytes tmp;

			// x coordinate
			tmp.bytes[0] = laserCloud->data[p];
			tmp.bytes[1] = laserCloud->data[p+1];
			tmp.bytes[2] = laserCloud->data[p+2];
			tmp.bytes[3] = laserCloud->data[p+3];
			point.x = tmp.floatV;

			// y coordinate
			tmp.bytes[0] = laserCloud->data[p+4];
			tmp.bytes[1] = laserCloud->data[p+5];
			tmp.bytes[2] = laserCloud->data[p+6];
			tmp.bytes[3] = laserCloud->data[p+7];
			point.y = tmp.floatV;

			// z coordinate
			tmp.bytes[0] = laserCloud->data[p+8];
			tmp.bytes[1] = laserCloud->data[p+9];
			tmp.bytes[2] = laserCloud->data[p+10];
			tmp.bytes[3] = laserCloud->data[p+11];
			point.z = tmp.floatV;

			// intensity
			tmp.bytes[0] = laserCloud->data[p+16];
			tmp.bytes[1] = laserCloud->data[p+17];
			tmp.bytes[2] = laserCloud->data[p+18];
			tmp.bytes[3] = laserCloud->data[p+19];
			intensities.values.push_back(tmp.floatV);

			// index (increments for each point)
			index.values.push_back(n);

			cloud.points.push_back(point);
		}
	}

	cloud.channels.push_back(intensities);
	cloud.channels.push_back(index);

	cloud.header.frame_id = "/laser";
	cloud.header.stamp = laserCloud->header.stamp;
	cloud.header.seq = laserCloud->header.seq;

	output_.publish(cloud);

//	for (int fn=0;fn<laserCloud->fields.size();fn++)
//	{
//		ROS_INFO("field name %d = %s - offset: %d, datatype: %d, count: %d",fn,laserCloud->fields[fn].name.c_str(),laserCloud->fields[fn].offset,laserCloud->fields[fn].datatype,laserCloud->fields[fn].count);
//	}
//
//	ROS_INFO("number of packets = %d",NPackets);


	//viewer->spin();
}

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "velodyne_node");
	ros::NodeHandle nh;

	nh.param<int>("ring", ring_no, 15); // the ring/laser to use [0...31]
	nh.param<std::string>("pointcloud_in",pointcloud_in,"/velodyne_points");
	nh.param<std::string>("pointcloud_out",pointcloud_out,"/pointcloud_out");

	ros::Subscriber subRawPointCloud = nh.subscribe<sensor_msgs::PointCloud2>
	(pointcloud_in, 2, rawCloudHandler);
	
	output_ = nh.advertise<sensor_msgs::PointCloud>(pointcloud_out, 10);

	ros::spin();

	return (0);
}
