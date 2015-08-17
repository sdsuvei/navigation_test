

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
int n;
ros::Publisher output_;



void rawCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
{
//	ROS_INFO("received point cloud");

	int NPackets = laserCloud->data.size();

	int n;

	//double x,y,z,intensity;
	int ring;
	//int baseIndex;

	//const sensor_msgs::PointCloud2ConstPtr& laserCloudRing;
	sensor_msgs::PointCloud cloud; //create empty pointcloud

	// allocate a point cloud with same time and frame ID as raw data
//	    velodyne_rawdata::VPointCloud::Ptr
//	      outMsg(new velodyne_rawdata::VPointCloud());
//	    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
//	    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
//	    outMsg->header.frame_id = scanMsg->header.frame_id;
//	    outMsg->height = 1;

	sensor_msgs::ChannelFloat32 intensities;
	sensor_msgs::ChannelFloat32 index;

	index.name = "index";
	intensities.name = "intensities";
	n = 0;

	for (int p=0;p<NPackets;p+=32) // jump in steps of 32
	{
		ring = laserCloud->data[p+20]+laserCloud->data[p+21]*256; // (read the ring number)
		//ROS_INFO("ring number = %d",ring);



		if (ring == ring_no)
		{
			n++;
			//laserCloudRing->
			//velodyne_rawdata::VPoint test;
			geometry_msgs::Point32 point;
			union four_bytes tmp;
			tmp.bytes[0] = laserCloud->data[p];
			tmp.bytes[1] = laserCloud->data[p+1];
			tmp.bytes[2] = laserCloud->data[p+2];
			tmp.bytes[3] = laserCloud->data[p+3];
			point.x = tmp.floatV;

			tmp.bytes[0] = laserCloud->data[p+4];
			tmp.bytes[1] = laserCloud->data[p+5];
			tmp.bytes[2] = laserCloud->data[p+6];
			tmp.bytes[3] = laserCloud->data[p+7];
			point.y = tmp.floatV;

			tmp.bytes[0] = laserCloud->data[p+8];
			tmp.bytes[1] = laserCloud->data[p+9];
			tmp.bytes[2] = laserCloud->data[p+10];
			tmp.bytes[3] = laserCloud->data[p+11];
			point.z = tmp.floatV;

			tmp.bytes[0] = laserCloud->data[p+16];
			tmp.bytes[1] = laserCloud->data[p+17];
			tmp.bytes[2] = laserCloud->data[p+18];
			tmp.bytes[3] = laserCloud->data[p+19];
			intensities.values.push_back(tmp.floatV);

			index.values.push_back(n);

			cloud.points.push_back(point);
		}
	}

	cloud.channels.push_back(intensities);
	cloud.channels.push_back(index);


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


	/*ros::Publisher pubProcessedPointCloud = nh.advertise<sensor_msgs::PointCloud2>
	("/point_cloud_processed", 1);*/


	nh.param<int>("ring", ring_no, 15);


	ros::Subscriber subRawPointCloud = nh.subscribe<sensor_msgs::PointCloud2>
	("/velodyne_points", 2, rawCloudHandler);
	
	output_ = nh.advertise<sensor_msgs::PointCloud>("velodyne_1d_cloud", 10);

	/*
	ros::Publisher pubOccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>
	("/velodyne_evidence_grid", 1);

	pubProcessedPointCloudPointer = &pubProcessedPointCloud;
	subRawPointCloudPointer = &subRawPointCloud;
	pubOccupancyGridPointer = &pubOccupancyGrid;

	// Set up PCL Viewer
	viewer.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PointCloud"));
	viewer->registerKeyboardCallback (keyboardEventOccurred);
	//
	//	// Create view ports
	//	// 1 view port
	viewer->createViewPort(0.0,0.0,1.0,1.0,v2);

	// 2 view ports
	//viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
	//viewer->createViewPort(0.5,0.0,1.0,1.0,v2);

	// 4 view ports
	//	viewer->createViewPort (0.0, 0.5, 0.5, 1.0, v1);
	//	viewer->createViewPort (0.5, 0.5, 1.0, 1.0, v2);
	//	viewer->createViewPort (0.0, 0.0, 0.5, 0.5, v3);
	//	viewer->createViewPort (0.5, 0.0, 1.0, 0.5, v4);


	viewer->addCoordinateSystem(1.0);

	n = 0;
	*/
	ros::spin();

	return (0);
}
