#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "sick_node/hough_points.h"

laser_geometry::LaserProjection projector_;
ros::Publisher point_pub;
ros::Publisher point_pub_left; //publisher left side and right side
ros::Publisher point_pub_right;


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
	sensor_msgs::PointCloud cloud; //create empty pointcloud
	projector_.projectLaser(*scan_in, cloud); //copy lasercloud to pointcloud 
	point_pub.publish(cloud); //publish it
	
	sick_node::hough_point left; 
	sick_node::hough_point right;

	sick_node::hough_points msgLeft;
	sick_node::hough_points msgRight;

	msgLeft.header.stamp = ros::Time::now();
	msgLeft.header.seq++;
        msgLeft.header.frame_id = "/laser";

	msgRight.header.stamp = ros::Time::now();
	msgRight.header.seq++;
        msgRight.header.frame_id = "/laser";

	int x = 0;
	int y = 0;
	for(int i = 0; i < cloud.points.size(); ++i){  //for all points smaller than y, copy in left pointcloud and opposite for right
		if(cloud.points[i].y < 0){ //should be y > 0, but sensor is mounted upside down
			right.x = cloud.points[i].x;
			right.y = cloud.points[i].y;
			msgRight.points.push_back(right);
		} else {
			left.x = cloud.points[i].x;
			left.y = cloud.points[i].y;
			msgLeft.points.push_back(left);
		}
	}
	point_pub_left.publish(msgLeft);
	point_pub_right.publish(msgRight);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sick_node");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");	
	
	std::string scan_in;	
    std::string pointcloud_out;
	std::string pointcloud_out_left;
	std::string pointcloud_out_right;

		
	nh.param<std::string>("scan_in",scan_in,"/scan");
	nh.param<std::string>("pointcloud_out",pointcloud_out,"/fmProcessing/laserpointCloud");
	
	nh.param<std::string>("out_left",pointcloud_out_left,"/fmProcessing/laserpointCloudLeft");
	nh.param<std::string>("out_right",pointcloud_out_right,"/fmProcessing/laserpointCloudRight");
	
	
	ros::Subscriber sub = n.subscribe(scan_in, 1000, scanCallback);
    point_pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud_out",1); //publisher for full pointcloud

	point_pub_left = n.advertise<sick_node::hough_points>(pointcloud_out_left,1);
	point_pub_right = n.advertise<sick_node::hough_points>(pointcloud_out_right,1);

	ros::spin();
  return 0;
}
