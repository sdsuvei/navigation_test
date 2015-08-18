#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"
#include "hough_line_detect/line.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include "sick_node/hough_points.h"
#include "hough-transform.h"
#include <vector>
#include <std_msgs/Float64MultiArray.h>


/* optimize offset calculation (400 / depend on from_point -> to_point) */

laser_geometry::LaserProjection projector_;
ros::Publisher scan_pub_;

class Transform {
public:

	std::vector<std::vector<int> > img;
	HoughTransform houghtransform;

	ros::Publisher scan_pub_, line_pub_, line_pub2_, hough_img_;
	ros::NodeHandle n;
	ros::Subscriber cloud_sub;
	ros::Publisher array_pub;

	int x;
	int y;



	Transform(std::string pointCloudIn, std::string lineOut, std::string lineOut1, int angle_res, int height, double r_res, double min_row_distance,double max_point_distance, std::string dataOut){
		hough_img_ = n.advertise<sensor_msgs::Image>("/hough_image", 10);
		cloud_sub = n.subscribe(pointCloudIn, 5, &Transform::pointCallback2, this); //subscribing to this same function
		line_pub_ = n.advertise<sick_node::hough_points>(lineOut, 10); //publishing the line
		line_pub2_ = n.advertise<sick_node::hough_points>(lineOut1, 10); //publishing the line
		array_pub = n.advertise<std_msgs::Float64MultiArray>(dataOut, 10);
		houghtransform.SetParams(angle_res, height, r_res, min_row_distance, max_point_distance);
	}

	void pointCallback2 (const sensor_msgs::PointCloud::ConstPtr & cloud_in){
		sick_node::hough_points pointcloudWline;
		sick_node::hough_points pointcloudWline2;

		OutputLines result = houghtransform.Transform2(cloud_in, &hough_img_);

				//hough_line_detect::line line;
				//line.header.seq++;
				//line.header.stamp = ros::Time::now();
				//line.header.frame_id = "/laser";
		
		std::cout << "line1 : " << "\n";
		result.line1.printVars();

		std::cout << "line2 : " <<  "\n";
		result.line2.printVars();

		pointcloudWline.header.seq++;
		pointcloudWline.header.stamp = ros::Time::now();
		pointcloudWline.header.frame_id = "/laser";


		pointcloudWline.begin.x = ((float)result.line1.x1);
		pointcloudWline.begin.y = ((float)result.line1.y1);

		pointcloudWline.end.x = ((float)result.line1.x2);
		pointcloudWline.end.y = ((float)result.line1.y2);

		//scan_pub_.publish(cloud);
		line_pub_.publish(pointcloudWline);

		pointcloudWline2.header.seq++;
		pointcloudWline2.header.stamp = ros::Time::now();
		pointcloudWline2.header.frame_id = "/laser";

		pointcloudWline2.begin.x = ((float)result.line2.x1);
		pointcloudWline2.begin.y = ((float)result.line2.y1);

		pointcloudWline2.end.x = ((float)result.line2.x2);
		pointcloudWline2.end.y = ((float)result.line2.y2);

		//scan_pub_.publish(cloud);
		line_pub2_.publish(pointcloudWline2);

		std_msgs::Float64MultiArray HoughMsg;
		HoughMsg.data.clear();

		HoughMsg.data.push_back((float)result.theta);
		HoughMsg.data.push_back((float)result.r1);
		HoughMsg.data.push_back((float)result.r2);
		HoughMsg.data.push_back((float)result.variance);

		array_pub.publish(HoughMsg);

	}

};

int main(int argc, char** argv){

	//const int w = 800;
	//const int h = 800;
	ros::init(argc, argv, "hough_2lines_detect");

	std::string in;
	std::string line_out;
	int height, angle_res;
	double min_row_distance, max_point_distance, r_res;

	std::string line_out1;

	int from_point = 0;
	int to_point = 0;
	ros::NodeHandle n("~");

	//n.param<std::string>("in",in,"/fmProcessing/laserpointCloud");

	std::string measHough;
	n.param<std::string>("measHough", measHough,"/measHough");

	n.param<std::string>("in",in,"/laserpointCloud");
	n.param<std::string>("line_out", line_out,"/2lines");
	n.param<std::string>("line_out1", line_out1,"/1lines");
	n.param<int>("height",height,100);
	n.param<int>("angle_res",angle_res,1);
	n.param<double>("min_row_distance", min_row_distance,0.5);
	n.param<double>("max_point_distance",max_point_distance,2.0);
	n.param<double>("r_res",r_res,0.05f);

	Transform transform(in, line_out,line_out1, angle_res, height, r_res, min_row_distance,max_point_distance, measHough);
/*
	transform.img.reserve(w);
	for(int i = 0; i < w; ++i){
		std::vector<int> temp(h);
		transform.img.push_back(temp);
	}*/
	ros::spin();
	return 0;

}



