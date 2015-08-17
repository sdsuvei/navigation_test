#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"
#include "hough_line_detect/line.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include "sick_node/hough_points.h"
#include "hough-transform.h"
#include <vector>


/* optimize offset calculation (400 / depend on from_point -> to_point) */

laser_geometry::LaserProjection projector_;
ros::Publisher scan_pub_;

class Transform {
public:

	std::vector<std::vector<int> > img;
	HoughTransform houghtransform;

	ros::Publisher scan_pub_, line_pub_, line_pub2_;;
	ros::NodeHandle n;
	ros::Subscriber cloud_sub;

	int x;
	int y;



	Transform(std::string pointCloudIn, std::string lineOut, std::string lineOut1){
		//cloud_sub = n.subscribe(pointCloudIn, 5, &Transform::pointCallback, this); //subscribing to this same function
		cloud_sub = n.subscribe(pointCloudIn, 5, &Transform::pointCallback2, this); //subscribing to this same function
		line_pub_ = n.advertise<sick_node::hough_points>(lineOut, 10); //publishing the line
		line_pub2_ = n.advertise<sick_node::hough_points>(lineOut1, 10); //publishing the line
		houghtransform.SetSize(2, 300);
	}

	void pointCallback2 (const sensor_msgs::PointCloud::ConstPtr & cloud_in){
		sick_node::hough_points pointcloudWline;
		sick_node::hough_points pointcloudWline2;

		OutputLines result = houghtransform.Transform2(cloud_in);

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
	}

};

int main(int argc, char** argv){

	const int w = 800;
	const int h = 800;
	ros::init(argc, argv, "hough_2lines_detect");

	std::string in;
	std::string line_out;

	std::string line_out1;

	int from_point = 0;
	int to_point = 0;
	ros::NodeHandle n("~");

	//n.param<std::string>("in",in,"/fmProcessing/laserpointCloud");

	n.param<std::string>("in",in,"/laserpointCloud");
	n.param<std::string>("line_out", line_out,"/2lines");
	n.param<std::string>("line_out1", line_out1,"/1lines");

	Transform transform(in, line_out,line_out1);

	transform.img.reserve(w);
	for(int i = 0; i < w; ++i){
		std::vector<int> temp(h);
		transform.img.push_back(temp);
	}
	ros::spin();
	return 0;

}



