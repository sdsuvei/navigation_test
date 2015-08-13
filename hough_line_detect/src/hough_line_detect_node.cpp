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
        ros::Publisher scan_pub_, line_pub_;
	ros::NodeHandle n;
	ros::Subscriber cloud_sub;

        int x;
        int y;


        Transform(std::string pointCloudIn, std::string lineOut){
		cloud_sub = n.subscribe(pointCloudIn, 5, &Transform::pointCallback, this); //subscribing to this same function
                line_pub_ = n.advertise<sick_node::hough_points>(lineOut, 10); //publishing the line
                houghtransform.SetSize(800, 800);
        }

        void pointCallback (const sick_node::hough_points::ConstPtr& points_in){
		sick_node::hough_points pointcloudWline;
		sick_node::hough_point pointToCloud;

                for(int i = 0; i < points_in->points.size(); ++i){
                        if(!(points_in->points[i].x == 0 && points_in->points[i].y == 0)) {
                                x = (points_in->points[i].x)*100+400;
                                y = (points_in->points[i].y)*100+400;
                                img[x][y] = 1;
	                }
			pointToCloud.x =  points_in->points[i].x;
			pointToCloud.y =  points_in->points[i].y;

			pointcloudWline.points.push_back(pointToCloud);
//			pointcloudWline.points[i].x = points_in->points[i].x;
//			pointcloudWline.points[i].y = points_in->points[i].y;
		}
                Line result = houghtransform.Transform(img);

//		hough_line_detect::line line;
//		line.header.seq++;
//		line.header.stamp = ros::Time::now();
//		line.header.frame_id = "/laser";

		pointcloudWline.header.seq++;
		pointcloudWline.header.stamp = ros::Time::now();
		pointcloudWline.header.frame_id = "/laser";

		pointcloudWline.begin.x = ((float)result.x1-400)/100;
		pointcloudWline.begin.y = ((float)result.y1-400)/100;

		pointcloudWline.end.x = ((float)result.x2-400)/100;
		pointcloudWline.end.y = ((float)result.y2-400)/100;

                //scan_pub_.publish(cloud);
		line_pub_.publish(pointcloudWline);
        }
};

int main(int argc, char** argv){

        const int w = 800;
        const int h = 800;
        ros::init(argc, argv, "hough_line_detect");

	std::string in;
	std::string line_out;

	int from_point = 0;
	int to_point = 0;
        ros::NodeHandle n("~");

	n.param<std::string>("in",in,"/fmProcessing/laserpointCloud");
	n.param<std::string>("line_out", line_out,"/fmProcessing/line");

        Transform transform(in, line_out);

        transform.img.reserve(w);
        for(int i = 0; i < w; ++i){
                std::vector<int> temp(h);
                transform.img.push_back(temp);
        }
  ros::spin();
  return 0;
}



