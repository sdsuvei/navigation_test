#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "hough_line_detect/line.h"
#include "line_calculation/errormsg.h"
#include "sick_node/hough_points.h"
#include <std_msgs/Int16.h>
#include <list>
#include "std_msgs/Float32MultiArray.h"
#include <visualization_msgs/Marker.h>

//#include <iostream>
//#include <fstream>

#define THRESHOLD 0.30 // threshold in meters(on each side of the line)
#define QUALITY_RANGE_DIFF 15



#define LINE_BLUE  1
#define LINE_LEFT  2
#define LINE_RIGHT 3

using namespace message_filters;
ros::Publisher angleerror_pub;
std::list<float> dist_error_list;
std::list<float> angle_error_list;

std::list<float> avg_blobs_right;
std::list<float> avg_blobs_left;

ros::Publisher eol_left_pub;
ros::Publisher eol_right_pub;

ros::Publisher marker_pub_blue;
ros::Publisher marker_pub_right;
ros::Publisher marker_pub_left;

void line(float x1,float y1, float x2,float y2, int line){
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = line_list.header.frame_id = "/laser";
        line_list.header.stamp = line_list.header.stamp = ros::Time::now();
        line_list.ns = line_list.ns = "points_and_lines";
        line_list.action = line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        line_list.scale.x = 0.1;

	if(line == LINE_BLUE){
	        line_list.color.b = 1.0;
	} else {
		line_list.color.r = 1.0;
	}
        line_list.color.a = 1.0;
	geometry_msgs::Point p;
        p.x = x2;
        p.y = y2;
        p.z = 0;

        // The line list needs two points for each line
        line_list.points.push_back(p);

        p.x = x1;
        p.y = y1;
        p.z = 0;
        line_list.points.push_back(p);

	if(line == LINE_BLUE){
		marker_pub_blue.publish(line_list);
	}else if(line == LINE_LEFT){
		marker_pub_left.publish(line_list);
	} else if(line == LINE_RIGHT){
		marker_pub_right.publish(line_list);
	}
}



float calc_avg(float new_angle, std::list<float> &list){
        float sum = 0;
        //angle_error_vector.push_back(new_angle_error);
        list.push_back(new_angle);
        if(list.size() > 10){
                list.pop_front();
        }

        for(std::list<float>::iterator it = list.begin(); it != list.end(); ++it) {
                sum+=*it;
        }
        return sum/(signed)list.size();
}


float get_avg(std::list<float> &list){
	int sum = 0;
        for(std::list<float>::iterator it = list.begin(); it != list.end(); ++it) {
                sum+=*it;
        }
	if(list.size() == 0){
		return 0;
	}
        return sum/(signed)list.size();
}

float point_to_line(float x1, float y1, float x2, float y2, sick_node::hough_point point){
//Distance formula:
//d = Abs[a*x0 + b - y]/Sqrt[a^2 + 1]
	printf("x1: %f, y1: %f, x2: %f, y2: %f\r\n", x1, y1, x2, y2);

        //Distance to this point
        float x0 = point.x;
        float y0 = point.y;

        if(y1 == y2){
                printf("parallel, discaring\n");
                return 0.0;
        }

        //calculate slope
        float a = (x2-x1)/(y2-y1);
        //and offset
        float b = x1-a*y1;

        printf("a: %f, b: %f, x: %f, y: %f\r\n", a, b, x0, y0);
        float dist = fabs(a*x0+b-y0)/sqrt(a*a+1);
        return dist;
}

float calc_distance(float x1, float y1, float x2, float y2){

	static float dist = 0;

	if(y1 == y2){
		printf("parallel\n");
		return dist;
	}

	float a = (x1-x2)/(y1-y2);
	float b = x1-a*y1;
        dist = fabs((0-b)/a);

	return dist;
}

float calc_angleError(float x1, float y1, float x2, float y2){
	static float angleError = 0;
	float a = 0;

        if(y1 == y2){
                printf("parallel\n");
                return angleError;
        }
	a = (x2-x1)/(y2-y1);


	if(a > 0){
		angleError = (90-atan(a)*(180/3.14));
	} else {
		angleError = -(90+atan(a)*(180/3.14));
	}
	return angleError;
}

void callback(const sick_node::hough_points::ConstPtr& line_left, const sick_node::hough_points::ConstPtr& line_right){


	//Draw lines
//	line(line_right->begin.x, line_right->begin.y, line_right->end.x, line_right->end.y, LINE_RIGHT);
//	line(line_left->begin.x, line_left->begin.y, line_left->end.x, line_left->end.y, LINE_LEFT);

	float angle_error = 0;
	float dist_error = 0;
        std_msgs::Float32MultiArray distToPlot;

	int blobs_left = 0;
	int blobs_right = 0;

	float tmp_dist = 0;

	line(line_right->begin.x, line_right->begin.y, line_right->end.x, line_right->end.y, LINE_BLUE);


	///////////////////LEFT///////////////////////////
        for(int i = 0; i < line_left->points.size(); ++i){
                //Calculate distance between point and line
                tmp_dist = point_to_line(line_left->begin.x, line_left->begin.y, line_left->end.x, line_left->end.y, line_left->points[i]);
		printf("dist left: %f\r\n", tmp_dist);
                if(tmp_dist < THRESHOLD){
			++blobs_left;
//                        distToPlot.data.push_back(line_left->points[i].y);
                }
        }
	float dist_left = calc_distance(line_left -> begin.x, line_left -> begin.y, line_left -> end.x, line_left -> end.y);



	/////////////////RIGHT////////////////////////
        for(int i = 0; i < line_right->points.size(); ++i){
                //Calculate distance between point and line

                tmp_dist = point_to_line(line_right->begin.y, line_right->begin.x, line_right->end.y, line_right->end.x, line_right->points[i]);
                if(tmp_dist < THRESHOLD){
			++blobs_right;
//                        distToPlot.data.push_back(line_right->points[i].y);
                }
        }

	float dist_right = calc_distance(line_right -> begin.x, line_right -> begin.y, line_right -> end.x, line_right -> end.y);

	int right_good = 0;
	int left_good = 0;

	printf("blobs right: %d, blobs left: %d \r\n", blobs_right, blobs_left);

	if( fabs(get_avg(avg_blobs_left) - blobs_left) < QUALITY_RANGE_DIFF){
		calc_avg(blobs_left, avg_blobs_left);
		left_good = 1;
	}


	if( fabs(get_avg(avg_blobs_right) - blobs_right) < QUALITY_RANGE_DIFF){
		calc_avg(blobs_right, avg_blobs_right);
		right_good = 1;
	}

	dist_error = dist_left - dist_right;

	if(angle_error > 90){
		angle_error = 90;
	}

	if(angle_error < -90){
		angle_error = -90;
	}

	angle_error = calc_avg(angle_error, angle_error_list);
	dist_error = calc_avg(dist_error, dist_error_list);

	printf("dist right: %f, dist left: %f \r\n",dist_left, dist_right);
	line_calculation::errormsg errormsg;

	errormsg.angle = angle_error;
	errormsg.displacement = dist_error;
	angleerror_pub.publish(errormsg);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "combine_calculate_node");

  ros::NodeHandle nh;

  eol_left_pub = nh.advertise<std_msgs::Float32MultiArray>("/fmError/lql", 100);
  eol_right_pub = nh.advertise<std_msgs::Float32MultiArray>("/fmError/lqr", 100);

  angleerror_pub = nh.advertise<line_calculation::errormsg>("/fmProcessing/angleError", 1000);

  marker_pub_blue = nh.advertise<visualization_msgs::Marker>("/fmDebug/line", 10);
//  marker_pub_left = nh.advertise<visualization_msgs::Marker>("/draw_line_left", 10);
//  marker_pub_right = nh.advertise<visualization_msgs::Marker>("/draw_line_right", 10);

//  ros::Subscriber line = nh.subscribe("/fmProcessing/line_left", 1000, CallbackLine);


  message_filters::Subscriber<sick_node::hough_points> left_line_sub(nh, "/fmProcessing/line_left", 1);
  message_filters::Subscriber<sick_node::hough_points> right_line_sub(nh, "/fmProcessing/line_right", 1);

  //TimeSynchronizer<hough_line_detect::line, hough_line_detect::line> sync(left_line_sub, right_line_sub, 10);

  typedef sync_policies::ApproximateTime<sick_node::hough_points, sick_node::hough_points> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_line_sub, right_line_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

