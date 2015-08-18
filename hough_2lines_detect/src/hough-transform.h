#include <iostream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>
#include "sensor_msgs/PointCloud.h"

#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"

#define DEG2RAD .017453293f
#define RRESOLUTION 0.05f

struct Max {

	double current;
	double r;
	double r_second;
	double t;
};

struct Line {
	int x1;
	int y1;
	int x2;
	int y2;

	void printVars(){

		std::cout << "point1 = (" << x1 <<" , " << y1 <<  ")\n";
		std::cout << "point2 = (" << x2 <<" , " << y2 <<  ")\n";
	}
};
struct OutputLines
{
	Line line1;
	Line line2;
};


class HoughTransform {
private:
	double hough_h;
	int _accu_h;
	Max max;
	int x1,
		x2,
		y1,
		y2;
	int h, w;
	double tRes;

public:
	HoughTransform();
	//Line Transform(std::vector<std::vector<int> >&img);
	OutputLines Transform2(const sensor_msgs::PointCloud::ConstPtr & cloud_in, ros::Publisher* hough_img_);
	void SetSize(int w, int h);
};

