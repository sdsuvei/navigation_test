#include <iostream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>
#include "sensor_msgs/PointCloud.h"


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

		std::cout << "x's: " << x1 <<" and " << x2 <<  "\n";
		std::cout << "y1: " << y1 << " and " << y2 << "\n";
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

public:
	HoughTransform();
	Line Transform(std::vector<std::vector<int> >&img);

	OutputLines Transform2(const sensor_msgs::PointCloud::ConstPtr & cloud_in);

	void SetSize(int w, int h);
};

