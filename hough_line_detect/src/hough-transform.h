#include <iostream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>


#define DEG2RAD .017453293f

struct Max {
	int current;
	int r;
	int t;
};

struct Line {
	int x1;
	int y1;
	int x2;
	int y2;

	void printVars(){
		std::cout << "x1: " << y1 <<  "\n";
		std::cout << "21: " << y2 <<  "\n";
	}
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
	void SetSize(int w, int h);
};

