//============================================================================
// Name        : Hough.cpp
// Author      : Exchizz
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "hough-transform.h"
using namespace std;

int y(int x){
	int retval = 5*x;
	return retval;
}

void HoughTransform::SetSize(int w, int h){
	this->w = w;
	this->h = h;
	hough_h = ((sqrt(2.0) * (double)(h>w?h:w)) / 2.0); //maximum height for the accumulator
	_accu_h = hough_h * 2.0; // -r -> +r
}

HoughTransform::HoughTransform(){

	x1 = y1 = x2 = y2 = 0;

	max.current = 0;
	max.r = 0;
	max.t = 0;
}


Line HoughTransform::Transform(std::vector<std::vector<int> > & img){

	   // int * accumulator = new int[180*_accu_h];
	    std::vector<int> accumulator(180*_accu_h); // create an empty accumulator with 180 bins (because theta=[0,180])
	    double x_center = w/2;
	    double y_center = h/2;


	max.current = 0;
	max.t = 0;
	max.r = 0;

		for(int y = 0; y < h; y++){
			for(int x = 0; x < w; x++){
				if(img[x][y] == 1){
					//count++;
					for(int t = 0; t < 180; t++){
						double r = ( ((double)x - x_center) * cos((double)t * DEG2RAD)) + (((double)y - y_center) * sin((double)t * DEG2RAD));
						accumulator[ (int)((round(r + hough_h) * 180.0)) + t]++;
					}
				}
				img[x][y] = 0;
			}
		}

		//std::cout << "value of countz: " << count << "\n";



		for(int t = 0; t < 180; t++){
			for(int r = 0; r < _accu_h; r++){
				if(max.current < accumulator[r*180+t]){
					max.current = accumulator[r*180+t];
					max.t = t;
					max.r = r;
				}
			}
		}

		if(max.t >=45 && max.t <= 135){
			//y = (r - x cos(t)) / sin(t);
			x1 = 0;
			y1 = ((double)(max.r -(_accu_h/2)) - ((x1 - (w/2) ) * cos(max.t * DEG2RAD))) / sin(max.t * DEG2RAD) + (h / 2);
			x2 = w;
			y2 = ((double)(max.r -(_accu_h/2)) - ((x2 - (w/2) ) * cos(max.t * DEG2RAD))) / sin(max.t * DEG2RAD) + (h / 2);
		} else {
			//x = (r - y sin(t)) / cos(t);
			y1 = 0;
			x1 = ((double)(max.r -(_accu_h/2)) - ((y1 - (h/2) ) * sin(max.t * DEG2RAD))) / cos(max.t * DEG2RAD) + (w / 2);
			y2 = h;
			x2 = ((double)(max.r -(_accu_h/2)) - ((y2 - (h/2) ) * sin(max.t * DEG2RAD))) / cos(max.t * DEG2RAD) + (w / 2);
		}
		Line line;
		line.x1 = x1;
		line.y1 = y1;
		line.x2 = x2;
		line.y2 = y2;
		//line.printVars();
		return line;
}
/*
int main() {
	int x1,y1,x2,y2;

	const int w = 70;
	const int h = y(w);


	std::vector<std::vector<int> > img;
	img.reserve(w);
	for(int i = 0; i < w; ++i){
		std::vector<int> temp(h);
		img.push_back(temp);
	}

	for(int x = 0; x < w; ++x){
		int y_ = y(x);
		std::cout << "x: " << x << " y: " << y_ << "\n";
		//making binary image - pixel = 1 else 0
		img[x][y_] = 1;
		//std::cout << " value from vector: "<< img[x][y_];
	}


	HoughTransform houghtransform(w, h);
	Line result = houghtransform.Transform(img);

	x1 = result.x1;
	y1 = result.y1;
	x2 = result.x2;
	y2 = result.y2;

	std::cout << "x1: " << x1 << " y1: " << y1 << "\n";
	std::cout << "x2: " << x2 << " y2: " << y2 << "\n";
	//cout << "x1: " << x1 << " y1: " << y1 << "\nx2: " << x2 << " y2: " << y2 << "\n";
	if((x2-x1) != 0 ){
		double a = (y2-y1)/(x2-x1);
		std::cout << "a: " << a << "\n";
	}else {
		cout << "Divide by 0\n";
	}
	cout << "Hello World" << endl; // prints !!!Hello World!!!
	return 0;
}

*/
