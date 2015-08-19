#include "navigation.h"

int main(int argc, char** argv){	
	ros::init(argc, argv, "navigation_node");
	
	Frobit *frobo = new Frobit();
    ros::Rate loop_rate(10); // publish 10 messages per second
	 while (ros::ok()){			
		frobo->updateVel(); //activate motors
		ros::spinOnce();
		loop_rate.sleep();
        }
 return 0;
}


