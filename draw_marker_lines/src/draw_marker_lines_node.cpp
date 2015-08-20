#include "ros/ros.h"
#include "hough_line_detect/line.h"
#include <visualization_msgs/Marker.h>
#include "sick_node/hough_points.h"
#include <obstacle_detection/boundingbox.h>
#include <obstacle_detection/boundingboxes.h>

float Start_x = 0;
float Start_y = 0;	
float Width_x = 0;
float Width_y = 0;

ros::Publisher marker_pub,marker_pub_BBox;
void line(float x1,float y1, float z1,float x2,float y2, float z2){
	visualization_msgs::Marker line_list;
        line_list.header.frame_id = line_list.header.frame_id = "/laser";
        line_list.header.stamp = line_list.header.stamp = ros::Time::now();
        line_list.ns = line_list.ns = "points_and_lines";
        line_list.action = line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        line_list.scale.x = 0.1;

        line_list.color.r = 1.0;
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

            marker_pub.publish(line_list);
        }
        
      visualization_msgs::Marker lines(float x1,float y1, float z1,float x2,float y2, float z2, visualization_msgs::Marker lineCollection ){
		visualization_msgs::Marker line_list;
        line_list.header.frame_id = line_list.header.frame_id = "/laser";
        line_list.header.stamp = line_list.header.stamp = ros::Time::now();
        line_list.ns = line_list.ns = "points_and_lines";
        line_list.action = line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        line_list.scale.x = 0.1;

        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
            geometry_msgs::Point p;
            p.x = x2;
            p.y = y2;
            p.z = 0;

            // The line list needs two points for each line
            lineCollection.points.push_back(p);

            p.x = x1;
            p.y = y1;
            p.z = 0;
            lineCollection.points.push_back(p);

            //marker_pub.publish(line_list);
        }

void pointsCallback (const sick_node::hough_points::ConstPtr& line_in){
	line(line_in->begin.x, line_in->begin.y,0, line_in->end.x, line_in->end.y, 0);
}

void bboxesCallback(const obstacle_detection::boundingboxes& bboxes)
{
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/laser";
	 line_list.header.stamp = ros::Time::now();
     line_list.ns = "BBox of Objects";
     line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 2;

    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.1;


    // Line list is red
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
		for (size_t i=0;i<bboxes.boundingboxes.size();i++)
	{
		
			Start_x = bboxes.boundingboxes[i].start.x;
			Start_y = bboxes.boundingboxes[i].start.y;	
			Width_x = bboxes.boundingboxes[i].width.x;
			Width_x = bboxes.boundingboxes[i].width.y;
		    //1-2
		    line_list=lines(Start_x, Start_y, 0, Start_x+Width_x, Start_y, 0,line_list);
		    
		    //2-3
		    line_list=lines(Start_x+Width_x, Start_y, 0, Start_x+Width_x, Start_y+Width_y, 0,line_list) ;
		    //3-4
		    line_list=lines(Start_x+Width_x, Start_y+Width_y, 0, Start_x, Start_y+Width_y, 0,line_list);
		    //1-4
		    line_list=lines( Start_x, Start_y, 0, Start_x, Start_y+Width_y, 0,line_list);
	}
	
    marker_pub_BBox.publish(line_list);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "plot_line");

  ros::NodeHandle n("~");
  std::string line_topic;
  std::string draw_topic;

  n.param<std::string>("line_topic",line_topic,"/fmProcessing/laserpointCloud");
  marker_pub_BBox = n.advertise<visualization_msgs::Marker>("BBbox_Viz", 10);
  n.param<std::string>("draw_topic",draw_topic,"/draw_line");
 // parameter handler goes here
  ros::Subscriber  sub = n.subscribe(line_topic, 1000, pointsCallback);
  ros::Subscriber sub_BB = n.subscribe("/obstacle_detection/boundingboxes", 5, bboxesCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>(draw_topic, 10);

  ros::spin();
  return 0;
}



