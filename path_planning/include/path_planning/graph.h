#ifndef _GRAPH_H
#define _GRAPH_H

#include "path_planning/robot_perception.h"


#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>



class Graph {
	
	ros::NodeHandle _node;
	ros::Publisher _graph_pub;
	
	RobotPerception robot_perception;
	
	int _step;
	
	int ** _create_graph;
	
	public:
	
	
	//~ Graph();
	//~ 
	//~ void createGraph(int size);
	//~ void visualGraph(int size);
	//~ bool obstacleSearch(int x1, int y1, int x2, int y2);z
	
};

#endif
