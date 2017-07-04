#ifndef _NODE_H
#define _NODE_H

#include "path_planning/robot_perception.h"

#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <vector>
#include <limits>


struct node {
	int x;
	int y;
	int node_counter;
	int** connections;
	float** distance;
};


class Node {
	
	private:
		ros::NodeHandle _node;
		ros::Publisher _graph_pub;
		ros::Publisher _graph_connections_pub;
		ros::Publisher _marker_pub;
		
		
		RobotPerception robot_perception;

		
		int _step_node;
		int _counter; // gia to node_counters
		
		int _start_counter;
		int _goal_counter;
		
		
		std::vector <node> _neighbour_cell_test; //uniforms
		std::vector <node> _neighbour_cell; //uniforms

		int ** _create_graph;
	
	
	public:
	
		Node();
		void createNodes(int width, int height, float resolution, int map_size, 
						int curr_map_x, int curr_map_y, int goal_map_x, int goal_map_y);
		
		void createGraph(int map_size, float resolution);
		void visualGraph(int size, float resolution);

		int getNeighbourCellSize();
		int getNeighbourCellX(int ii);
		int getNeighbourCellY(int ii);
		int getNeighbourCellNodeCounter(int curr_map_x, int curr_map_y);
		int getStep();
		float getMinDistance(int ii, int last_curr_node);
		
		bool obstacleSearch(int x1, int y1, int x2, int y2);
		bool getObstacleSearch(int ii, int jj);
		bool getPixelGraph(int x, int y);
	
		
		int startCounter();
		int goalCounter();
		
		void visual(float x, float y, float x1, float y1, int ii);
	
};

#endif
