#include "path_planning/graph.h"

//~ Graph::Graph()
//~ {
	//~ _graph_pub = _node.advertise<visualization_msgs::Marker>("visualization_graph", 1);
//~ }
//~ 
//~ 
//~ 
//~ void Graph::createGraph(int size)
//~ {
	//~ _create_graph = new int *[size];
	//~ for (unsigned int ii = 0; ii < size; ii ++) 
	//~ {
		//~ _create_graph[ii] = new int [size];
		//~ for (unsigned int jj = 0; jj < size; jj ++) 
		//~ {
			//~ _create_graph[ii][jj] = 0;
		//~ }
	//~ }
	//~ 
	//~ for (unsigned int ii = 0; ii < size; ii ++)
	//~ {
		//~ for (unsigned int jj = 0; jj < size; jj ++)
		//~ {
			//~ if (ii != jj)
			//~ {
				//~ if ((_neighbour_cell[ii].x == _neighbour_cell[jj].x + _step && _neighbour_cell[ii].y == _neighbour_cell[jj].y)
					//~ || (_neighbour_cell[ii].x == _neighbour_cell[jj].x + _step && _neighbour_cell[ii].y == _neighbour_cell[ii].y + _step)
					//~ || (_neighbour_cell[ii].x == _neighbour_cell[jj].x && _neighbour_cell[ii].y == _neighbour_cell[ii].y + _step)
					//~ || (_neighbour_cell[ii].x == _neighbour_cell[jj].x - _step && _neighbour_cell[ii].y == _neighbour_cell[ii].y + _step))
				//~ {
					//~ if (obstacleSearch(_neighbour_cell[ii].x, _neighbour_cell[ii].y, _neighbour_cell[jj].x, _neighbour_cell[jj].y))
					//~ {
						//~ _create_graph[ii][jj] = 1;
					//~ }
				//~ }
			//~ }
		//~ }
	//~ }
	//~ 
//~ }

//~ void Graph::visualGraph(int size)
//~ {
	//~ // Visualize the nodes
    //~ visualization_msgs::Marker marker;
	//~ geometry_msgs::Point p;
	//~ 
	//~ marker.header.frame_id = "map";
	//~ marker.header.stamp = ros::Time::now();
	//~ 
	//~ marker.type = visualization_msgs::Marker::CUBE_LIST;
	//~ marker.action = visualization_msgs::Marker::ADD;
	//~ 
	//~ marker.id = 0;
	//~ marker.ns = "path_planning";
		//~ 
	//~ for (unsigned int ii = 0; ii < size; ii ++)
	//~ {
		//~ p.x = _neighbour_cell[ii].x * robot_perception.getMapResolution();
		//~ p.y = _neighbour_cell[ii].y * robot_perception.getMapResolution();
		//~ p.z = 0;
		//~ marker.points.push_back(p);	
	//~ }
	//~ 
	//~ marker.pose.orientation.x = 0.0;
	//~ marker.pose.orientation.y = 0.0;
	//~ marker.pose.orientation.z = 0.0;
	//~ marker.pose.orientation.w = 1.0;
  //~ 
	//~ marker.scale.x = 0.15;
	//~ marker.scale.y = 0.15;
	//~ marker.scale.z = 0.00;
	//~ marker.color.a = 1.0;
	//~ marker.color.r = 1.0;
	//~ marker.color.g = 0.0;
	//~ marker.color.b = 0.0;
	//~ 
	//~ marker.lifetime = ros::Duration();
	//~ 
	//~ _graph_pub.publish(marker);
    //~ 
//~ }

//~ bool Graph::obstacleSearch(int x1, int y1, int x2, int y2)
//~ {
	//~ int counter = 0;
	//~ bool obst_flag = false;
	//~ 
	//~ for (unsigned int ii = 0; ii < _step; ii ++)
	//~ {
		//~ if ((x1 == x2 + _step) && (y1 == y2))
		//~ {
			//~ if (robot_perception.rightCell(x2 + ii, y2))
			//~ {
				//~ counter ++ ;
			//~ }
		//~ }
		//~ if ((x1 == x2 + _step) && (y1 == y2 + _step))
		//~ {
			//~ if (robot_perception.rightCell(x2 + ii, y2 + ii))
			//~ {
				//~ counter ++ ;
			//~ }
		//~ }
		//~ if ((x1 == x2) && (y1 == y2 + _step))
		//~ {
			//~ if (robot_perception.rightCell(x2, y2 + ii))
			//~ {
				//~ counter ++ ;
			//~ }
		//~ }
		//~ if ((x1 == x2 - _step) && (y1 == y2 + _step))
		//~ {
			//~ if (robot_perception.rightCell(x2 - ii, y2 + ii))
			//~ {
				//~ counter ++ ;
			//~ }
		//~ }
	//~ }
	//~ 
	//~ if (counter = _step)
	//~ {
		//~ obst_flag = true;
	//~ }
	//~ 
	//~ return obst_flag;
	//~ 
//~ }




			
