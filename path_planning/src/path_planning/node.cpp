#include "path_planning/node.h"

Node::Node()
{
	if(!_node.getParam("step_node", _step_node))
    {
		ROS_ERROR("Duration param does not exist");
	}
	
	_graph_pub = _node.advertise<visualization_msgs::Marker>("visualization_graph", 1);
	_graph_connections_pub = _node.advertise<visualization_msgs::MarkerArray>("visualization_graph_connection", 1);
	_marker_pub = _node.advertise<visualization_msgs::Marker>("visualize_graph", 1);
}

void Node::createNodes(int width, int height, float resolution, int map_size, int curr_map_x, 
						int curr_map_y, int goal_map_x, int goal_map_y)
{
	_neighbour_cell_test.clear();
	node N; // gia uniforms (test)
	
	_counter = 0;
	int curr_counter = 0;
	int goal_counter = 0;
	
	for (unsigned ii = 0; ii < width; ii = ii + _step_node)
	{
		for (unsigned jj = 0; jj < height; jj = jj + _step_node)
		{
			if (robot_perception.rightCell(ii, jj))
			{
				N.x = ii;
				N.y = jj;
				N.node_counter = _counter;
				_neighbour_cell_test.push_back(N);
				
				_counter ++ ;
			}
		}
	}
	
	int nodes_size = _neighbour_cell_test.size();
	
	// elegxw an h ekkinhsh kai o stoxos anhkoun ston grafo
	for (unsigned int zz = 0; zz < _neighbour_cell_test.size(); zz ++)
	{
		if (!(curr_map_x == _neighbour_cell_test[zz].x && curr_map_y == _neighbour_cell_test[zz].y))
		{
			curr_counter ++ ;
		}
		else
		{
			_start_counter = _neighbour_cell_test[zz].node_counter;
		}
		if (!(goal_map_x == _neighbour_cell_test[zz].x && goal_map_y == _neighbour_cell_test[zz].y))
		{
			goal_counter ++ ;
			//~ ROS_INFO_STREAM("telos " << _neighbour_cell_test[zz].node_counter);
		}
		else
		{
			//~ ROS_INFO_STREAM("telos " << _neighbour_cell_test[zz].node_counter);
			_goal_counter = _neighbour_cell_test[zz].node_counter;
		}
	}
	
	if (curr_counter == nodes_size)
	{
		N.x = curr_map_x;
		N.y = curr_map_y;
		N.node_counter = _counter;
		//~ ROS_INFO_STREAM("ARXH " << _counter);
		_start_counter = _counter;
		_counter ++ ;
		_neighbour_cell_test.push_back(N);
	}
	
	if (goal_counter == nodes_size)
	{
		N.x = goal_map_x;
		N.y = goal_map_y;
		N.node_counter = _counter;
		//~ ROS_INFO_STREAM("GOAL " << _counter);
		_goal_counter = _counter;
		_counter ++ ;
		_neighbour_cell_test.push_back(N);
	}
	//~ ROS_INFO_STREAM("GOAL " << _goal_counter << " " << _neighbour_cell_test[_goal_counter].x << " " << _neighbour_cell_test[_goal_counter].y);
	//~ ROS_INFO_STREAM("START " << _start_counter << " " << _neighbour_cell_test[_start_counter].x << " " << _neighbour_cell_test[_start_counter].y);
	
	createGraph(map_size, resolution);
	
	//~ graph_obj.visualGraph(_neighbour_cell.size());
	float k = 0.02;
	visualGraph(_neighbour_cell.size(), k);
	
}

void Node::createGraph(int map_size, float resolution)
{
	_neighbour_cell.clear();
	node M;
	
	M.connections = new int *[_neighbour_cell_test.size()];
	M.distance = new float *[_neighbour_cell_test.size()];
	for (unsigned int ii = 0; ii < _neighbour_cell_test.size(); ii ++) 
	{
		M.connections[ii] = new int [_neighbour_cell_test.size()];
		M.distance[ii] = new float [_neighbour_cell_test.size()];
		for (unsigned int jj = 0; jj < _neighbour_cell_test.size(); jj ++) 
		{
			M.connections[ii][jj] = 0;
			M.distance[ii][jj] = map_size + 1;
		}
	}
	
	float step;
	
	float step_x;
	float step_y;
	
	for (unsigned ii = 0; ii < _neighbour_cell_test.size(); ii ++)
	{
		M.x = _neighbour_cell_test[ii].x;
		M.y = _neighbour_cell_test[ii].y;
		M.node_counter = ii;
		
		for (unsigned jj = 0; jj < _neighbour_cell_test.size(); jj ++)
		{
			if (ii != jj)
			{
				if ((_neighbour_cell_test[ii].x % _step_node == 0 && _neighbour_cell_test[ii].y % _step_node == 0)
						&& (_neighbour_cell_test[jj].x % _step_node == 0 && _neighbour_cell_test[jj].y % _step_node == 0))
				{
					step = _step_node;
					//~ step_x = _neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x;
					//~ step_y = _neighbour_cell_test[ii].y - _neighbour_cell_test[jj].y;
					//~ ROS_INFO_STREAM("1");
				}
				else
				{
					step = 10;
					//~ step_x = _neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x;
					//~ step_y = _neighbour_cell_test[ii].y - _neighbour_cell_test[jj].y;
					
					//~ step_x = 10;
					//~ step_y = 10;
					
					//~ if (step_x > _step_node)
					//~ {
						//~ step_x = _step_node;
						//~ ROS_INFO_STREAM("2 " << step_x);
					//~ }
					//~ if (step_x < - _step_node)
					//~ {
						//~ step_x = - _step_node;
						//~ ROS_INFO_STREAM("2 " << step_x);
					//~ }
						//~ 
					//~ if (step_y > _step_node)
					//~ {
						//~ step_y = _step_node;
						//~ ROS_INFO_STREAM("3 " << step_y);
					//~ }
					//~ if (step_y < - _step_node)
					//~ {
						//~ step_y = _step_node;
						//~ ROS_INFO_STREAM("3 " << step_y);
					//~ }
				//~ }
				//~ if (step_x > _step_node)
				//~ {
					//~ step_x = _step_node;
					//~ ROS_INFO_STREAM("2 " << step_x);
				//~ }
				//~ if (step_x < - _step_node)
				//~ {
					//~ step_x = - _step_node;
					//~ ROS_INFO_STREAM("2 " << step_x);
				//~ }
					//~ 
				//~ if (step_y > _step_node)
				//~ {
					//~ step_y = _step_node;
					//~ ROS_INFO_STREAM("3 " << step_y);
				//~ }
				//~ if (step_y < - _step_node)
				//~ {
					//~ step_y = _step_node;
					//~ ROS_INFO_STREAM("3 " << step_y);
				//~ }
				
				//~ if ((_neighbour_cell_test[ii].x == _neighbour_cell_test[jj].x + step_x 
						//~ && _neighbour_cell_test[ii].y == _neighbour_cell_test[jj].y + step_y))
				//~ {
					//~ if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 	
														//~ _neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					//~ {
						//~ M.connections[ii][jj] = 1;
						//~ float dis;
						//~ dis = (_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x) * (_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x)
							//~ + (_neighbour_cell_test[ii].y - _neighbour_cell_test[jj].y) * (_neighbour_cell_test[ii].y - _neighbour_cell_test[jj].y);
						//~ dis = sqrt(dis);
						//~ 
						//~ M.distance[ii][jj] = dis;
						//~ 
						//~ if ((jj == _neighbour_cell_test.size() - 1) || (ii == _neighbour_cell_test.size() - 1))
						//~ {
							//~ ROS_INFO_STREAM("ti skata " << M.connections[ii][jj] << " " << _neighbour_cell_test[ii].x << " " 
										//~ << _neighbour_cell_test[ii].y << " " << M.connections[jj][ii] << " " 
										//~ << _neighbour_cell_test[jj].x << " " << _neighbour_cell_test[jj].y);
									//~ _neighbour_cell_test[ii].x << " " << _neighbour_cell_test[ii].y << " " << step_x << " "  
									//~ << _neighbour_cell_test[jj].x << " " << _neighbour_cell_test[jj].y << " " << step_y << " " << dis);
						//~ }
					//~ }
					//~ if (jj == 809 && ii == 1079) 
					//~ {
						//~ ROS_INFO_STREAM("ti skata " << _neighbour_cell_test[ii].x << " " << _neighbour_cell_test[ii].y << " " << step_x << " "  
									//~ << _neighbour_cell_test[jj].x << " " << _neighbour_cell_test[jj].y << " " << step_y);
					//~ }
				}
				
				if ((_neighbour_cell_test[ii].x == _neighbour_cell_test[jj].x + step 
						&& _neighbour_cell_test[ii].y == _neighbour_cell_test[jj].y))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 	
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x);
					}
				}
				
				if ((_neighbour_cell_test[ii].x == _neighbour_cell_test[jj].x + step 
						&& _neighbour_cell_test[ii].y == _neighbour_cell_test[jj].y + step))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x);
					}
				}
				
				if ((_neighbour_cell_test[ii].x == _neighbour_cell_test[jj].x 
						&& _neighbour_cell_test[ii].y == _neighbour_cell_test[jj].y + step))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y,
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].y - _neighbour_cell_test[jj].y);
					}
				}
				
				if ((_neighbour_cell_test[ii].x == _neighbour_cell_test[jj].x - step 
						&& _neighbour_cell_test[ii].y == _neighbour_cell_test[jj].y + step))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x);
					}
				}
			
			}
			//~ if (M.connections[ii][jj] == 1 && (ii == _neighbour_cell_test.size() - 1 || jj == _neighbour_cell_test.size() - 1))
			//~ {
				//~ visual(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, _neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y, ii);
				//~ ROS_INFO_STREAM("DF");
			//~ }
		}
		
		
		_neighbour_cell.push_back(M);
	}
	
	//~ ROS_INFO_STREAM("mpike " << _neighbour_cell_test[1079].x << " " << _neighbour_cell_test[1079].y << " " 
									//~ << _neighbour_cell_test[809].x << " " << _neighbour_cell_test[809].y << " " << 
									//~ _neighbour_cell[1079].connections[1079][809] << " " << _neighbour_cell[809].connections[809][1079]);
	
	visualGraph(_neighbour_cell.size(), resolution);
	
}

void Node::visual(float x, float y, float x1, float y1, int ii)
{
	visualization_msgs::Marker marker, line;
	visualization_msgs::MarkerArray line_strip;
	
	geometry_msgs::Point p;

	geometry_msgs::Point p1, p2;
	
	line.header.frame_id = "map";
	line.header.stamp = ros::Time::now();

	line.type = visualization_msgs::Marker::LINE_STRIP;
	line.action = visualization_msgs::Marker::ADD;
	
	line.id = ii;
	
	line.ns = "graph";
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.03;
	
	
	line.color.a = 1.0;
	line.color.r = 1.0;
	line.color.g = 0.5;
	line.color.b = 0.0;
	
	p1.x = x * 0.02;
	p1.y = y * 0.02;
	p1.z = 0;
	//~ ROS_INFO_STREAM("X1 " << x1 << " Y1 " << y1);
	
	p2.x = x1 * 0.02;
	p2.y = y1 * 0.02;
	p2.z = 0;
	// ROS_INFO_STREAM("X2 " << x1 * _map_resolution << " Y2 " << y1 * _map_resolution);
	
	
	line.points.push_back(p2);
	line.points.push_back(p1);
	line_strip.markers.push_back(line);
		
	_marker_pub.publish(line);
}

bool Node::obstacleSearch(int x1, int y1, int x2, int y2)
{
	int counter = 0;
	bool obst_flag = false;
	
	int x = abs(x1 - x2);
	int y = abs(y1 - y2);
	int step;
	int step_x = x1 - x2;
	int step_y = y1 - y2;
	
	if (x != 0)
	{
		step = x;
	}
	else
	{
		step = y;
	}
	
	//~ ROS_INFO_STREAM("X1 " << x1 << " Y1 " << y1 << " X2 " << x2 << " Y2 " << y2);
	for (unsigned int ii = 0; ii < step; ii ++)
	{
		if ((x1 == x2 + step && y1 == y2))
		{
			if (robot_perception.rightCell(x2 + ii, y2))
			{
				counter ++ ;
			}
		}
		if ((x1 == x2 + step && y1 == y2 + step))
		{
			if (robot_perception.rightCell(x2 + ii, y2 + ii))
			{
				counter ++ ;
			}
		}
		if ((x1 == x2 && y1 == y2 + step))
		{
			if (robot_perception.rightCell(x2, y2 + ii))
			{
				counter ++ ;
			}
		}
		if ((x1 == x2 - step && y1 == y2 + step))
		{
			if (robot_perception.rightCell(x2 - ii, y2 + ii))
			{
				counter ++ ;
			}
		}
	}
	
	
	//~ if (counter == (abs(step_x) + abs(step_y)))
	if (counter == step)
	{
		if (x1 == 600 && y1 == 300)
		{
			//~ ROS_INFO_STREAM("TA KATAFERA " << x2 << " " << y2);
		}
		obst_flag = true;
	}
	
	return obst_flag;
	
}

int Node::getNeighbourCellSize()
{
	return _neighbour_cell.size();
}

int Node::getNeighbourCellX(int ii)
{
	return _neighbour_cell[ii].x;
}

int Node::getNeighbourCellY(int ii)
{
	return _neighbour_cell[ii].y;
}

int Node::getNeighbourCellNodeCounter(int curr_map_x, int curr_map_y)
{
	int curr_node;
	for (unsigned int ii = 0; ii < getNeighbourCellSize(); ii ++)
	{
		if ((curr_map_x == getNeighbourCellX(ii)) && (curr_map_y == getNeighbourCellY(ii)))
		{
			curr_node = _neighbour_cell[ii].node_counter;
		}
	}
	
	return curr_node;
}

int Node::getStep()
{
	return _step_node;
}

void Node::visualGraph(int size, float resolution)
{
	// Visualize the nodes
    visualization_msgs::Marker marker, line;
	visualization_msgs::MarkerArray line_strip;
	
	geometry_msgs::Point p;
	
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	
	marker.type = visualization_msgs::Marker::CUBE_LIST;	
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.id = 0;	
	marker.ns = "path_planning";
	
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
  
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.00;
	
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	
	
	for (unsigned int ii = 0; ii < size; ii ++)
	{
		p.x = _neighbour_cell[ii].x * resolution;
		p.y = _neighbour_cell[ii].y * resolution;
		p.z = 0;
		//~ ROS_INFO_STREAM("X " << _neighbour_cell[ii].x << " Y " 
				//~ << _neighbour_cell[ii].y << " COUNTER " << ii);

		marker.points.push_back(p);

	}
	
	
	//~ int st = 0;
	//~ 
	//~ ROS_INFO_STREAM("SIZE " << size);
	//~ 
	//~ for (unsigned int ii = 120; ii < 150; ii ++)
	//~ {
		//~ 
		//~ for (unsigned int jj = 0; jj < size; jj ++)
		//~ {
			//~ geometry_msgs::Point p1, p2;
			//~ if (_neighbour_cell[ii].connections[ii][jj] == 1)
			//~ {
				//~ //ROS_INFO_STREAM("X " << ii << " Y " << jj);
				//~ line.header.frame_id = "map";
				//~ line.header.stamp = ros::Time::now();
				//~ 
				//~ line.type = visualization_msgs::Marker::LINE_STRIP;
				//~ line.action = visualization_msgs::Marker::ADD;
				//~ 
				//~ line.id = ii * size + jj;
				//~ 
				//~ line.ns = "graph";
				//~ line.pose.orientation.w = 1.0;
				//~ line.scale.x = 0.05;
				//~ 
				//~ 
				//~ if (st == 0)
				//~ {
					//~ line.color.a = 1.0;
					//~ line.color.r = 0.0;
					//~ line.color.g = 0.0;
					//~ line.color.b = 1.0;
					//~ st ++;
				//~ }
				//~ else
				//~ {
					//~ line.color.a = 1.0;
					//~ line.color.r = 1.0;
					//~ line.color.g = 1.0;
					//~ line.color.b = 0.0;
					//~ st = 0;
				//~ }
				//~ 
				//~ 
				//~ p1.x = _neighbour_cell[ii].x * resolution;
				//~ p1.y = _neighbour_cell[ii].y * resolution;
				//~ p1.z = 0;
				//~ ROS_INFO_STREAM("X1 " << _neighbour_cell[ii].x << " Y1 "
								//~ << _neighbour_cell[ii].y);
				//~ 
				//~ p2.x = _neighbour_cell[jj].x * resolution;
				//~ p2.y = _neighbour_cell[jj].y * resolution;
				//~ p2.z = 0;
				//~ ROS_INFO_STREAM("X2 " << _neighbour_cell[jj].x << " Y2 " 
								//~ << _neighbour_cell[jj].y);
				//~ 
				//~ 
				//~ line.points.push_back(p2);
				//~ line.points.push_back(p1);
				//~ line_strip.markers.push_back(line);
				//~ 
				//~ 
			//~ }
		//~ }
	//~ }
	
	marker.lifetime = ros::Duration();
	
	_graph_pub.publish(marker);
	//~ _graph_connections_pub.publish(line_strip);
    
}

bool Node::getObstacleSearch(int ii, int jj)
{
	if (_neighbour_cell[ii].connections[ii][jj] == 1
		|| _neighbour_cell[ii].connections[jj][ii] == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

float Node::getMinDistance(int ii, int last_curr_node) // to ii einai to trexon node, dhladh h arxh
{
	float infinity = std::numeric_limits<float>::infinity();
	float min = infinity;
	//~ ROS_INFO_STREAM(" " << _neighbour_cell[ii].x << " " << _neighbour_cell[ii].y);
	//~ ROS_INFO_STREAM(" " << _neighbour_cell[last_curr_node].x << " " << _neighbour_cell[last_curr_node].y);
	//~ ROS_INFO_STREAM("MIN222: " << min << "   " << _neighbour_cell.size());
	for (unsigned jj = 0; jj < _neighbour_cell.size(); jj ++)
	{
		if (jj != last_curr_node)
		{
			if (min > _neighbour_cell[ii].distance[ii][jj])
			{			
				min = _neighbour_cell[ii].distance[ii][jj];
			}
			if (min > _neighbour_cell[jj].distance[jj][ii])
			{
				min = _neighbour_cell[jj].distance[jj][ii];
			}
		}
	}
	
	//~ ROS_INFO_STREAM("MIN: " << min);
	return min;
}

//ean einai auto to shmeio tou xarth ston grafo
bool Node::getPixelGraph(int x, int y)
{
	bool flag = false;
	for (unsigned int ii = 0; ii < _neighbour_cell.size(); ii ++)
	{
		if ((x == _neighbour_cell[ii].x && y == _neighbour_cell[ii].y))
		{
			flag = true;
		}
	}
	
	return flag;
}

int Node::startCounter()
{
	return _start_counter;
}

int Node::goalCounter()
{
	return _goal_counter;
}








