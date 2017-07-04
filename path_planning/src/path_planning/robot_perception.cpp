#include "path_planning/robot_perception.h"

RobotPerception::RobotPerception()
{	
	_map_sub = _node.subscribe("/map", 1000, &RobotPerception::readMap, this);
	
	if(!_node.getParam("/dynamic_topic", _dynamic_topic))
    {
		ROS_ERROR("Dynamic_topic param does not exist");
	}
	_dynamic_obst_sub = _node.subscribe(_dynamic_topic, 1, &RobotPerception::dynamicC, this);
	
	_br_pub = _node.advertise<nav_msgs::OccupancyGrid>("/brushfire", 1);
	
	
	
	if(!_node.getParam("replanning_counter", _replanning_counter))
    {
		ROS_ERROR("Replanning_counter param does not exist");
	}
	_replanning_sub = _node.subscribe(_replanning_counter, 1, &RobotPerception::replanningCounter, this);
	
	_replanning_flag = false;
	
	//~ _a = 0;
	//~ _m = 0;
	
}

void RobotPerception::readMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
	_height = msg->info.height;
	_width = msg->info.width;
	_resolution = msg->info.resolution;
	_map_size = _height * _width;
	
	_index = new int[_map_size];
	
	int a = 0;
	for (unsigned int ii = 0; ii <= _map_size; ii++) 
	{
		_index[ii] = msg->data[ii];
		if (_index[ii] == 100)
			a ++;
	}
	
	//~ ROS_INFO_STREAM("Height: " << _height << "   " << a);
	//~ ROS_INFO_STREAM("Width: " << _width );
	//~ ROS_INFO_STREAM("Resolution: " << _resolution);
}

void RobotPerception::dynamicC(const art_localization_particle_filters_ogm::DynamicObstacles& msg)
{
	//~ std::vector<int> dokimi;
	_dynamic_map.clear();
	dynamic D;
	_dynamics = msg;
	
	
	
	for (unsigned int ii = 0; ii < _dynamics.dynamics.size(); ii ++)
	{
		D.x = _dynamics.dynamics[ii].x;
		D.y = _dynamics.dynamics[ii].y;
		_dynamic_map.push_back(D);
	}
	
	//~ ROS_INFO_STREAM("Y " << _dynamic_map.size());
	
	int b;
	
	
}

//~ float RobotPerception::getCurrentXPosition()
//~ {
	//~ return _curr_cell_x;
//~ }
//~ 
//~ float RobotPerception::getCurrentYPosition()
//~ {
	//~ return _curr_cell_y;
//~ }
//~ 
//~ float RobotPerception::getCurrentYaw()
//~ {
	//~ return _yaw;
//~ }

float RobotPerception::getGoalXPosition()
{
	return _goal_cell_x;
}

float RobotPerception::getGoalYPosition()
{
	return _goal_cell_y;
}

//metatrepei tis suntetagmenes x,y tou xarth se suntetagmenes keliou
int RobotPerception::worldToMap(float w_coor)
{
	int m_coor = w_coor / _resolution;
	return m_coor;
}

//metatrepei tis suntetagmenes x,y tou keliou se suntetagmenes tou xarth
void RobotPerception::mapToWorld(int m_x, int m_y)
{
	int w_x = m_x * _resolution;
	int w_y = m_y * _resolution;
}

int RobotPerception::getMapHeight()
{
	return _height;
}

int RobotPerception::getMapWidth()
{
	return _width;
}

float RobotPerception::getMapResolution()
{
	return _resolution;
}

int RobotPerception::getMapSize()
{
	return _map_size;
}

int RobotPerception::getMapCell(int ii, int jj) 
{
	int b = jj * _width + ii;
	return _index[b];
}

int* RobotPerception::getMapIndex() 
{
	return _index;
}

//elegxei an oi suntetagmenes tou keliou pou dothike anhkei ston xarth kai an einai eleuthero h oxi
bool RobotPerception::rightCell(int x, int y)
{
	bool valid = true;
	
	if (x >= _width || y >= _height || x < 0 || y < 0) 
	{
		valid = false;
	}
	else
	{
		int b = y * _width + x;
		int k;
		//~ ROS_INFO_STREAM("index " << _index[b]);
		int index = _index[b];
		//~ ROS_INFO_STREAM("TO FLAG2  " << _replanning_flag);
		if (!_replanning_flag)
		{
			for (unsigned int ii = 0; ii < _dynamic_map.size(); ii ++)
			{
				k = _dynamic_map[ii].y * _width + _dynamic_map[ii].x;
				if (k == b)
				{
					//~ ROS_INFO_STREAM("priin " << _index[k]);
					index = 0;
					//~ ROS_INFO_STREAM("index " << _index[k]);
				}
			}
		}
		
		if (index == 0 )
		{
			//~ ROS_INFO_STREAM("FREE");			
		}
		else if (index == 100)
		{
			//~ ROS_INFO_STREAM("OBSTACLE");
			valid = false;
		}
		else
		{
			//~ ROS_INFO_STREAM("UNKNOWN");
			valid = false;
		}
	}
	
	return valid;
}


void RobotPerception::replanningCounter(const std_msgs::Bool& flag)
{
	_replanning_flag = flag.data;
}

void RobotPerception::brushfire()
{
	int brushfire_flag = 0;
	int k = 0;
	int l = 0;
	int m = 0;
	int n = 0;
	ROS_INFO_STREAM("TO FLAG  " << _replanning_flag);
	int brushfire_counter = 0;
	_brushfire = new float *[_width];
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		_brushfire[ii] = new float [_height];
		for (unsigned int jj = 0; jj < _height; jj ++) 
		{
			if (!rightCell(ii, jj))  // ean einai empodio kanei to brushfire iso me mhden
			{
				_brushfire[ii][jj] = 0;
				brushfire_counter ++ ;
			}
			else
			{
				_brushfire[ii][jj] = std::numeric_limits<float>::infinity();
			}
			
		}
	}
	
	int free_cell_counter = _map_size - brushfire_counter;
	float max = 0;
	float min = _map_size + 1;
	//~ ROS_INFO_STREAM(" " << brushfire_counter);
	while (brushfire_counter < _map_size)
	{
		//~ ROS_INFO_STREAM("a ");
		brushfire_flag ++ ;
		for (unsigned int ii = 0; ii < _width; ii ++) 
		{
			int a = ii;
			//~ ROS_INFO_STREAM(" " << a << " " << a + 1);
			for (unsigned int jj = 0; jj < _height; jj ++) 
			{
				int b = jj;
				
				if (_brushfire[ii][jj] > brushfire_flag)
				{
				
						//~ ROS_INFO_STREAM(" Mpike ");
						if ((_brushfire[ii][jj - 1] == brushfire_flag - 1) 
							|| (_brushfire[ii - 1][jj - 1] == brushfire_flag - 1)
							|| (_brushfire[ii + 1][jj - 1] == brushfire_flag - 1) 
							|| (_brushfire[ii][jj + 1] == brushfire_flag - 1)
							|| (_brushfire[ii - 1][jj + 1] == brushfire_flag - 1) 
							|| (_brushfire[ii + 1][jj + 1] == brushfire_flag - 1)
							|| (_brushfire[ii - 1][jj] == brushfire_flag - 1) 
							|| (_brushfire[ii + 1][jj] == brushfire_flag - 1))
						{
							_brushfire[ii][jj] = brushfire_flag;
							brushfire_counter ++ ;
							//~ ROS_INFO_STREAM(" " << brushfire_counter);

						}
					//~ }
					if (_brushfire[ii][jj] < min)
					{
						//~ ROS_INFO_STREAM("d " << min);
						min = _brushfire[ii][jj];
					}
					if (_brushfire[ii][jj] > max)
					{
						//~ ROS_INFO_STREAM("f " << max);
						max = _brushfire[ii][jj];
					}
				}
				
			}
			//~ ROS_INFO_STREAM(" " << ii);
		}
		//~ ROS_INFO_STREAM(" " << brushfire_counter << " " << _map_size);
	}
	
	ROS_INFO_STREAM("MAX: " << max << " MIN: " << min);
	
	//gia na emfanisei to brushfire ston xarth
	nav_msgs::OccupancyGrid br;
	br.data.clear();
	
	br.info.width = _width;
	br.info.height = _height;
	br.info.resolution = _resolution;
	br.header.frame_id = "map";
	
	for (unsigned int jj = 0; jj < _height; jj ++) 
	{
		for (unsigned int ii = 0; ii < _width; ii ++) 
		{
			br.data.push_back(_brushfire[ii][jj]);
		}
	}
	
	_br_pub.publish(br);
}

int RobotPerception::getBrushfireCell(int x, int y)
{
	return _brushfire[x][y];
}


