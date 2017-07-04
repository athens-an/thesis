#include <dynamic_localization/robot_perception.h>

/**
 * @brief Default Constructor. Gets map topic param and laser topic
 * param and subcribes to these topics.
 */
RobotPerception::RobotPerception () 
{
  map_initialized = false;

  //~ ROS_INFO_STREAM("RobotPerception Constructor");
  if(!_n.getParam("/map_topic", _map_topic_param))
  {
    ROS_ERROR("Map topic param does not exist");
  }	
  _map_sub = _n.subscribe(_map_topic_param, 1,
    &RobotPerception::mapCallback, this);

  if(!_n.getParam("/robot_laser_topic", _laser_topic_param))
  {
    ROS_ERROR("Laser topic param does not exist");
  }	
  _laser_sub = _n.subscribe(_laser_topic_param, 1,
    &RobotPerception::laserRangesCallback, this);

    if(!_n.getParam("duration", _duration))
	{
		ROS_ERROR("Duration param does not exist");
	}




  if (!_n.getParam("/rfid_tags_topic", _rfid_tags_topic_param))
  {
    ROS_ERROR("Rfid_tags topic param does not exist");
  }
  _rfid_tags_sub = _n.subscribe(_rfid_tags_topic_param, 1,
    &RobotPerception::RfidTagsCallback, this);

  if (!_n.getParam("/rfid_reader_topic", _rfid_reader_topic_param))
  {
    ROS_ERROR("Rfid_reader topic param does not exist");
  }
  _rfid_reader_sub = _n.subscribe(_rfid_reader_topic_param, 1,
    &RobotPerception::RfidReaderCallback, this);
    
    
    if(!_n.getParam("/position_topic", _position_topic))
    {
		ROS_ERROR("Position_topic param does not exist");
	}
    _position_timer = _n.createTimer(ros::Duration(_duration), &RobotPerception::currentPosition, this);
    
    //~ if(!_n.getParam("/target_topic", _target_topic))
    //~ {
		//~ ROS_ERROR("Position_topic param does not exist");
	//~ }
    //~ _target_timer = _n.createTimer(ros::Duration(_duration), &RobotPerception::target, this);
    

    
    
    if(!_n.getParam("a", _a))
	{
		ROS_ERROR("Noise param a1 does not exist");
	}
    
	_dynamic_coor_pub = _n.advertise<dynamic_localization::DynamicCoordinates>("dynamic_coor", 1);
	_dynamic_obst_pub = _n.advertise<dynamic_localization::DynamicObstacles>("dynamic_obst", 1);
    
    _marker_pub = _n.advertise<visualization_msgs::Marker>("visualize_dynamic", 1);
    _dynamic_pub = _n.advertise<std_msgs::Bool>("dynamic_counter", 1);
    
    _prev_x = 6;
    _prev_y = 3;
    _x = 6;
    _y = 3;
    //~ _x = 5;
    //~ _y = 5;
    _yaw = 0;
    _prev_yaw = 0;
    
    _dx = 0;
    _dy = 0;
    _dyaw = 0;
    
    //~ _linear = 0;
    //~ _angular = 0;
    _time_flag = false;
    
    
    _counter = 0;
    _prev_linear = 0;
	_prev_angular = 0;
	_counter3 = 0;
    //~ _ranges = new float [667];
    _ranges1 = new float [667];
    _ranges.resize(667);
 
}

/**
 * @brief Gets the necessary info for the map (width, height, data)
 * @param occupancy_grid_msg [nav_msgs::OccupancyGrid] Message 
 * containing the map info
 */
void RobotPerception::mapCallback (
  nav_msgs::OccupancyGrid occupancy_grid_msg)
{	
  ROS_ERROR("Called!");  
  _map_width = occupancy_grid_msg.info.width;	
  _map_height = occupancy_grid_msg.info.height;
  _map_resolution = occupancy_grid_msg.info.resolution; 

  //~ ROS_INFO_STREAM ("RobotPerception:map_width ="<< " " << _map_width << 
  //~ " " << "RobotPerception:map_height ="<< " " << _map_height);

  _map_data = new int*[_map_width];
  for (unsigned int i = 0 ; i < _map_width ; i++)
  {
    _map_data[i] = new int[_map_height];
  }

  for (unsigned int j = 0 ; j < _map_height ; j++)
  {
    for (unsigned int i = 0 ; i < _map_width ; i++)
    {
      _map_data[i][j] =
        (int)occupancy_grid_msg.data[_map_width*j + i];
    }
  }
  
	staticMap();
	
	ROS_INFO_STREAM("TO 9 6 " << _map_data[350][550]);
	ROS_INFO_STREAM("TO 13 6 " << _map_data[150][350]);
	ROS_INFO_STREAM("TO 9 9 " << _map_data[350][150]);
	ROS_INFO_STREAM("width " << _map_width << " height " << _map_height);
	
  map_initialized = true;  
}

void RobotPerception::staticMap()
{
	nav_msgs::GetMap::Response map_response;
	
	std::string map_name;
	float resolution;
	double origin[3];
	float occupied_thresh;
	float free_thresh;
	int negate;
	
	std::string static_map_path = ros::package::getPath("dynamic_localization");

	YAML::Node static_map = YAML::LoadFile(static_map_path + "/cfg/xarths4.yaml");
	//~ YAML::Node static_map = YAML::LoadFile(static_map_path + "/cfg/sparse_obstacles.yaml");
	if (static_map.IsNull())
		ROS_INFO_STREAM("oxi");
	
	map_name = static_map["image"].as<std::string>();
	map_name = std::string(static_map_path) + "/cfg/" + map_name;
	
	nav_msgs::OccupancyGrid loadMap(const std::string &map_name);

	//~ ROS_INFO_STREAM("IMAGE " << map_name.c_str());
	
	resolution = static_map["resolution"].as<float>();
	origin[0] = static_map["origin"][0].as<double>();
	origin[1] = static_map["origin"][1].as<double>();
	origin[2] = static_map["origin"][2].as<double>();
	occupied_thresh = static_map["occupied_thresh"].as<float>();
	free_thresh = static_map["free_thresh"].as<float>();
	negate = static_map["negate"].as<int>();

	map_server::loadMapFromFile(&map_response, map_name.c_str(), resolution, 
								negate, occupied_thresh, free_thresh, origin);
	
	_static_width = map_response.map.info.width;
	_static_height = map_response.map.info.height;
	_static_resolution = map_response.map.info.resolution;
	map_response.map.header.frame_id = "map";
	
	ROS_INFO_STREAM("IMAGE " << map_response.map.header.frame_id);
	_static_data = new int* [_static_width];
	for (unsigned int ii = 0; ii < _static_width; ii ++)
	{
		_static_data[ii] = new int[_static_height];
	}
			
	for (unsigned int jj = 0; jj < _static_height; jj ++)
	{
		for (unsigned int ii = 0; ii < _static_width; ii ++)
		{
			_static_data[ii][jj] = map_response.map.data[_static_width * jj + ii];
		}
	}
	
	//~ ROS_INFO_STREAM("STATIC WIDTH: " << _static_width);
	//~ ROS_INFO_STREAM("STATIC HEIGHT: " << _static_height);
	//~ ROS_INFO_STREAM("STATIC RESOLUTION: " << _static_resolution);
	
}

void RobotPerception::currentPosition(const ros::TimerEvent& e)
{
	//~ _current_time = ros::Time::now();
	//~ _dt1 = _current_time - _previous_time;
	try
	{
		tf::StampedTransform transform;
		
		_listener.lookupTransform(_map_topic_param, _position_topic, ros::Time(0), transform);
		
		_x = transform.getOrigin()[0]; //transform.getOrigin().x()
		_y = transform.getOrigin()[1];
		_yaw = tf::getYaw(transform.getRotation());
		
		//~ ROS_INFO_STREAM("CURRENT POSITION: " << _x << " " 
							//~ << _y << " " << _yaw);					
	}
	
	
	
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(2.0).sleep();
	}
	
	findLaserXY();
	_counter3 = _counter3 + 1;
	//~ _dx = _dy = _dyaw = 0;
	
}



void RobotPerception::findLaserXY()
{
	float range_angle; // einai h gwnia ths aktinas se sxesh me to rompot (ton eutheia aksona tou rompot))
	float laser_x;
	float laser_y;
	int countD;
	int countS;
	std::vector <laser> dynamic_test;
	_dynamic_ranges.clear(); 
	dynamic_test.clear(); 
	laser D;
		
	if (_time_flag == true)
	{
		_dt = ros::Time::now() - _current_time;
		_current_time = ros::Time::now();
	}
	else
	{
		_current_time = ros::Time::now();
		_time_flag = true;
	}
	
	float x;
	float y;
	float yaw = _yaw;
	if (fabs(_angular) < 0.0000000001)
	{
		_dx = _prev_linear * _dt.toSec() * cos(_yaw);
		_dy = _prev_linear * _dt.toSec() * sin(_yaw);
		_dyaw = 0;
	}
	else
	{
		_dx = 0;
		_dy = 0;
		_dyaw = _angular * (_dt.toSec());		
	}

	_x = _x + _dx;
	_y = _y + _dy;
	_yaw = _yaw + _dyaw;
	
	//~ ROS_INFO_STREAM("x " << _x << " y " << _y << " yaw " << _yaw);
	//~ ROS_INFO_STREAM("dx " << _dx << " dy " << _dy << " dyaw " << _dyaw << " dt " << _dt);
	//~ ROS_INFO_STREAM("dt " << _dt);
	
	int k = 20;

	for (unsigned int ii = 0; ii < _laser_ranges.size(); ii ++)
	{
		countS = 0;
		countD = 0;
		
		range_angle = ii * _increment + _angle_min + _yaw;
		laser_x = _x + _laser_ranges[ii] * cos(range_angle);
		laser_y = _y + _laser_ranges[ii] * sin(range_angle);
		
		laser_x = laser_x / _map_resolution;
		laser_y = laser_y / _map_resolution;
		
		//~ D.x = laser_x;
		//~ D.y = laser_y;
		//~ _dynamic_ranges.push_back(S);
		
		for (unsigned int jj = 0; jj < k; jj ++)
		{
			for (unsigned int zz = 0; zz < k; zz ++)
			{
				
				if (((int)laser_x + jj - k / 2 >= 0) && ((int)laser_y + zz - k / 2 >= 0)
					&& ((int)laser_x + jj - k / 2 <= _map_width) && ((int)laser_y + zz - k / 2 <= _map_height))
				{
					
					if (_static_data[(int)laser_x + jj - k / 2][(int)laser_y + zz - k / 2] != _map_data[(int)laser_x + jj - k / 2][(int)laser_y + zz - k / 2])
					{
						//~ ROS_INFO_STREAM("mpike");
						countD ++ ;
						//~ for (unsigned kk = 0; kk < _dynamic_ranges.size(); kk ++)
						//~ {
							//~ if ((_dynamic_ranges[kk].x != (int)laser_x + jj - k / 2) && (_dynamic_ranges[kk].y != (int)laser_y + zz - k / 2))
							//~ {
								//~ D.laser_id = ii;
								//~ D.x = (int)laser_x + jj - k / 2;
								//~ D.y = (int)laser_y + zz - k / 2;
								//~ _dynamic_ranges.push_back(D);
								// ROS_INFO_STREAM("??? " << S.x << "   " << S.y);
							//~ }
						//~ }
						//~ if (_dynamic_ranges.size() == 0)
						//~ {
							D.laser_id = ii;
							D.x = (int)laser_x + jj - k / 2;
							D.y = (int)laser_y + zz - k / 2;
							dynamic_test.push_back(D);
						//~ }
					}
				}
			}
		}
		
		//~ ROS_INFO_STREAM("DUNAMIKA " << countD << " " << laser_x);
		if (countD > 0)
		{
			//~ ROS_INFO_STREAM("DUNAMIKA " << countD);
			_ranges[ii] = -1;
		}
		else
		{
			_ranges[ii] = _laser_ranges[ii];
		}
		//~ visual(_x - _dx, _y - _dy, laser_x, laser_y, ii);
	}
	
	int metrhths = 0;
	int w = 0;
	for (unsigned int ii = 0; ii < dynamic_test.size(); ii ++)
	{
		w = 0;
		if (metrhths == 0)
		{
			D.laser_id = ii;
			D.x = dynamic_test[ii].x;
			D.y = dynamic_test[ii].y;
			_dynamic_ranges.push_back(D);
			metrhths = 1;
		}
		else
		{
			for (unsigned int jj = 0; jj < _dynamic_ranges.size(); jj ++)
			{
				if (dynamic_test[ii].x == _dynamic_ranges[jj].x && dynamic_test[ii].y == _dynamic_ranges[jj].y)
				{
					w ++;
				}
			}
			if (w == 0)
			{
				D.laser_id = ii;
				D.x = dynamic_test[ii].x;
				D.y = dynamic_test[ii].y;
				_dynamic_ranges.push_back(D);
			}
		}
	}
	
	dynamic_localization::DynamicCoordinates dynamic_coor;
	dynamic_localization::DynamicObstacles dynamic_obst;
	for (unsigned int ii = 0; ii < _dynamic_ranges.size(); ii ++)
	{
		dynamic_coor.x = _dynamic_ranges[ii].x;
		dynamic_coor.y = _dynamic_ranges[ii].y;
		dynamic_obst.dynamics.push_back(dynamic_coor);
	}
	
	_dynamic_obst_pub.publish(dynamic_obst);
	
	//~ ROS_INFO_STREAM("BGHKE");
	findStaticLaser();
	
	visual();

	
	
	
	_prev_linear = _linear;
	_prev_angular = _angular;
	_prev_x = _x;
	_prev_y = _y;
	_prev_yaw = yaw;
	//~ _prev_target_yaw = _target_yaw;
	
	_counter3 = 0;
	
}

void RobotPerception::findStaticLaser()
{
	_static_ranges.clear();
	laser S;
	
	for (unsigned int ii = 0; ii < _ranges.size(); ii ++)
	{
		if (_ranges[ii] != -1)
		{
			S.laser_id = ii;
			S.laser_range = _ranges[ii];
			_static_ranges.push_back(S);
		}
	}
	
	//~ ROS_INFO_STREAM("TA STATIKA EINAI: " << _static_ranges.size() << " " << _laser_ranges.size() - _static_ranges.size());
	
	tooManyDynamic();
	
	_static_laser_ranges.resize(_static_ranges.size());
	_static_id_ranges.resize(_static_ranges.size());
	
	for (unsigned int ii = 0; ii < _static_laser_ranges.size(); ii ++)
	{
		_static_laser_ranges[ii] = _static_ranges[ii].laser_range;
		_static_id_ranges[ii] = _static_ranges[ii].laser_id;
	}
	
	
}

void RobotPerception::visual()
{
    visualization_msgs::Marker marker;
	geometry_msgs::Point p;
	
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.id = 0;
	marker.ns = "laser";
	
	//~ ROS_INFO_STREAM("SIZE: " << _dynamic_ranges.size());
	for (unsigned int ii = 0; ii < _dynamic_ranges.size(); ii ++)
	{
		p.x = _dynamic_ranges[ii].x * _map_resolution;
		p.y = _dynamic_ranges[ii].y * _map_resolution;
		p.z = 0;
		//~ ROS_INFO_STREAM("!!!!!!! " << p.x / _map_resolution << " " << p.y / _map_resolution);
		 //~ << " " << ii + 270);
		marker.points.push_back(p);	
	}

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
  
	marker.scale.x = 0.03;
	marker.scale.y = 0.03;
	marker.scale.z = 0.00;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.5;
	marker.color.b = 0.0;
	
	marker.lifetime = ros::Duration();
	
	_marker_pub.publish(marker);
	
}

void RobotPerception::visual(float x, float y, float x1, float y1, int ii)
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
	
	p1.x = x;
	p1.y = y;
	p1.z = 0;
	//~ ROS_INFO_STREAM("X1 " << x1 << " Y1 " << y1);
	
	p2.x = x1 * _map_resolution;
	p2.y = y1 * _map_resolution;
	p2.z = 0;
	// ROS_INFO_STREAM("X2 " << x1 * _map_resolution << " Y2 " << y1 * _map_resolution);
	
	
	line.points.push_back(p2);
	line.points.push_back(p1);
	line_strip.markers.push_back(line);
		
	_marker_pub.publish(line);
}

void RobotPerception::RfidTagsCallback (stdr_msgs::RfidTagVector
  rfid_tag_msg)
{
  _rfid_tags = rfid_tag_msg.rfid_tags;

  std::ofstream data_file;
  data_file.open("/home/aspa/catkin_ws/src/thesis/localization_project/cfg/example.txt");
  for (unsigned int i = 0 ; i < _rfid_tags.size(); i++)
  {
    data_file << _rfid_tags[i].tag_id << "\t" <<
      _rfid_tags[i].pose.x << "\t" << _rfid_tags[i].pose.y << "\n"; 
  }
  data_file.close();

  std::string line;
  std::ifstream file ("/home/aspa/catkin_ws/src/thesis/localization_project/cfg/example.txt");
  if (file.is_open())
  {
    while (getline (file,line))
    {
      std::string id;
      float x, y;
      std::istringstream ss(line);
      ss >> id >> x >> y;
      _rfid_tags_id.push_back(id);
      _rfid_tags_x.push_back(x);
      _rfid_tags_y.push_back(y);
    }
    file.close();
  }	    
}

void RobotPerception::RfidReaderCallback (stdr_msgs::RfidSensorMeasurementMsg
  rfid_reader_msg)
{
  _rfid_pose.clear();
  _rfid_ids = rfid_reader_msg.rfid_tags_ids;
  _rfid_msgs = rfid_reader_msg.rfid_tags_msgs;
  RfidPose();
}

void RobotPerception::RfidPose()
{
  for (unsigned int i = 0 ; i < _rfid_ids.size() ; i++)
  {
    for (unsigned int j = 0 ; j < _rfid_tags_id.size() ; j++)
    {
      if (!_rfid_ids[i].compare(_rfid_tags_id[j]))
      {
        std::vector<float> temp;
        temp.push_back(_rfid_tags_x[j]);
        temp.push_back(_rfid_tags_y[j]);
        _rfid_pose.push_back(temp);
      }
    }
  }
  //~ for (unsigned int i = 0 ; i < _rfid_pose.size() ; i++)
  //~ {
  //~ ROS_INFO_STREAM (" i = " << i );
  //~ ROS_INFO_STREAM(" Pose x = " << _rfid_pose[i][0] << " y = " << _rfid_pose[i][1]);
  //~ }
}

/**
 * @brief Gets the laser ranges
 * @param laser_scan_msg [sensor_msgs::LaserScan] Message 
 * containing the laser ranges info
 */
void RobotPerception::laserRangesCallback(
  sensor_msgs::LaserScan laser_scan_msg) 
{
  _increment = laser_scan_msg.angle_increment;
  _angle_min = laser_scan_msg.angle_min;
  _laser_ranges = laser_scan_msg.ranges;
  _max_range = laser_scan_msg.range_max;
  
  //~ _ranges = new float [_laser_ranges.size()];
  for (unsigned int i = 0 ; i < _laser_ranges.size() ; i ++)
  {
    if (_laser_ranges[i] > _max_range) {
      _laser_ranges[i] = _max_range;
      //~ _ranges[i] = _laser_ranges[i];
	}
  }
  
}

/**
 * @brief Returns the map width
 * @return unsigned int - Map width
 */
unsigned int RobotPerception::getMapWidth()
{
  return _map_width;
}

/**
 * @brief Returns the map height
 * @return unsigned int - Map height
 */	
unsigned int RobotPerception::getMapHeight()
{
  return _map_height;
}

float RobotPerception::getMapResolution()
{
  return _map_resolution;
} 

/**
 * @brief Returns the occupancy state of a map cell
 * @param i [int] Coordinate x of the map data array 
 * @param j [int] Coordinate y of the map data array 
 * @return int - Map cell occupancy state 
 */
int RobotPerception::getMapCell ( int i, int j ) 
{
  return _map_data[i][j];
}

/**
 * @brief Returns the map data
 * @return int** - Map occupancy data array
 */
int** RobotPerception::getMapData () 
{
  return _map_data;
}

int** RobotPerception::getStaticData () 
{
  return _static_data;
}

/**
 * @brief Returns the laser ranges
 * @return float* - Laser ranges array
 */
std::vector<float> RobotPerception::getLaserRanges() 
{
  return _laser_ranges;
}

std::vector <float> RobotPerception::getStaticRanges()
{
	//~ ROS_INFO_STREAM("TO MEGETHOS TOY VECTOR EINAI: " << _static_ranges.size());
	return _static_laser_ranges;
}

std::vector <float> RobotPerception::getStaticIdRanges()
{
	//~ ROS_INFO_STREAM("TO MEGETHOS TOY VECTOR: " << _static_ranges.size());
	return _static_id_ranges;
}

float RobotPerception::getRangeMax()
{
  return _max_range;
}

float RobotPerception::getAngleIncrement()
{
  return _increment;
}

float RobotPerception::getAngleMin()
{
  return _angle_min;
}

std::vector<std::string> RobotPerception::getRfidIds()
{
  return _rfid_ids;
}

std::vector<std::string> RobotPerception::getRfidMsgs()
{
  return _rfid_msgs;
}

std::vector<std::vector<float> > RobotPerception::getRfidPose()
{
  return _rfid_pose;
}



void RobotPerception::tooManyDynamic()
{
	std_msgs::Bool flag;
	
	int allDynamicRanges = 0;
	for (unsigned int ii = 0; ii < _laser_ranges.size(); ii ++)
	{
		if (_ranges[ii] < 0)
			allDynamicRanges ++ ;
	}
	
	//~ ROS_INFO_STREAM("ALLDYNAMICRANGES: " << allDynamicRanges);
	if (allDynamicRanges >= (_laser_ranges.size() * 0.65))
	{
		flag.data = true; // gia na mhdenizei thn taxuthta sto path planning
		//~ flag.data = false;
		ROS_INFO_STREAM("TOO MANY DYNAMIC LASER RANGES");
		for (unsigned int ii = 0; ii < _laser_ranges.size(); ii ++)
		{
			if (_ranges[ii] >= 0)
			{
				_static_ranges.clear();
			}
		}
	}
	else
	{
		flag.data = false;
	}
	//~ ROS_INFO_STREAM("sfvsf " << _static_ranges.size());
	_dynamic_pub.publish(flag);
	
}







