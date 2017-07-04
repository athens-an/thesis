#include "path_planning/path_planning.h"
#include <tf/transform_broadcaster.h>

Planner::Planner()
{
	_goal_counter = 0;
	//~ _service2 = _node.advertiseService("start", &Planner::start, this);
	_service1 = _node.advertiseService("goal", &Planner::goal, this);
	
	if(!_node.getParam("/map_topic", _map_topic))
    {
		ROS_ERROR("Map_topic param does not exist");
	}
	
	if(!_node.getParam("/velocity_topic", _velocity_topic))
    {
		ROS_ERROR("Velocity_topic param does not exist");
	}
	
	if(!_node.getParam("/position_topic", _position_topic))
    {
		ROS_ERROR("Position_topic param does not exist");
	}
	
	//~ if(!_node.getParam("/target_topic", _target_topic))
    //~ {
		//~ ROS_ERROR("Target_topic param does not exist");
	//~ }
	
	if(!_node.getParam("duration", _duration))
    {
		ROS_ERROR("Duration param does not exist");
	}
	if(!_node.getParam("time", _time))
    {
		ROS_ERROR("Duration param does not exist");
	}
	
	if(!_node.getParam("distance_limit", _distance_limit))
    {
		ROS_ERROR("Distance_limit param does not exist");
	}
	
	if(!_node.getParam("yaw_limit", _yaw_limit))
    {
		ROS_ERROR("Yaw_limit param does not exist");
	}
	
	if(!_node.getParam("dis_to_target", _dis_to_target))
    {
		ROS_ERROR("Dis_to_target param does not exist");
	}
	
	if(!_node.getParam("brushfire_const", _brushfire_const))
    {
		ROS_ERROR("Brushfire_const param does not exist");
	}
	
	if(!_node.getParam("brushfire_limit", _brushfire_limit))
    {
		ROS_ERROR("Brushfire_limit param does not exist");
	}
	
	if(!_node.getParam("dynamic_counter", _dynamic_counter))
    {
		ROS_ERROR("Dynamic_counter param does not exist");
	}
	_dynamic_sub = _node.subscribe(_dynamic_counter, 1, &Planner::dynamicCounter, this);
	
	_path_pub = _node.advertise<nav_msgs::Path>("/move_base_simple/path", 10);
	_marker_pub = _node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	_vel_pub = _node.advertise<geometry_msgs::Twist>(_velocity_topic, 1);
	//~ _target_pub = _node.advertise<geometry_msgs::Twist>(_target_topic, 1);
	
	_replanning_pub = _node.advertise<std_msgs::Bool>("replanning_counter", 1);
	
	
	//transform a point once every second
	_position_timer = _node.createTimer(ros::Duration(_time), &Planner::currentPosition, this);
	

	_flag_time = false;
	_target_counter = 0;
	
	last_curr_node = 0;
	prev_goal_map_x = 300;
	prev_goal_map_y = 150;
	
}


bool Planner::start(path_planning::startRequest &req, 
					path_planning::startResponse &res)
{
	ROS_INFO_STREAM("start");
	res.success = true;
	return true;
}

bool Planner::goal(path_planning::goalRequest &req, 
					path_planning::goalResponse &res)
{

	
	
	
	// gia synexomenous stoxous
	std::vector <cell> best_path;
	best_path.clear();
	
	_replanning_counter = 0;
	
	
	
	std::string _path = ros::package::getPath("path_planning");

	YAML::Node baseNode = YAML::LoadFile(_path + "/cfg/goals.yaml");
	if (baseNode.IsNull())
		return false;
	
	YAML::Node goalNode = baseNode["goal"];
	
	_goal_cell_x = goalNode["x"][_goal_counter].as<int>();
	_goal_cell_y = goalNode["y"][_goal_counter].as<int>();
	
	ROS_INFO_STREAM("GOAL X: " << _goal_cell_x << " GOAL Y: " << _goal_cell_y);
	
	int curr_node;

	int curr_map_x = robot_perception.worldToMap(_curr_cell_x);
	int curr_map_y = robot_perception.worldToMap(_curr_cell_y);
	int goal_map_x = robot_perception.worldToMap(_goal_cell_x);
	int goal_map_y = robot_perception.worldToMap(_goal_cell_y);
	//~ ROS_INFO_STREAM("SLJDB " << curr_map_x << " " << curr_map_y);
	if (robot_perception.rightCell(goal_map_x, goal_map_y))
	{
		best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
		ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
							<< " and a goal: " << _goal_cell_x << " " << _goal_cell_y);
		
		if (_final_goal)
		{
			publishNewPath(best_path);
			

			res.success = true;
		}
		else
		{
			
			ROS_INFO_STREAM("DEN MPOREI NA PAEI " << _goal_counter);
			res.success = false;
			return 1;
		}
		_goal_counter ++;
	}
	else
	{
		ROS_INFO_STREAM("WRONG GOAL");
		_goal_counter ++;
		res.success = false;
		return 1;
	}
	
	return true;
}

void Planner::publishNewPath(const std::vector <cell>& best_path)
{
	//~ ROS_INFO_STREAM("3");
	if (_final_goal)
	{
		//~ ROS_INFO_STREAM("4");
		nav_msgs::Path plan;
		geometry_msgs::PoseStamped pose;
	
		plan.header.frame_id = "/map";
				
		for (unsigned int ii = 0; ii < best_path.size(); ii ++)
		{
	
			// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
			pose.pose.position.x = best_path[ii].x * robot_perception.getMapResolution();
			pose.pose.position.y = best_path[ii].y * robot_perception.getMapResolution();
			pose.pose.position.z = 0.0;
	
			// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;
						
			plan.poses.push_back(pose);
			curr_node = node_obj.getNeighbourCellNodeCounter(best_path[ii].x, best_path[ii].y);
			ROS_INFO_STREAM("POSE_X " << pose.pose.position.x << " POSE_Y " << pose.pose.position.y << " NODE " << curr_node);
				
		}
		_path_pub.publish(plan);
	}
}




bool Planner::random()
{
	std::vector <cell> best_path;
	best_path.clear();
	
	int w = robot_perception.getMapWidth() * robot_perception.getMapResolution();
	int h = robot_perception.getMapHeight() * robot_perception.getMapResolution();
	
	int curr_map_x = robot_perception.worldToMap(_curr_cell_x);
	int curr_map_y = robot_perception.worldToMap(_curr_cell_y);
	int goal_cell_x = robot_perception.getGoalXPosition();
	int goal_cell_y = robot_perception.getGoalYPosition();
	
	goal_cell_x = rand() % w;
	goal_cell_y = rand() % h;
	
	// ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
	int goal_map_x = robot_perception.worldToMap(goal_cell_x); // map(pixel)
	int goal_map_y = robot_perception.worldToMap(goal_cell_y);
	
	while (!robot_perception.rightCell(goal_map_x, goal_map_y))
	{
		goal_cell_x = rand() % w;
		goal_cell_y = rand() % h;
	
		
		// ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
		// _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
		// _goal_map_y = worldToMap(_goal_cell_y);
		
		int goal_map_x = robot_perception.worldToMap(goal_cell_x); // map(pixel)
		int goal_map_y = robot_perception.worldToMap(goal_cell_y);
		
	}
	
	// best_path = path(_curr_cell_x, _curr_cell_y, _goal_map_x, _goal_map_y);
	// ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
						// << " and a goal: " << _goal_cell_x << " " << _goal_cell_y);
						
	best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
					<< " and a goal: " << robot_perception.getGoalXPosition() << " " << robot_perception.getGoalYPosition());
		
	nav_msgs::Path plan;
	geometry_msgs::PoseStamped pose;


	plan.header.frame_id = "/map";
		
		
	for (unsigned int ii = 0; ii < best_path.size(); ii ++)
	{

		// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
		pose.pose.position.x = best_path[ii].x * robot_perception.getMapResolution();
		pose.pose.position.y = best_path[ii].y * robot_perception.getMapResolution();
		pose.pose.position.z = 0.0;

		// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;

		plan.poses.push_back(pose);
			
	}
	
	_path_pub.publish(plan);
	return true;
}

void Planner::currentPosition(const ros::TimerEvent& e)
{
	try
	{
		tf::StampedTransform transform;
		
		_listener.lookupTransform(_map_topic, _position_topic, ros::Time(0), transform);
		
		_curr_cell_x = transform.getOrigin()[0]; //transform.getOrigin().x()
		_curr_cell_y = transform.getOrigin()[1];
		_curr_cell_z = transform.getOrigin()[2];
		_yaw = tf::getYaw(transform.getRotation());
		
		//~ ROS_INFO_STREAM("CURRENT POSITION: " << _curr_cell_x << " " 
							//~ << _curr_cell_y << " " << _yaw);					
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(2.0).sleep();
	}
	
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	transform.setOrigin( tf::Vector3(_curr_cell_x, _curr_cell_y, 0.0) );
	q.setRPY(0, 0, _yaw);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "current_pose"));
	
	
}

void Planner::dynamicCounter(const std_msgs::Bool& flag)
{
	_dynamic_flag = flag.data;
	//~ while (_dynamic_flag)
	//~ {
		//~ ROS_INFO_STREAM("DYNAMIC FLAG: " << _dynamic_flag);
	//~ }
}

//calculate h(x), manhattan - euclidean distance
float Planner::calculateHScore (int curr_map_x, int curr_map_y, 
								int goal_map_x, int goal_map_y) 
{
	float h_score = (goal_map_x - curr_map_x) * (goal_map_x - curr_map_x) 
						+ (goal_map_y - curr_map_y) * (goal_map_y - curr_map_y);
	h_score = sqrt(h_score);
	return h_score;
}

std::vector <cell> Planner::path (int curr_map_x, int curr_map_y, 
									int goal_map_x, int goal_map_y)
{
	std::vector <cell> open_list;
	std::vector <cell> closed_list;
	std::vector <cell> best_path;
	std::vector <cell> came_from_list;
	
	open_list.clear();
	closed_list.clear();
	best_path.clear();
	came_from_list.clear();
	
	cell C; // gia closed_list kai came_from_list
	cell O; // gia open_list
	cell C_F; // gia came_from_list
	
	ROS_INFO_STREAM(" " << curr_map_x << " " << curr_map_y << " " << goal_map_x << " " << goal_map_y);
	
	
	std_msgs::Bool flag;
	bool flag_test = false;
	
	float infinity = std::numeric_limits<float>::infinity();
		
	g_score = new float *[robot_perception.getMapWidth()];
	
	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		g_score[ii] = new float [robot_perception.getMapHeight()];
		for (unsigned int jj = 0; jj < robot_perception.getMapHeight(); jj ++) 
		{
			g_score[ii][jj] = robot_perception.getMapSize() + 1;
		}
	}
		
	f_score = new float *[robot_perception.getMapWidth()];
	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		f_score[ii] = new float [robot_perception.getMapHeight()];
		for (unsigned int jj = 0; jj < robot_perception.getMapHeight(); jj ++) 
		{
			f_score[ii][jj] = std::numeric_limits<float>::infinity();
		}
	}
	
	if (_replanning_counter == 0)
	{
		flag_test = true;
		curr_map_x = prev_goal_map_x;
		curr_map_y = prev_goal_map_y;
	}
	
	flag.data = flag_test;
	_replanning_pub.publish(flag);
	
	
	_replanning_counter = 1;
	if (flag_test)
	{
		robot_perception.brushfire();
	}
	
	//~ curr_map_x = prev_goal_map_x;
	//~ curr_map_y = prev_goal_map_y;
	
	node_obj.createNodes(robot_perception.getMapWidth(), robot_perception.getMapHeight(), 
						robot_perception.getMapResolution(), robot_perception.getMapSize(),
						curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	
	//~ int curr_node;
	//~ int last_curr_node = 0;
	curr_node = node_obj.getNeighbourCellNodeCounter(curr_map_x, curr_map_y); // o arithmos node ths trexousas theshs
	
	//prosthetw thn trexousa thesh sthn open list
	C.x = curr_map_x;
	C.y = curr_map_y;
	open_list.push_back(C);
	
	int c_f_counter = 0; // gia na kserei apo poio keli egine h eksaplwsh
	int g_counter = 0; // counter gia ta g_score
	int counter = 0; // gia na kserei poio keli tha diagrapsei apo thn open list
		
	g_score[curr_map_x][curr_map_y] = 0; //gia to keli sto opoio vriskomaste
	float h_score = calculateHScore(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	f_score[curr_map_x][curr_map_y] = g_score[curr_map_x][curr_map_y] + h_score;
	
	float nodes_distance = node_obj.getMinDistance(curr_node, last_curr_node);
	//~ ROS_INFO_STREAM("F " << f_score[curr_map_x][curr_map_y] << " G " << g_score[curr_map_x][curr_map_y] << 
					//~ " H " << h_score << " " << curr_map_x << " " << curr_map_y);
	
	//~ ROS_INFO_STREAM(" " << curr_node << " " << last_curr_node << " " << nodes_distance);
	ROS_INFO_STREAM(" " << open_list.size() << " " << curr_map_x << " " << curr_map_y << " " << goal_map_x << " " << goal_map_y);
	
	while (!open_list.empty() && !(curr_map_x == goal_map_x && curr_map_y == goal_map_y))
	{
		open_list.erase(open_list.begin() + counter);
		closed_list.push_back(C);
			
		float min = infinity;
		float new_g;
		
		last_curr_node = curr_node;
		//~ ROS_INFO_STREAM("mpike sto path " << nodes_distance);
		
		//~ nodes_distance = (int)nodes_distance;
		
		for (unsigned int ii = nodes_distance; ii <= 3 * nodes_distance; ii = ii + nodes_distance) 
		{
			//~ ROS_INFO_STREAM("4 " << nodes_distance);
			for (unsigned int jj = nodes_distance; jj <= 3 * nodes_distance; jj = jj + nodes_distance) 
			{	
				int mx = curr_map_x + ii - 2 * nodes_distance; //to x tou geitona
				int my = curr_map_y + jj - 2 * nodes_distance; //to y tou geitona
				int brushfire = robot_perception.getBrushfireCell(mx, my);
				//~ ROS_INFO_STREAM("5 " << mx << " " << my << " " << brushfire);
				
				//elegxw an anhkei ston grafo (eleythero) h oxi
				//~ if (node_obj.getPixelGraph(mx, my))  // gia ta peiramata xwris brushfire
				if (node_obj.getPixelGraph(mx, my) && brushfire > _brushfire_limit)
				{
					int neighbour_node = node_obj.getNeighbourCellNodeCounter(mx, my);
						
					if (g_score[mx][my] == robot_perception.getMapSize() + 1) //den exei epektathei akoma (den einai to trexon dhladh)
					{
						//elegxei tous geitones an enwnontai
						if (node_obj.getObstacleSearch(curr_node, neighbour_node))
						{
							//gia ta diagwnia kelia kelia
							if (!(mx == curr_map_x || my == curr_map_y))
							{
								g_score[mx][my] = g_score[curr_map_x][curr_map_y] + 1.4;
								//~ ROS_INFO_STREAM(" G1 " << g_score[mx][my] << " " << mx << " " << my);
							}
							else //gia ta panw-katw, deksia-aristera kelia
							{
								g_score[mx][my] = g_score[curr_map_x][curr_map_y] + 1;
								//~ ROS_INFO_STREAM(" G " << g_score[mx][my] << " " << mx << " " << my);
							}
							
							//~ ROS_INFO_STREAM("Not visited yet");
							h_score = calculateHScore(mx, my, goal_map_x, goal_map_y);
							f_score[mx][my] = g_score[mx][my] + h_score + _brushfire_const / brushfire;
							//~ f_score[mx][my] = g_score[mx][my] + h_score;  // gia ta peiramata xwris brushfire
							
							//pernaei sthn open list to geitoniko diathesimo keli
							O.x = mx;
							O.y = my;
							O.f_score = f_score[mx][my];
							O.cf_x = curr_map_x; //apo auto egine h epektash
							O.cf_y = curr_map_y;
							O.counter = c_f_counter;
							open_list.push_back(O);
						}
					}
					else //to exei ksanaepiskeuthei
					{
						if (!(ii - 2 * nodes_distance == 0) && !(jj - 2 * nodes_distance) == 0)
						{
							for (unsigned int zz = 0; zz < open_list.size(); zz ++)
							{
								if (mx == open_list[zz].x && my == open_list[zz].y)
								{
									//gia ta diagwnia kelia kelia
									if (!(mx == curr_map_x || my == curr_map_y))
									{
										new_g = g_score[curr_map_x][curr_map_y] + 1.4;
									}
									else //gia ta panw-katw, deksia-aristera kelia
									{
										new_g = g_score[curr_map_x][curr_map_y] + 1;
									}
										
									if (new_g < g_score[mx][my])
									{
										g_score[mx][my] = new_g;
										f_score[mx][my] = g_score[mx][my] + calculateHScore(mx, my, goal_map_x, goal_map_y) 
															+ _brushfire_const / brushfire;
															//~ ;  // gia ta peiramata xwris brushfire
										open_list[zz].f_score = f_score[mx][my];
										open_list[zz].cf_x = curr_map_x;
										open_list[zz].cf_y = curr_map_y;
										open_list[zz].counter = c_f_counter;
									}
								}
							}
							//~ ROS_INFO_STREAM("Has visited");
						}
					}
					
				}
				else
				{
					//~ ROS_INFO_STREAM("DEN ANHKEI STON GRAFO");
				}
			}
		}
		
		//upologizei to mikrotero f_score apo ta kelia pou exei episkeuthei mexri stigmhs
		for (unsigned int zz = 0; zz < open_list.size(); zz ++)
		{
			//~ ROS_INFO_STREAM("2");
			if (min > open_list[zz].f_score)
			{	
				min = open_list[zz].f_score;
				curr_map_x = open_list[zz].x;
				curr_map_y = open_list[zz].y;
				curr_node = node_obj.getNeighbourCellNodeCounter(curr_map_x, curr_map_y);
				counter = zz;
				//~ ROS_INFO_STREAM(" D " << robot_perception.getBrushfireCell(curr_map_x, curr_map_y) << " F " 
									//~ << min << " " << curr_map_x << " " << curr_map_y);
			}
		}
		
		ROS_INFO_STREAM("Next cell " << curr_node << " " << last_curr_node);
		ROS_INFO_STREAM("Next cell " << curr_map_x << " " << curr_map_y << " " << counter << " " << open_list.size());
		C.x = curr_map_x;
		C.y = curr_map_y;
		
		C_F.x = curr_map_x;
		C_F.y = curr_map_y;
		C_F.cf_x = open_list[counter].cf_x;
		C_F.cf_y = open_list[counter].cf_y;
		C_F.f_score = open_list[counter].f_score;
		C_F.counter = open_list[counter].counter;
		c_f_counter ++ ;
		
		//~ if (nodes_distance == 10)
			//~ ROS_INFO_STREAM("came from list  " << C.x << " " << C.y << " " << C_F.cf_x << " " << C_F.cf_y);
		
		came_from_list.push_back(C_F);
		
		nodes_distance = node_obj.getMinDistance(curr_node, last_curr_node);
		//~ ROS_INFO_STREAM("3");
	}
	//~ ROS_INFO_STREAM("2 " << open_list.size());
	C.x = goal_map_x;
	C.y = goal_map_y;
	C.counter = came_from_list.size();
	
	closed_list.push_back(C);
	//came_from_list.push_back(C);
	
	//~ last_curr_node = node_obj.goalCounter();
	last_curr_node = curr_node;
	//~ curr_node = last_curr_node;
	prev_goal_map_x = goal_map_x;
	prev_goal_map_y = goal_map_y;
	
	
	if (finalGoal(came_from_list, goal_map_x, goal_map_y))
	{
		best_path = reconstructPath(came_from_list, goal_map_x, goal_map_y);
	}

	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		delete [] g_score[ii];
		delete [] f_score[ii];
	}
	
	delete [] g_score;
	delete [] f_score;
	//~ delete [] _index;
	
	//~ _goal_counter ++;
	
	return best_path;
}

std::vector <cell> Planner::reconstructPath (const std::vector <cell>& came_from_list, 
												int goal_map_x, int goal_map_y)
{
	std::vector <cell> best_path;
	subobjective_path.clear();
	
	best_path.clear();
	
	int count = came_from_list.size();

	cell B;
	cell S;
	
	int sub_target_x = goal_map_x;
	int current_x = goal_map_x;
	int current_y = goal_map_y;
	int sub_target_y = goal_map_y;
		
	int counter1 = 0; // for sub_target
	_dokimi = 0;
	int end_x;
	int end_y;
	ROS_INFO_STREAM("CAME FROM LIST " << came_from_list.size());
	while (!(count == 0))
	{
		//~ ROS_INFO_STREAM("SIZE " << count);
		current_x = came_from_list[count - 1].x;
		current_y = came_from_list[count - 1].y;
		count = came_from_list[count - 1].counter;
				
		//~ ROS_INFO_STREAM("reConstructorPath " << current_x << " " << current_y << " " << count);
		B.x = current_x;
		B.y = current_y;
		best_path.push_back(B);
		
		if (counter1 % 1 == 0)
		{
			end_x = current_x;
			end_y = current_y;
			S.x = current_x;
			S.y = current_y;
			subobjective_path.push_back(S);
		}
		counter1 ++;
	}
	//~ ROS_INFO_STREAM("reConstr " << current_x << " " << current_y << " " << end_x << " " << end_y);

	if (!(end_x * robot_perception.getMapResolution() == _curr_cell_x) 
		|| !(end_y * robot_perception.getMapResolution() == _curr_cell_y))
	{
		S.x = _curr_cell_x / robot_perception.getMapResolution();
		S.y = _curr_cell_y / robot_perception.getMapResolution();
		subobjective_path.push_back(S);
		best_path.push_back(S);
	}
	_counter = subobjective_path.size();
	
	_target_x = new float [_counter];
	_target_y = new float [_counter];
	for (unsigned int ii = 0; ii < _counter; ii ++) 
	{
		_target_x[ii] = subobjective_path[_counter - 1 - ii].x;
		_target_y[ii] = subobjective_path[_counter - 1 - ii].y;
		
		int i = _target_x[ii];
		int j = _target_y[ii];
		
		ROS_INFO_STREAM("targets " << i << " " << j << " F " << f_score[i][j] << " Brush " << robot_perception.getBrushfireCell(i, j));
	}
	
	_prev_sub_target_x = _target_x[0] * robot_perception.getMapResolution();
	_prev_sub_target_y = _target_y[0] * robot_perception.getMapResolution();
	

	_vel_timer = _node.createTimer(ros::Duration(_time), &Planner::velocity, this);		
	
	visual(subobjective_path);
	
	return best_path;
}

void Planner::visual(const std::vector <cell>& subobjective_path)
{
	 // Visualize the nodes
    visualization_msgs::Marker marker;
	geometry_msgs::Point p;
	
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.id = 0;
	marker.ns = "path_planning";
	
	for (unsigned int ii = 0; ii < subobjective_path.size(); ii ++)
	{
		p.x = subobjective_path[ii].x * robot_perception.getMapResolution();
		p.y = subobjective_path[ii].y * robot_perception.getMapResolution();
		p.z = 0;
		ROS_INFO_STREAM("PATH " << subobjective_path[ii].x * robot_perception.getMapResolution() 
						 << " " << subobjective_path[ii].y * robot_perception.getMapResolution());
		marker.points.push_back(p);	
	}
	
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
  
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.00;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	
	marker.lifetime = ros::Duration();
	
	_marker_pub.publish(marker);
    
    
}

void Planner::velocity(const ros::TimerEvent& event)
{
	geometry_msgs::Twist twist;
	std_msgs::Bool flag;
	std::vector <cell> best_path;
	best_path.clear();
	    
    //~ _current_time = ros::Time::now();
    
    if (subobjective_path.size() == 0)
	{
		_vel_pub.publish(twist);
		ROS_INFO_STREAM("No target");
		return;
	}

	_sub_target.x = _target_x[_dokimi];
	_sub_target.y = _target_y[_dokimi];
	
	
	//~ bool flag_test = false;
	//~ flag.data = flag_test;
	//~ _replanning_pub.publish(flag);
	//~ robot_perception.brushfire();

	if (robot_perception.getBrushfireCell(_sub_target.x, _sub_target.y) <= _brushfire_limit)
	{
		int curr_map_x = robot_perception.worldToMap(_prev_sub_target_x);
		int curr_map_y = robot_perception.worldToMap(_prev_sub_target_y);
		int goal_map_x = robot_perception.worldToMap(_goal_cell_x);
		int goal_map_y = robot_perception.worldToMap(_goal_cell_y);
		//~ _final_goal = false;
		best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
		//~ ROS_INFO_STREAM("COUNTER  " << _replanning_counter);
		if (_replanning_counter > 0)
		{
			publishNewPath(best_path);
		}
		
		_dynamic_flag = true;
		//~ subobjective_path.clear();
	}
	
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    	
	
	float sub_target_x = _sub_target.x * robot_perception.getMapResolution();
	float sub_target_y = _sub_target.y * robot_perception.getMapResolution();
	//~ robot_perception.brushfire();
	
	
	float dis = distance(_curr_cell_x, _curr_cell_y, sub_target_x, sub_target_y);
	float pi = 3.14159;
	
	_z = atan2(sub_target_y - _curr_cell_y, sub_target_x - _curr_cell_x);
	
	float yaw = _yaw;
	float a = yaw - _z;
	if (!_dynamic_flag)
	{
		if (dis > _dis_to_target)
		{
			//~ ROS_INFO_STREAM(" " << cos(yaw - _z) << " " << _yaw_limit);
			if (cos(yaw - _z) < _yaw_limit)
			{
				twist.angular.z = - sin(yaw - _z) * 0.2;
				
				//~ ROS_INFO_STREAM(" " << yaw - _z);
				if (yaw - _z < - 3)
				{
					twist.angular.z = (yaw - _z) / pi * 0.2;
					//~ ROS_INFO_STREAM(" " << twist.angular.z);
				}
				
				
				
				if (fabs(twist.angular.z) > 3.12 * 0.1)
				{
					twist.angular.z = twist.angular.z / 2;
				}
			}
			else
			{
				twist.linear.x = 0.05;
			}
			
			
			
		}
		else
		{
			_dokimi ++;
			_target_counter = 0;
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.linear.z = 0.0;
			twist.angular.x = 0.0;
			twist.angular.y = 0.0;
			twist.angular.z = 0.0;
			_vel_pub.publish(twist);
			
			bool flag_test = true;
			flag.data = flag_test;
			_replanning_pub.publish(flag);

			
			_prev_sub_target_x = sub_target_x;
			_prev_sub_target_y = sub_target_y;
			robot_perception.brushfire();
			
			if (_dokimi >= subobjective_path.size())
			{
				subobjective_path.clear();
			}
		}
	}
	//~ ROS_INFO_STREAM(
		//~ "subtarget: " << _dokimi << "( " << sub_target_x << "," << sub_target_y 
		//~ << " ) \n" <<
		//~ "robot: " << "( " << _curr_cell_x << "," << _curr_cell_y << "\n" <<
		//~ "distance to sub: " << dis << "\n" <<
		//~ "angle to sub: " << _z << "\n" <<
		//~ "trig: " << cos(yaw - _z) << " " << sin(yaw - _z) << "\n" <<
		//~ "theta: " << a << "\n" <<
		//~ "speeds: " << twist.linear.x << "\n" << 
		//~ "angular: " << twist.angular.z << "\n" <<
		//~ "\n"
	//~ );
	

	
	
	
	_vel_pub.publish(twist);
}

float Planner::distance(float x1, float y1, float x2, float y2)
{
	float dis = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	dis = sqrt(dis);
	return dis;
}

bool Planner::finalGoal(const std::vector <cell>& came_from_list, 
							int goal_map_x, int goal_map_y)
{
	_final_goal = false;
	ROS_INFO_STREAM("final_goal " << _final_goal << " " << came_from_list.size());
	for (unsigned int ii = 0; ii < came_from_list.size(); ii ++)
	{
		if (goal_map_x == came_from_list[ii].x && goal_map_y == came_from_list[ii].y)
		{
			ROS_INFO_STREAM("final_goal3333 " << _final_goal);
			_final_goal = true;
		}
	}
	
	ROS_INFO_STREAM("final_goal " << _final_goal);
	return _final_goal;
}

