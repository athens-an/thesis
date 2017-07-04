#ifndef _PATH_PLANNING_H
#define _PATH_PLANNING_H

#include "ros/package.h"
#include "path_planning/start.h"
#include "path_planning/goal.h"


#include "path_planning/node.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>



struct cell {
	int x;
	int y;
	float f_score;
	int cf_x;
	int cf_y;
	int counter;
};


class Planner {
	private:
		ros::NodeHandle _node;
		ros::ServiceServer _service1;
		ros::ServiceServer _service2;
		ros::ServiceServer _service3;
		ros::Publisher _path_pub;
		ros::Publisher _marker_pub;
		ros::Publisher _vel_pub;
		ros::Publisher _replanning_pub;
		ros::Subscriber _dynamic_sub;
		
		ros::Timer _position_timer;
		ros::Timer _vel_timer;
		
		tf::TransformListener _listener;	
		geometry_msgs::Twist _twist;	
		geometry_msgs::Pose2D _sub_target;
		geometry_msgs::Pose2D _current_pose;
		
		std::string _map_topic;
		std::string _velocity_topic;
		std::string _position_topic;

		std::string _dynamic_counter;
		
		RobotPerception robot_perception;
		Node node_obj;

		
		float _duration;
		float _time;
		float _distance_limit;
		float _yaw_limit;
		float _dis_to_target;
		float _brushfire_const;
		int _brushfire_limit;
		
		int _goal_counter;
		int _goal_cell_x;
		int _goal_cell_y;
		
		std::vector <cell> subobjective_path;	
		float ** g_score;
		float ** f_score;
		
		float * _target_x;
		float * _target_y;

		float _z; //krataei thn prohgoumenh twist.angular.z

		int _dokimi; //counter gia ton pinaka twn upostoxwn
		int _counter;
		
		int curr_node;
		int last_curr_node;
		int prev_goal_map_x;
		int prev_goal_map_y;
		
		float _curr_cell_x;
		float _curr_cell_y;
		float _curr_cell_z;
		float _yaw;
		float _prev_linear;
		float _prev_angular;
		
		
		bool _final_goal;
		bool _dynamic_flag;
		bool _flag_time;
		ros::Time _current_time;
		ros::Time _previous_time;
		ros::Duration _dt;
		
		
		int _target_counter;
		float _prev_sub_target_x;
		float _prev_sub_target_y;
		int _replanning_counter;
		
		
		
		
		
		
		
	public:
	
		Planner();
		bool random();
		bool goal(path_planning::goalRequest &req, path_planning::goalResponse &res);
		bool start(path_planning::startRequest &req, path_planning::startResponse &res);
		void currentPosition(const ros::TimerEvent& e);
		
		void dynamicCounter(const std_msgs::Bool& flag);
		void publishNewPath(const std::vector <cell>& best_path);
		
		float calculateHScore(int curr_map_x, int curr_map_y, int goal_map_x, int goal_map_y);
		
		std::vector <cell> path (int _curr_cell_x, int _curr_cell_y, int goal_map_x, int goal_map_y);
		std::vector <cell> reconstructPath (const std::vector <cell>& _came_from, int goal_map_x, int goal_map_y);
		
		bool finalGoal(const std::vector <cell>& _came_from, int goal_map_x, int goal_map_y);
		
		void visual(const std::vector <cell>& subobjective_path);
		
		void velocity(const ros::TimerEvent& event);
		void test();
		float distance(float current_x, float current_y, float sub_target_x, float sub_target_y);		
		
		

};

#endif

