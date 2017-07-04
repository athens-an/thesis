#ifndef _ROBOT_PERCEPTION_H
#define _ROBOT_PERCEPTION_H


#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <sstream>
#include <iterator>


#include <art_localization_particle_filters_ogm/DynamicObstacles.h> 
#include <art_localization_particle_filters_ogm/DynamicCoordinates.h> 

struct dynamic
{
	int x;
	int y;
};

class RobotPerception {
	
	private:
		ros::NodeHandle _node;
		ros::Subscriber _map_sub;
		ros::Publisher _br_pub;
		
		int _width;
		int _height;
		int _map_size;
		float _resolution;
		
		int * _index;
		int * data;
		float ** _brushfire;
		
		float _goal_cell_x;
		float _goal_cell_y;
		
		int _a;
		int _m;
		std::string _dynamic_topic;
		art_localization_particle_filters_ogm::DynamicObstacles _dynamics;
		ros::Subscriber _dynamic_obst_sub;
		std::vector <dynamic> _dynamic_map;
		
		ros::Subscriber _replanning_sub;
		std::string _replanning_counter;
		bool _replanning_flag;
		
	
	public:
	
		RobotPerception();
		
		void readMap(const nav_msgs::OccupancyGridConstPtr& msg);
		int getMapWidth();
		int getMapHeight();
		int getMapSize();
		int getMapCell(int ii, int jj); 
		int* getMapIndex();
		float getMapResolution();
		float getGoalXPosition();
		float getGoalYPosition();
		
		//~ void currentPosition(const ros::TimerEvent& e);
		//~ float getCurrentXPosition();
		//~ float getCurrentYPosition();
		//~ float getCurrentYaw();
		
		int worldToMap(float w_coor);
		void mapToWorld(int m_x, int m_y);
		bool rightCell(int x, int y);
		void brushfire();
		int getBrushfireCell(int x, int y);
		
		
		void dynamicC(const art_localization_particle_filters_ogm::DynamicObstacles& msg);
		void replanningCounter(const std_msgs::Bool& flag);
		
		
		
};

#endif
