#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <dynamic_localization/particle.h>
#include <dynamic_localization/robot_perception.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <dynamic_localization/particleInitSrv.h>


class ParticleFilter {
  private:
    ros::NodeHandle _n;
    ros::ServiceServer _particle_initialization_service;
    ros::Publisher _visualization_pub;
    ros::Subscriber _velocity_sub;
    ros::Subscriber _odometry_sub;
    ros::Time _current_time;
    ros::Time _previous_time;
    ros::Timer _timer;
    ros::Duration _dt;
    
    tf::TransformListener _listener;
    
    std::string _velocity_topic;
    std::string _odometry_topic;
    RobotPerception robot_percept;
    
    
    int _particles_number;
    float _duration;
    double _noise_param1;
    double _noise_param2;
    float _current_linear;
    float _current_angular;
    float _previous_linear;
    float _previous_angular;
    std::vector<Particle> _particles; 
    bool _visualization_enabled;
    bool _particles_initialized;
    bool _flag;
    bool _motion_flag;
    int _subs_step;
    double _strict;

    bool _start_from_point;
    int _start_x;
    int _start_y;
    float _start_theta;
    float _curr_x;
    float _curr_y;
    float _curr_theta;
    
    float prev_x;
    float prev_y;
    float prev_th;
    
    
    int _best_particle_id;
    float _robot_x;
    float _robot_y;
    float _robot_z;
    float _robot_yaw;
    
    
      
  public:
    ParticleFilter();
    bool particlesInit ( 
      dynamic_localization::particleInitSrv::Request& req,
      dynamic_localization::particleInitSrv::Response& res
      );
    void visualize(float resolution);
    void particlesCallback(const ros::TimerEvent& event);
    void velocityCallback(geometry_msgs::Twist twist);
    void odometryCallback(nav_msgs::Odometry odometry);
    void resample();
    
    void experiment();
    
};

#endif
