#include "dynamic_localization/particle_filter.h"

ParticleFilter::ParticleFilter() 
{
  sleep(5);
  ROS_ERROR("Initializing");
  _previous_linear = 0;
  _previous_angular = 0;
  _current_angular = 0;
  _current_linear = 0;
  _previous_time = ros::Time::now();
  _flag = false;
  _motion_flag = false;
  _particles_initialized = false;
  _start_from_point = false;

  _start_x = _start_y = -1;
  
  if(!_n.getParam("/known_initial_pose", _start_from_point))
  {
    ROS_ERROR("Enable_visualization param does not exist");
  }
  else
  {
    if(_start_from_point == true)
    {
      _n.getParam("/initial_pose_x", _start_x);
      _n.getParam("/initial_pose_y", _start_y);
      _n.getParam("/initial_pose_theta", _start_theta);
      std::cout << "Particle filter initializes from point " << _start_x << 
        " " << _start_y << " " << _start_theta << "\n";
    }
  }

  if(!_n.getParam("/enable_visualization", _visualization_enabled))
  {
    ROS_ERROR("Enable_visualization param does not exist");
  }
  if(!_n.getParam("duration", _duration))
  {
    ROS_ERROR("Duration param does not exist");
  }
  if(!_n.getParam("a1", _noise_param1))
  {
    ROS_ERROR("Noise param a1 does not exist");
  }
  if(!_n.getParam("a2", _noise_param2))
  {
    ROS_ERROR("Noise param a2 does not exist");
  }
  if(!_n.getParam("/velocity_topic", _velocity_topic))
  {
    ROS_ERROR("Velocity_topic param does not exist");
  }
  if(!_n.getParam("/odometry_topic", _odometry_topic))
  {
    ROS_ERROR("odometry_topic param does not exist");
  }

  if(!_n.getParam("/particles_number", _particles_number))
  {
    ROS_ERROR("Particles number param does not exist");
  }
  if(!_n.getParam("/subsampling_step", _subs_step))
  {
    ROS_ERROR("Particles number param does not exist");
  }
  if(!_n.getParam("/resampling_strictness", _strict))
  {
    ROS_ERROR("Particles number param does not exist");
  }
  ROS_ERROR_STREAM(_strict);
  _visualization_pub = _n.advertise<visualization_msgs::Marker>(
    "visualization_marker", 0);

  while(robot_percept.map_initialized == false){
    usleep(100000);
    ros::spinOnce();
  }
  ROS_ERROR("Initializing particles");
  
  for (unsigned int i = 0 ; i < _particles_number ; i++ )
  {
    Particle particle(robot_percept.getMapWidth(),
      robot_percept.getMapHeight(), robot_percept.getMapData(),
      robot_percept.getLaserRanges(), robot_percept.getMapResolution());
    if(_start_from_point == true)
    {
      particle.setXYTheta(_start_x, _start_y, _start_theta);
    }
    _particles.push_back(particle);
  }
  _particles_initialized = true;

  ROS_INFO_STREAM(_particles_number << " " << "particles initialized");
  if(_visualization_enabled)
    visualize(robot_percept.getMapResolution());

  //~ if(_odometry_topic == "")
  //~ {
    _velocity_sub = _n.subscribe(_velocity_topic, 10,
      &ParticleFilter::velocityCallback, this);
  //~ } 
  //~ else
  //~ {
    //~ _odometry_sub = _n.subscribe(_odometry_topic, 10,
      //~ &ParticleFilter::odometryCallback, this);
  //~ }
  
  _curr_x = _start_x;
  _curr_y = _start_y;
  _curr_theta = _start_theta;

  _timer = _n.createTimer(ros::Duration(_duration),
    &ParticleFilter::particlesCallback, this);
}

void ParticleFilter::particlesCallback(const ros::TimerEvent& event)
{
	//~ robot_percept.getCoordinates(prev_x, prev_y, prev_th);
	
  if (_particles_initialized)
  {
	
	if (_previous_angular || _previous_linear)
	  _motion_flag = true;
	_current_time = ros::Time::now();
	_dt = _current_time - _previous_time;
	for (unsigned int i = 0 ; i < _particles_number ; i++)
	{
	  _particles[i].calculateMotion(_previous_linear, _previous_angular, _dt, _noise_param1, _noise_param2);
	  _particles[i].move();
	  //~ ROS_INFO_STREAM("New x = " << _particles[i].getX() << " new y = " << _particles[i].getY());
	  _previous_angular = _current_angular;
	  _previous_linear = _current_linear;
	  _previous_time = _current_time;
	}    
		
	visualize(robot_percept.getMapResolution());
	for (unsigned int i = 0 ; i < _particles_number ; i++)
	{
	  _particles[i].setParticleWeight(robot_percept.getMapWidth(),
		robot_percept.getMapHeight(), robot_percept.getMapData(),
		robot_percept.getMapResolution(), robot_percept.getStaticRanges(), robot_percept.getStaticIdRanges(), 
		robot_percept.getRangeMax(), robot_percept.getAngleIncrement(), 
		robot_percept.getAngleMin(), robot_percept.getRfidPose(), 
		_subs_step, _strict);
	}
	if (_motion_flag)
	  resample();
	_motion_flag = false;
	visualize(robot_percept.getMapResolution());
  }
  
	
}

void ParticleFilter::resample()
{
  float average = 0;
  float sum = 0;
  bool flag = false;
  for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
  {
    if (_particles[i].getWeight() > 0.00001)
    {
      flag = true;
      break;
    }
  }
  for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
  {
    sum += _particles[i].getWeight();
  }
  average = sum/ _particles_number;
  //~ ROS_INFO_STREAM("average1 = " << average);
  if (flag == true)
  {
    sum = 0;
    std::vector<Particle> new_particles;
    int index = std::rand() % ( _particles_number );
    float beta = 0.0;
    float max_weight = _particles[0].getWeight();
    for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
    {
      if (_particles[i].getWeight() > max_weight)
        max_weight = _particles[i].getWeight();
    }
    for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
    {
      beta += static_cast <float> (rand()) / static_cast <float> 
        (RAND_MAX/ (2*max_weight));
      while (beta > _particles[index].getWeight())
      {
        beta -= _particles[index].getWeight();
        index = (index + 1) % _particles_number;
      }
      new_particles.push_back(_particles[index]);
    }

    _particles = new_particles;
    for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
    {
      sum += _particles[i].getWeight();
    }
    average = sum/ _particles_number;
  }
  
  
  experiment();
  
}

void ParticleFilter::velocityCallback(geometry_msgs::Twist twist)
{
  //~ ROS_INFO_STREAM("VelocityCallback");
  _current_linear = twist.linear.x;
  _current_angular = twist.angular.z;
	
	//~ ROS_INFO_STREAM("TAXUTHTES  " << _current_linear << "   " << _current_angular);
  if (_particles_initialized)
  {
    if ((_current_angular != _previous_angular) || (_current_linear != _previous_linear))
    {
		//~ if (_current_angular !=0)
		//~ {
			//~ robot_percept.giaNaKaleswStoParticleFilter(_previous_linear, _previous_angular);
		//~ }
		//~ robot_percept.visual();
      
      if (_flag == false)
      {
        _previous_time = ros::Time::now();
        _previous_angular = _current_angular;
        _previous_linear = _current_linear;
        _flag = true;
      }
      else
      {
        _motion_flag = true;
        _current_time = ros::Time::now();
        _dt = _current_time - _previous_time;
        for (int i = 0 ; i < _particles_number; i ++)
        {
          _particles[i].calculateMotion(_previous_linear, _previous_angular, _dt, _noise_param1, _noise_param2);
        }
        _previous_angular = _current_angular;
        _previous_linear = _current_linear;
        _previous_time = _current_time;
      }
    }
  }
  //~ robot_percept.getCoordinates(_current_linear, _current_angular, _dt);
	
  //~ robot_percept.findLaserXY();
  //~ ROS_INFO_STREAM("XRONOS " << _dt);
}

void ParticleFilter::odometryCallback(nav_msgs::Odometry odometry)
{
  //~ ROS_INFO_STREAM("VelocityCallback");
  _current_linear = odometry.twist.twist.linear.x;
  _current_angular = odometry.twist.twist.angular.z;

  if (_particles_initialized)
  {
    if ((_current_angular != _previous_angular) || (_current_linear != _previous_linear))
    {
      if (_flag == false)
      {
        _previous_time = ros::Time::now();
        _previous_angular = _current_angular;
        _previous_linear = _current_linear;
        _flag = true;
      }
      else
      {
        _motion_flag = true;
        _current_time = ros::Time::now();
        _dt = _current_time - _previous_time;
        for (int i = 0 ; i < _particles_number; i ++)
        {
          _particles[i].calculateMotion(_previous_linear, _previous_angular, _dt, _noise_param1, _noise_param2);
        }
        _previous_angular = _current_angular;
        _previous_linear = _current_linear;
        _previous_time = _current_time;
      }
    }
  }
}

void ParticleFilter::visualize(float resolution)
{
  visualization_msgs::Marker m, m1;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time();
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.id = 0;
  m.ns = "Particles";
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;

  for(unsigned int i = 0 ; i < _particles_number ; i++)
  {
    geometry_msgs::Point p;
    p.x = _particles[i].getX();
    p.y = _particles[i].getY();
    m.points.push_back(p);
  }

  _visualization_pub.publish(m);

  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time();
  m1.type = visualization_msgs::Marker::SPHERE_LIST;
  m1.action = visualization_msgs::Marker::ADD;
  m1.id = 0;
  m1.ns = "Best Particle";
  m1.scale.x = 0.35;
  m1.scale.y = 0.35;
  m1.scale.z = 0.35;
  m1.color.a = 1.0;
  m1.color.r = 0.0;
  m1.color.g = 0.0;
  m1.color.b = 1.0;

  static bool pose_initialized = false;
  //~ prev_x = 0;
  //~ prev_y = 0;
  //~ prev_th = 0;

  float max_weight = _particles[0].getWeight();
  int id = 0;
  for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
  {
    if (_particles[i].getWeight() > max_weight)
    {
      max_weight = _particles[i].getWeight();
      id = i;
    }
  }
  
  _best_particle_id = id;

  geometry_msgs::Point p1;
  p1.x = _particles[id].getX();
  p1.y = _particles[id].getY();
  m1.points.push_back(p1);

  _visualization_pub.publish(m1);

  if (pose_initialized == false)
  {
    prev_x = _particles[id].getX();
    prev_y = _particles[id].getY();
    prev_th = _particles[id].getTheta();
    pose_initialized = true;
  }

  prev_x = _particles[id].getX() * 0.3 + prev_x * 0.7;
  prev_y = _particles[id].getY() * 0.3 + prev_y * 0.7;
  prev_th = _particles[id].getTheta() * 0.3 + prev_th * 0.7;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin( tf::Vector3(_particles[id].getX(), _particles[id].getY(), 0.0) );
  q.setRPY(0, 0, _particles[id].getTheta());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "art_robot"));

  transform.setOrigin( tf::Vector3(prev_x, prev_y, 0.0) );
  q.setRPY(0, 0, prev_th);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "art_robot_tensor"));
  
   _curr_x = _particles[id].getX();
  _curr_y = _particles[id].getY();
  _curr_theta = _particles[id].getTheta();


  
}


void ParticleFilter::experiment()
{
	//~ ROS_INFO_STREAM("Sygkrish ths theshs");
	
	try
	{
		tf::StampedTransform transform;
		
		_listener.lookupTransform("map", "robot0", ros::Time(0), transform);
		
		_robot_x = transform.getOrigin()[0]; //transform.getOrigin().x()
		_robot_y = transform.getOrigin()[1];
		_robot_z = transform.getOrigin()[2];
		_robot_yaw = tf::getYaw(transform.getRotation());
		
		//~ ROS_INFO_STREAM("KANONIKH THESH: " << _robot_x << " " 
							//~ << _robot_y << " " << _robot_yaw);					
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(2.0).sleep();
	}
	
	
	float best_particle_pose_x = _particles[_best_particle_id].getX();
	float best_particle_pose_y = _particles[_best_particle_id].getY();
	float robot_pose_x = _robot_x / robot_percept.getMapResolution();
	float robot_pose_y = _robot_y / robot_percept.getMapResolution();
	
	ROS_INFO_STREAM("KANONIKH THESH: " << robot_pose_x << " " 
							<< robot_pose_y << " " << best_particle_pose_x << " " << best_particle_pose_y);
	
	
	float dis_x = best_particle_pose_x - _robot_x;
	float dis_y = best_particle_pose_y - _robot_y;
	float dis = sqrt(dis_x * dis_x + dis_y * dis_y);
	//~ ROS_INFO_STREAM("DIAFORA APOSTASHS X: " << dis_x << " Y: " << dis_y << " genika: " << dis);
	ROS_INFO_STREAM("" << dis);
	
	float average = 0;
	float sum = 0;
	
	for (unsigned int ii = 0; ii < _particles_number; ii ++)
	{
		average += sqrt(_particles[ii].getX() * _particles[ii].getX() + 
						_particles[ii].getY() * _particles[ii].getY());
	}
	
	average = average / _particles_number;
	
	float k;
	float var;
	for (unsigned int ii = 0; ii < _particles_number; ii ++)
	{
		k = sqrt(_particles[ii].getX() * _particles[ii].getX() + 
					_particles[ii].getY() * _particles[ii].getY());
		sum += (k - average) * (k - average);
	}
	
	var = sum / _particles_number;
	
	//~ ROS_INFO_STREAM("DIASPORA: " << var);
	//~ ROS_INFO_STREAM("" << var);
	ROS_INFO_STREAM("" << dis << " " << var);
	
}



