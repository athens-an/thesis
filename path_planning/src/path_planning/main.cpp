#include "path_planning/path_planning.h"


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "path_planning");
	
	Planner p;
	
	
	ros::spin();
	return 0;
		
}
