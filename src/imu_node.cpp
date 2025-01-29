#include <ros/ros.h>
#include "imu_cvsa/Imu.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "imu_cvsa_node");

	Imu imu;
	
	if(imu.configure() == false) {
		ROS_ERROR("[imu_cvsa] Error configuring the system");
		imu.~Imu();
		ros::shutdown();
		return -1;
	}

    if(imu.setUp() == false){
		ROS_ERROR("[imu_cvsa] Error setting up the system");
		imu.~Imu();
		ros::shutdown();
        return -1;
    }
	
	imu.run();
    
	imu.~Imu();
	ros::shutdown();
	return 0;
}
