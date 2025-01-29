#ifndef IMU_HPP
#define IMU_HPP

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>	

#include <ros/ros.h>
#include <imu_cvsa/imu_data.h>
#include "std_srvs/Trigger.h"

#include "xsmutex.h"
#include "utils.h"
#include "MtwCallback.h"
#include "WirelessMasterCallback.h"

class Imu{
    public:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        ros::Publisher pub_;
        ros::ServiceServer srv_;

        const int desiredUpdateRate_ = 75;	// Use 75 Hz update rate for MTWs
	    const int desiredRadioChannel_ = 19;	// Use radio channel 19 for wireless master.

        WirelessMasterCallback wirelessMasterCallback_; // Callback for wireless master
	    std::vector<MtwCallback*> mtwCallbacks_;
        XsControl* control_;
        XsDevicePtr wirelessMasterDevice_;

        int counter_;

    public:
        Imu();
        bool configure(void);
        bool setUp(void);
        bool run(void);
        bool receiving_singals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        ~Imu();
};

#endif