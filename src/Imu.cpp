#include "Imu.hpp"

Imu::Imu(void) : nh_("~") {
    this->pub_ = this->nh_.advertise<imu_cvsa::imu_data>("/imu_cvsa", 1);
    this->srv_ = this->nh_.advertiseService("/imu_cvsa/receiving_singals", &Imu::receiving_singals, this);
}

bool Imu::receiving_singals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if(this->counter_ > 0){
        res.success = true;
        res.message = "Receiving signals";
    }else{
        res.success = false;
        res.message = "Not receiving signals";
    }
    return true;
}

bool Imu::configure(){
    this->control_ = XsControl::construct();
    if(this->control_ == 0){
        ROS_ERROR("[imu_cvsa] Error XsControl instance construct");
        return false;
    }

    ROS_INFO("[imu_cvsa] Scanning ports and finding wireless master...");
    XsPortInfoArray detectedDevices = XsScanner::scanPorts();
    XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();

    // open the port to search the master
    while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster()){
        ++wirelessMasterPort;
    }
    if (wirelessMasterPort == detectedDevices.end()){
        ROS_ERROR("[imu_cvsa] No wireless masters found");
        return false;
    }
    if (!this->control_->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate())){
        ROS_ERROR("[imu_cvsa] Failed to open master port");
        return false;
    }

    this->wirelessMasterDevice_ = this->control_->device(wirelessMasterPort->deviceId());
    if (this->wirelessMasterDevice_ == 0){
        ROS_ERROR("[imu_cvsa] Failed to construct XsDevice instance");
        return false;
    }
    if (!this->wirelessMasterDevice_->gotoConfig()){
        ROS_ERROR("[imu_cvsa] Failed to goto config mode for the master");
        return false;
    }

    // add callback for the master and update the devices
    this->wirelessMasterDevice_->addCallbackHandler(&this->wirelessMasterCallback_);

    // update the supported update rates -> set the new rate
    const XsIntArray supportedUpdateRates = this->wirelessMasterDevice_->supportedUpdateRates();
    const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, this->desiredUpdateRate_);
    if (!this->wirelessMasterDevice_->setUpdateRate(newUpdateRate)){
        ROS_ERROR("[imu_cvsa] Failed to set update rate");
        return false;
    }

    // disabiling radio channel if previously enabled -> enable the desired radio channels
    if (this->wirelessMasterDevice_->isRadioEnabled()){
        if (!this->wirelessMasterDevice_->disableRadio()){
            ROS_ERROR("[imu_cvsa] Failed to disable radio channel");
            return false;
        }
    }
    if (!this->wirelessMasterDevice_->enableRadio(this->desiredRadioChannel_)){
        ROS_ERROR("[imu_cvsa] Failed to set radio channels");
        return false;
    }

    // counter for the received data
    this->counter_ = 0;

    ROS_INFO("[imu_cvsa] Configuration done");
    return true;
}

bool Imu::setUp(){
    ROS_INFO("[imu_cvsa] Waiting for MTW to wirelessly connection");
    bool waitForConnections = true;
    try{
    do{
        XsTime::msleep(100);
        size_t nextCount = this->wirelessMasterCallback_.getWirelessMTWs().size();
        if(nextCount > 0){
            ROS_INFO("[imu_cvsa] Number of connected MTWs: %ld", nextCount);
            waitForConnections = false;
        }
    }
    while (waitForConnections);
    }catch(...){
        ROS_ERROR("[imu_cvsa] Failed setup configuration: unkown error");
        return false;
    }

    ROS_INFO("[imu_cvsa] Set up done");
    return true;
}

bool Imu::run(){
    // go to measurement mode
    if (!this->wirelessMasterDevice_->gotoMeasurement()){
        ROS_ERROR("[imu_cvsa] Failed to goto measurement mode");
        return false;
    }

    // Getting XsDevice instances for all MTWs
    XsDeviceIdArray allDeviceIds = this->control_->deviceIds();
    XsDeviceIdArray mtwDeviceIds;
    for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i){
        if (i->isMtw()){
            mtwDeviceIds.push_back(*i);
        }
    }
    XsDevicePtrArray mtwDevices;
    for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i){
        XsDevicePtr mtwDevice = this->control_->device(*i);
        if (mtwDevice != 0){
            mtwDevices.push_back(mtwDevice);
        }else{
            ROS_ERROR("[imu_cvsa] Failed to create an MTW XsDevice instance");
            return false;
        }
    }

    // Attaching callback handlers to MTWs
    this->mtwCallbacks_.resize(mtwDevices.size());
    for (int i = 0; i < (int)mtwDevices.size(); ++i){
        this->mtwCallbacks_[i] = new MtwCallback(i, mtwDevices[i]);
        mtwDevices[i]->addCallbackHandler(this->mtwCallbacks_[i]);
    }

    // start the measure
    ROS_INFO("[imu_cvsa] Starting the measure");
    ros::Rate r(512);
    while (ros::ok()) {
        for (size_t i = 0; i < this->mtwCallbacks_.size(); ++i){
            if (this->mtwCallbacks_[i]->dataAvailable()){
                imu_cvsa::imu_data imu_msg;

                XsDataPacket const * packet = this->mtwCallbacks_[i]->getOldestPacket(); 

                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.seq = this->counter_;

                imu_msg.orientation.x = packet->orientationQuaternion().x();
                imu_msg.orientation.y = packet->orientationQuaternion().y();
                imu_msg.orientation.z = packet->orientationQuaternion().z();
                imu_msg.orientation.w = packet->orientationQuaternion().w();
                imu_msg.orientation_covariance.front() = 0.0;

                imu_msg.angular_velocity.x = packet->calibratedGyroscopeData().at(0);
                imu_msg.angular_velocity.y = packet->calibratedGyroscopeData().at(1);
                imu_msg.angular_velocity.z = packet->calibratedGyroscopeData().at(2);
                imu_msg.angular_velocity_covariance.front() = 0.0;

                imu_msg.linear_acceleration.x = packet->calibratedAcceleration().at(0);
                imu_msg.linear_acceleration.y = packet->calibratedAcceleration().at(1);
                imu_msg.linear_acceleration.z = packet->calibratedAcceleration().at(2);
                imu_msg.linear_acceleration_covariance.front() = 0.0;

                imu_msg.euler_roll = packet->orientationEuler().roll();
                imu_msg.euler_pitch = packet->orientationEuler().pitch();
                imu_msg.euler_yaw = packet->orientationEuler().yaw();
                    
                this->mtwCallbacks_[i]->deleteOldestPacket();

                this->pub_.publish(imu_msg);
                ++this->counter_;
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return true;
}

Imu::~Imu(){
    ROS_INFO("[imu_cvsa] Closing XsControl and deleting MTW callbacks");
    this->control_->close();

    for (std::vector<MtwCallback*>::iterator i = this->mtwCallbacks_.begin(); i != this->mtwCallbacks_.end(); ++i){
        delete (*i);
    }
}
