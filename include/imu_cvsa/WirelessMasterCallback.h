#ifndef IMU_CVSA_WIRELESSMASTER_H
#define IMU_CVSA_WIRELESSMASTER_H

#include "xsmutex.h"
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>    
#include <ros/ros.h>

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback{
public:
    typedef std::set<XsDevice*> XsDeviceSet;

    XsDeviceSet getWirelessMTWs() const{
        XsMutexLocker lock(m_mutex);
        return m_connectedMTWs;
    }

protected:
    virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState){
        XsMutexLocker lock(m_mutex);
        XsDeviceId deviceId = dev->deviceId();
        switch (newState){
            case XCS_Disconnected:        /*!< Device has disconnected, only limited informational functionality is available. */
                ROS_INFO("[imu_cvsa] MTW Disconnected -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.erase(dev);
                break;
            case XCS_Rejected:            /*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
                ROS_INFO("[imu_cvsa] MTW Rejected -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.erase(dev);
                break;
            case XCS_PluggedIn:            /*!< Device is connected through a cable. */
                ROS_INFO("[imu_cvsa] MTW PluggedIn -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.erase(dev);
                break;
            case XCS_Wireless:            /*!< Device is connected wirelessly. */
                ROS_INFO("[imu_cvsa] MTW Connected -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.insert(dev);
                break;
            case XCS_File:                /*!< Device is reading from a file. */
                ROS_INFO("[imu_cvsa] MTW File -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.erase(dev);
                break;
            case XCS_Unknown:            /*!< Device is in an unknown state. */
                ROS_INFO("[imu_cvsa] MTW Unknown -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.erase(dev);
                break;
            default:
                ROS_ERROR("[imu_cvsa] MTW Error -> ID: %s", deviceId.toString().toStdString().c_str());
                m_connectedMTWs.erase(dev);
                break;
        }
    }
private:
    mutable XsMutex m_mutex;
    XsDeviceSet m_connectedMTWs;
};

#endif