#ifndef IMU_CVSA_MTW_H
#define IMU_CVSA_MTW_H

#include "xsmutex.h"
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>    

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback{
public:
    MtwCallback(int mtwIndex, XsDevice* device)
        :m_mtwIndex(mtwIndex)
        ,m_device(device)
    {}

    bool dataAvailable() const{
        XsMutexLocker lock(m_mutex);
        return !m_packetBuffer.empty();
    }

    XsDataPacket const * getOldestPacket() const{
        XsMutexLocker lock(m_mutex);
        XsDataPacket const * packet = &m_packetBuffer.front();
        return packet;
    }

    void deleteOldestPacket(){
        XsMutexLocker lock(m_mutex);
        m_packetBuffer.pop_front();
    }

    int getMtwIndex() const{
        return m_mtwIndex;
    }

    XsDevice const & device() const{
        assert(m_device != 0);
        return *m_device;
    }

protected:
    virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet){
        XsMutexLocker lock(m_mutex);
        // NOTE: Processing of packets should not be done in this thread.

        m_packetBuffer.push_back(*packet);
        if (m_packetBuffer.size() > 300){
            deleteOldestPacket();
        }
    }

private:
    mutable XsMutex m_mutex;
    std::list<XsDataPacket> m_packetBuffer;
    int m_mtwIndex;
    XsDevice* m_device;
};

#endif