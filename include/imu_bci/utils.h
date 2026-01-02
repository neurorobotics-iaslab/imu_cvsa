#ifndef IMU_CVSA_UTILS_H
#define IMU_CVSA_UTILS_H

#include <xstypes/xstypesconfig.h>

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const & p){
    out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
        << std::setw(7) << p.baudrate() << " Bd"
        << ", " << "ID: " << p.deviceId().toString().toStdString();
    return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const & d){
    out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
    return out;
}

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate){
    if (supportedUpdateRates.empty()){
        return 0;
    }

    if (supportedUpdateRates.size() == 1){
        return supportedUpdateRates[0];
    }

    int uRateDist = -1;
    int closestUpdateRate = -1;
    for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate){
        const int currDist = std::abs(*itUpRate - desiredUpdateRate);

        if ((uRateDist == -1) || (currDist < uRateDist)){
            uRateDist = currDist;
            closestUpdateRate = *itUpRate;
        }
    }
    return closestUpdateRate;
}

#endif