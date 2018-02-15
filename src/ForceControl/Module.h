#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>
//#include <yarp/eigen/Eigen.h>

//#include <eigen3/Eigen/Eigen>

// FILL IN THE CODE
// hint: add any include that you need
#include <cstdlib>
#include <string>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/iKinConversions.h>
class Module : public yarp::os::RFModule
{
    // Class to compute model quantities
    iDynTree::KinDynComputations kinDynModel;

    // Attributes to communicate with the robot YARP-based interface
    yarp::dev::PolyDriver robotDevice,drvArm;
    yarp::dev::ICartesianControl *iarm;
    bool simulation;

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IControlLimits2   *ilim{nullptr};
    yarp::dev::IEncoders         *ienc{nullptr};
    yarp::dev::IControlMode2     *imod{nullptr};
    yarp::dev::ITorqueControl    *itrq{nullptr};
    bool triggerOnce;
    // FILL IN THE CODE
    // hint: what attributes you need?
    // Quantities used by the control
    yarp::sig::Vector positionsInRad;
    yarp::sig::Vector positionsInitInRad;
    yarp::sig::Vector velocitiesInRadS;
    yarp::sig::Vector positionsInDeg;
    yarp::sig::Vector velocitiesInDegS;
    yarp::sig::Vector gravityCompensation;
    yarp::sig::Vector *referencePositionsInRad;
    yarp::sig::Vector xCur,xIni,oIni;
    yarp::sig::Vector oCur;
    unsigned actuatedDOFs;

    //write
    yarp::sig::Vector errorInRad;
    yarp::sig::Vector kp; // Nm/rad
    yarp::sig::Vector kd; // Nm/rad
    yarp::sig::Vector torquesInNm;
    yarp::sig::Vector zeroDofs;
    yarp::sig::Vector baseZeroDofs;
    yarp::sig::Vector grav;

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
