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

// FILL IN THE CODE
// hint: add any include that you need

#include <iDynTree/KinDynComputations.h>

class Module : public yarp::os::RFModule
{


    yarp::os::BufferedPort<yarp::sig::Vector> externalForcesPort;
    yarp::sig::Vector externalForces;
    yarp::os::BufferedPort<yarp::os::Bottle> outPutFlagPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inPutFlagPort;
    bool go;


public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
