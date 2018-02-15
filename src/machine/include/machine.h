#ifndef MACHINE_
#define MACHINE_

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

enum STATES{IDLE, CMD, POINT_ACTION, REWARD, HOME, QUIT};

class MachineModule : public yarp::os::RFModule
{

private:

    STATES state;
    string cmdStr;
    int feedback;

    RpcServer commandPort;
    RpcClient speechRecogn;
    RpcClient positionControl;
    BufferedPort<Bottle> noObjectIn;
    BufferedPort<Bottle> objectBottleIn;
    BufferedPort<Bottle> objectBottleOut;
    BufferedPort<Bottle> reachedPosIn;
    BufferedPort<Bottle> rewardDetectorOut;
    BufferedPort<Bottle> feedbackIn;
    //BufferedPort<Bottle> homeOut;
    //BufferedPort<Bottle> homeIn;

    bool openPorts(const string &moduleName);

public:

    /**
     * Configure function. Receive a previously initialized
     * resource finder object. Use it to configure your module.
     * Open port and attach it to message handler and etc.
     */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /**
     * Get the period with which updateModule(void) should be called
     */
    virtual double getPeriod(void);

    /**
     * This is our main function. Will be called periodically every getPeriod(void) seconds.
     */
    virtual bool updateModule(void);

    /**
     * Message handler. Just echo all received messages.
     */
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

     /**
      * Interrupt function.
      */
    virtual bool interruptModule(void);

     /**
      * Close function, to perform cleanup.
      */
     virtual bool close(void);

};

#endif //MACHINE_
