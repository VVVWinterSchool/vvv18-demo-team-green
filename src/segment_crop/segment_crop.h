#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

//#include "closestBlob_IDL.h"

class Module : public yarp::os::RFModule //, public closestBlob_IDL
{
    
public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool quit();
    double getPeriod();
    bool updateModule();

};
