#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/all.h>
#include <yarp/os/RpcClient.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

//#include "closestBlob_IDL.h"

class CustomProcessor : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >
{
    std::string moduleName;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    inRGBPort;                     // if we ever need rgb image in
    yarp::os::BufferedPort<yarp::os::Bottle>                            outStuffPort;                  // to stream object positions out
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    outDebugPortRGB;               // output port from debugging
    yarp::os::BufferedPort<yarp::os::Bottle>                            detection_timeout;             // signal when timeout

    double valid_detection_timer, valid_detection_timer_threshold;

public:
    int                             threshold_value;
    yarp::os::RpcClient             queryClient;

    CustomProcessor(const std::string &moduleName);
    bool open();
    void close();
    void interrupt();
    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage );
};

class Module : public yarp::os::RFModule //, public closestBlob_IDL
{

    std::string moduleName;

    yarp::os::RpcServer             rpcPort;
    yarp::os::ResourceFinder        *rf;
    CustomProcessor                 *inDispPort;
    friend class                    inDispPort;
    bool                            closing;

    
public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
    bool close();
    bool quit();
    double getPeriod();
    bool updateModule();


};


