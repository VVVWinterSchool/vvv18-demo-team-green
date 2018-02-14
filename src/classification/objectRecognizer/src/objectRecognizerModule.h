#ifndef OBJECT_RECOGNIZER_MODULE_H
#define OBJECT_RECOGNIZER_MODULE_H

// Inspired from the tutorial on how to wrap Caffe in YARP and recognize objects in images - Giulia Pasquale - <giulia.pasquale@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/Stamp.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <highgui.h>
#include <cv.h>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <cstdio>
#include <cstdlib> // getenv
#include <string>
#include <deque>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>

#include "CaffeWrapper.hpp"
#include "definitions.h"
#include "objectRecognizerPort.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class ObjectRecognizerModule: public RFModule
{
    protected:

        Semaphore              mutex;

        ObjectRecognizerPort   *imagePort;

        Port                   rpcPort;
        RpcServer              rpcPortHuman;

    public:

        ObjectRecognizerModule();
        bool configure(ResourceFinder &rf);
        bool interruptModule();
        bool close();
        bool respond(const Bottle &command, Bottle &reply);
        double getPeriod();
        bool updateModule();

};

#endif /* end of include guard: OBJECT_RECOGNIZER_MODULE_H */
