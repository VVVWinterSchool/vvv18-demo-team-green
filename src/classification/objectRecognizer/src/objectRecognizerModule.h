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

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define RADIUS 256
#define FLAG_UNIT_TEST true

class ObjectRecognizerModule: public RFModule
{
    protected:

        yarp::os::Mutex        mutex;
        bool                   noObject;

        BufferedPort<ImageOf<PixelRgb> > imageInport;
        BufferedPort<Bottle>   segmentationInport;
        RpcServer              userPrefInport;

        BufferedPort<Vector>   positionOutport;
        BufferedPort<ImageOf<PixelRgb> > port_out_view;
        Port                   port_out_scores;

        CaffeWrapper<float>    *caffe_wrapper;

        vector<cv::Scalar>     colors;

        string*                labels;
        int                    n_classes;

        int                    radius;

        std::map<std::string, Vector> classifPosMap;
        std::map<std::string, float> classifScoreMap;

        bool classify(cv::Mat& image_cropped, float& max_score, int& classObject);
        bool cropImage(const cv::Mat& in_image, cv::Mat& out_image, cv::Point tl, cv::Point br);

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
