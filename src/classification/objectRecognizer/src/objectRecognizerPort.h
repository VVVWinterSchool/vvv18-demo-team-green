#ifndef OBJECT_RECOGNIZER_PORT_H
#define OBJECT_RECOGNIZER_PORT_H

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

#define FLAG_UNIT_TEST true

class ObjectRecognizerPort: public BufferedPort<Image>
{
    private:
        // Resource Finder and module options

        Semaphore              mutex;

        cv::Mat                img_mat;
        cv::Mat                img_crop_mat;

        int                    radius;

        CaffeWrapper<float>    *caffe_wrapper;

        BufferedPort<Bottle>   port_in_centroid;
        BufferedPort<Bottle>   port_in_roi;

        Port                   port_out_view;
        Port                   port_out_scores;
        Port                   port_out_hist;

        vector<cv::Scalar>     colors;

        string*                labels;
        int                    n_classes;

        void onRead(Image &img);

    public:
        ObjectRecognizerPort(ResourceFinder &rf);
        bool set_radius(int _radius);
        bool get_radius(int &_radius);
        void interrupt();
        void resume();
        void close();

};

#endif /* end of include guard: OBJECT_RECOGNIZER_PORT_H */
