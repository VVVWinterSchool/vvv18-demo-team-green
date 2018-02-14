#include "segment_crop.h"

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    this->rf=&rf;
    std::string moduleName = rf.check("name", yarp::os::Value("segment_crop"), "module name (string)").asString();
    setName(moduleName.c_str());
    inDispPort = new CustomProcessor(moduleName);
    closing = false;
    inDispPort->threshold_value = 70;

    rpcPort.open("/" + moduleName + "/service");
    attach(rpcPort);
    inDispPort->useCallback();
    inDispPort->open();

    if (inDispPort->queryClient.getOutputCount()==0)
    {

    }

    return true;
}

bool Module::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{
    inDispPort->threshold_value = command.get(0).asInt();
    reply.addString("ack");
    return true;
}

bool Module::close()
{
    inDispPort->interrupt();
    inDispPort->close();
    delete inDispPort;
    return true;
}

bool Module::quit()
{
    closing = true;
    return true;
}

double Module::getPeriod()
{
    return 0.1;
}

bool Module::updateModule()
{
    return !closing;
}

CustomProcessor::CustomProcessor(const std::string &moduleName)
{
    this->moduleName = moduleName;
    queryClient.open("/"+moduleName+"/rpcSFM");
}

bool CustomProcessor::open()
{
    //this->useCallback();
    this->BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
    //inRGBPort.open("/" + moduleName + "/RGBimage:i");
    outStuffPort.open("/"+ moduleName + "/crops:o");
    outDebugPortRGB.open("/"+ moduleName + "/debugRGB:o");
}

void CustomProcessor::close()
{
    outDebugPortRGB.close();
    outStuffPort.close();
    //inRGBPort.close();
    this->BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::close();
}

void CustomProcessor::interrupt()
{
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::interrupt();
}

void CustomProcessor::onRead(yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outDebugRGB = outDebugPortRGB.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inRGB = inRGBPort.read();

    outDebugRGB.resize(dispImage.width(),dispImage.height());
    outDebugRGB.zero();

    cv::Mat disp = cv::cvarrToMat((IplImage*)dispImage.getIplImage());
    cv::Mat processed_disp = disp.clone();


    cv::GaussianBlur(processed_disp,processed_disp,cv::Size(3, 3),2,2);
    cv::dilate(processed_disp,processed_disp,cv::Mat(),cv::Point(-1,-1),2, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
    cv::erode(processed_disp,processed_disp,cv::Mat(),cv::Point(-1,-1),3,cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

    cv::threshold(processed_disp,processed_disp,threshold_value,255,cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(processed_disp,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

    cv::Rect bounding_box_1;
    cv::Rect bounding_box_2;
    yarp::sig::Vector  object_1;
    yarp::sig::Vector  object_2;

    if(contours.size() == 2)
    {
        bounding_box_1 = cv::boundingRect(contours[0]);
        bounding_box_2 = cv::boundingRect(contours[1]);


    }

    cv::cvtColor(processed_disp,processed_disp,CV_GRAY2RGB);

    //cv::rectangle(disp,cv::Rect(disp.cols/2, disp.rows/2, disp.cols/2 + disp.cols/4, disp.rows/2 + disp.rows/4),cv::Scalar(255,0,0));

    cv::resize(processed_disp,processed_disp,cv::Size(outDebugRGB.width(),outDebugRGB.height()));
    IplImage out = processed_disp;

    cvCopy(&out,(IplImage*) outDebugRGB.getIplImage());
    outDebugPortRGB.write();

}



















