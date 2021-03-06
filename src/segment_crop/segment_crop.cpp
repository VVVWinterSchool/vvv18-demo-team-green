#include "segment_crop.h"

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    this->rf=&rf;
    std::string moduleName = rf.check("name", yarp::os::Value("segment_crop"), "module name (string)").asString();
    setName(moduleName.c_str());
    inDispPort = new CustomProcessor(moduleName);
    closing = false;
    inDispPort->threshold_value = 50;

    rpcPort.open("/" + moduleName + "/service");
    attach(rpcPort);
    inDispPort->useCallback();
    inDispPort->open();
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
    valid_detection_timer_threshold = 10.0;
    valid_detection_timer = yarp::os::Time::now();
}

bool CustomProcessor::open()
{
    //this->useCallback();
    this->BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
    inRGBPort.open("/" + moduleName + "/RGBimage:i");
    outStuffPort.open("/"+ moduleName + "/crops:o");
    outDebugPortRGB.open("/"+ moduleName + "/debugRGB:o");
    detection_timeout.open("/" + moduleName + "/valid_detection:o");
}

void CustomProcessor::close()
{
    outDebugPortRGB.close();
    outStuffPort.close();
    detection_timeout.close();
    inRGBPort.close();
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
    yarp::os::Bottle &outBottle = outStuffPort.prepare();

    outDebugRGB.resize(dispImage.width(),dispImage.height());
    outDebugRGB.zero();

    cv::Mat disp = cv::cvarrToMat((IplImage*)dispImage.getIplImage());
    cv::Mat rgb_with_bb = cv::cvarrToMat((IplImage*)inRGB->getIplImage());
    cv::Mat processed_disp = disp.clone();

    // blur the disparity map to soften edges
    cv::GaussianBlur(processed_disp,processed_disp,cv::Size(3, 3),2,2);

    // threshold the disp map to obtain bin image
    cv::threshold(processed_disp,processed_disp,threshold_value,255,cv::THRESH_BINARY);

    // fix morphology of the blobs
    cv::erode(processed_disp,processed_disp, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(processed_disp,processed_disp,cv::Mat(),cv::Point(-1,-1),3, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
    //cv::erode(processed_disp,processed_disp,cv::Mat(),cv::Point(-1,-1),3,cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

    //  find blob contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(processed_disp.clone(),contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    //  convert the processed disp map to rgb
    cv::cvtColor(processed_disp,processed_disp,CV_GRAY2RGB);

    //  display all the bounding boxes
    for (int i=0; i<contours.size(); i++)
    {
        cv::Rect b_box = cv::boundingRect(contours[i]);
        cv::rectangle(processed_disp, b_box, cv::Scalar(0, 255, 255), 2);
        cv::rectangle(rgb_with_bb, b_box, cv::Scalar(0, 255, 255), 2);
    }

    //  init structures to contain bounding boxes
    cv::Rect bounding_box_1;
    cv::Rect bounding_box_2;

    yarp::sig::Vector centre_1;
    yarp::sig::Vector centre_2;
    centre_1.resize(2);
    centre_2.resize(2);

    //  init vector of 3D object centers
    yarp::sig::Vector  object_1;
    yarp::sig::Vector  object_2;

    //  detecting 2 contours means 2 objects are being shown. Otherwise, no coordinate stream happens
    if(contours.size() == 2 )
    {
        bounding_box_1 = cv::boundingRect(contours[0]);
        bounding_box_2 = cv::boundingRect(contours[1]);

        //  prepare some data for output stream

        centre_1[0] = (bounding_box_1.tl().x + bounding_box_1.br().x)/2;
        centre_1[1] = (bounding_box_1.tl().y + bounding_box_1.br().y)/2;

        centre_2[0] = (bounding_box_2.tl().x + bounding_box_2.br().x)/2;
        centre_2[1] = (bounding_box_2.tl().y + bounding_box_2.br().y)/2;

        yarp::os::Bottle cmd, response;

        cmd.addString("Points");
        cmd.addInt(centre_1[0]);
        cmd.addInt(centre_1[1]);
        cmd.addInt(centre_2[0]);
        cmd.addInt(centre_2[1]);

        yInfo() << "Querying SFM: " << cmd.toString().c_str();

        queryClient.write(cmd,response);
        yInfo() << "Got response : " << response.toString().c_str();

        object_1.push_back(response.get(0).asDouble());
        object_1.push_back(response.get(1).asDouble());
        object_1.push_back(response.get(2).asDouble());
        object_2.push_back(response.get(3).asDouble());
        object_2.push_back(response.get(4).asDouble());
        object_2.push_back(response.get(5).asDouble());

        //  if disparity map has some errors, the object positions will not be valid
        //  in this case, nothing is streamed
        if (object_1[0] < -0.2 && object_2[0] < -0.2)
        {
            yarp::os::Bottle listObj1_Rect;
            yarp::os::Bottle listObj2_Rect;
            yarp::os::Bottle listObj1_pos;
            yarp::os::Bottle listObj2_pos;

            listObj1_Rect.addInt(bounding_box_1.tl().x);
            listObj1_Rect.addInt(bounding_box_1.tl().y);
            listObj1_Rect.addInt(bounding_box_1.br().x);
            listObj1_Rect.addInt(bounding_box_1.br().y);

            listObj2_Rect.addInt(bounding_box_2.tl().x);
            listObj2_Rect.addInt(bounding_box_2.tl().y);
            listObj2_Rect.addInt(bounding_box_2.br().x);
            listObj2_Rect.addInt(bounding_box_2.br().y);

            listObj1_pos.addDouble(object_1[0]);
            listObj1_pos.addDouble(object_1[1]);
            listObj1_pos.addDouble(object_1[2]);

            listObj2_pos.addDouble(object_2[0]);
            listObj2_pos.addDouble(object_2[1]);
            listObj2_pos.addDouble(object_2[2]);

            outBottle.clear();

            outBottle.addList() = listObj1_Rect;
            outBottle.addList() = listObj1_pos;
            outBottle.addList() = listObj2_Rect;
            outBottle.addList() = listObj2_pos;

            yInfo() << outBottle.toString();

            outStuffPort.write();

            valid_detection_timer = yarp::os::Time::now();
        }
    }
    else
    {
        // detect timeout
        if (yarp::os::Time::now() - valid_detection_timer > valid_detection_timer_threshold)
        {
            // signal the timeout ONCE
            yarp::os::Bottle &b =  detection_timeout.prepare();
            b.clear();
            b.addInt(-1);
            detection_timeout.write();
            yDebug() << "Timeout! Show me 2 objects...";
            // clear timer
            valid_detection_timer =yarp::os::Time::now();

        }
    }

    //  prepare debug disp map and stream it
    cv::resize(processed_disp,processed_disp,cv::Size(outDebugRGB.width(),outDebugRGB.height()));
    ///IplImage out = processed_disp;

    //cvCopy(&out,(IplImage*) outDebugRGB.getIplImage());
    //outDebugPortRGB.write();

    IplImage out = rgb_with_bb;

    cvCopy(&out,(IplImage*) outDebugRGB.getIplImage());
    outDebugPortRGB.write();

}



















