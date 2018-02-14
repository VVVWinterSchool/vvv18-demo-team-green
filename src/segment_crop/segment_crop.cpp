#include "segment_crop.h"

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    this->rf=&rf;
    std::string moduleName = rf.check("name", yarp::os::Value("segment_crop"), "module name (string)").asString();
    setName(moduleName.c_str());
    inDispPort = new CustomProcessor(moduleName);
    closing = false;

    inDispPort->open();

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
}

bool CustomProcessor::open()
{
    this->useCallback();
    this->BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
    inRGBPort.open("/" + moduleName + "/RGBimage:i");
    outStuffPort.open("/"+ moduleName + "/crops:o");
}

void CustomProcessor::close()
{
    outStuffPort.close();
    inRGBPort.close();
    this->close();
}

void CustomProcessor::interrupt()
{
    this->interrupt();
}

void CustomProcessor::onRead(yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
{

}
