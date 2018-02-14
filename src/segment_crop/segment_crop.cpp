#include "segment_crop.h"

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    std::string moduleName = rf.check("name", yarp::os::Value("closest-blob"), "module name (string)").asString();
    setName(moduleName.c_str());
    inDispPort = new CustomProcessor(moduleName);
    return true;
}

bool Module::close()
{
    return true;
}

bool Module::quit()
{
    return true;
}

double Module::getPeriod()
{
    return 0.1;
}

bool Module::updateModule()
{
    return true;
}

CustomProcessor::CustomProcessor(const std::string &moduleName)
{

}

void CustomProcessor::onRead(yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
{

}

bool CustomProcessor::open()
{

}

void CustomProcessor::close()
{

}

void CustomProcessor::interrupt()
{

}

