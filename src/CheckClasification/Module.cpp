#include <Module.h>

#include <cmath>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

// iDynTree headers
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConversions.h>

void addVectorOfStringToProperty(yarp::os::Property& prop, std::string key, std::vector<std::string> & list)
{
    prop.addGroup(key);
    yarp::os::Bottle & bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

void convertDegToRad(const yarp::sig::Vector& vecDeg, yarp::sig::Vector& vecRad)
{
    if (vecDeg.size() != vecRad.size()) {
        yError() << "convertDegToRad: wrong vector size";
        return;
    }

    for (size_t i=0; i < vecDeg.size(); i++) {
        vecRad[i] = (M_PI/180.0)*vecDeg[i];
    }
}

void convertRadToDeg(const yarp::sig::Vector& vecRad, yarp::sig::Vector& vecDeg)
{
    if (vecDeg.size() != vecRad.size()) {
        yError() << "convertDegToRad: wrong vector size";
        return;
    }

    for (size_t i=0; i < vecRad.size(); i++) {
        vecDeg[i] = (180.0/M_PI)*vecRad[i];
    }
}

double Module::getPeriod () { return 0.01; }

bool Module::updateModule ()
{
    yarp::os::Bottle *flagin;

    if (!go){
        flagin = inPutFlagPort.read();
        if (flagin == NULL)
        {
            yError() << "Error reading flag input port";
           // return -1;
        }
        if (flagin->get(0).asInt() == 1){
            go = true;
        }
    }

    yarp::sig::Vector *input= externalForcesPort.read();

    if (input == NULL)
    {
        yError() << "Error reading forcers port";
       // return -1;
    }
    
    externalForces = *input;
    yInfo() <<"external Forces: " << externalForces.toString();
    double threshold = 1.0;

    yarp::os::Bottle &outbot = outPutFlagPort.prepare();
    if (externalForces(0) > threshold)
    {
        outbot.addInt(1);
        yInfo() << "happy!";
        outPutFlagPort.write();

    }else if (externalForces(0) < -threshold)
    {
        outbot.addInt(0);
        yInfo() << "sad!";
        outPutFlagPort.write();
    }else{
        //Do nothing
    }

    return true;
}

bool Module::configure (yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    externalForces.resize(8);

    //BufferedPort<yarp::sig::Vector> inPort;

    if (!externalForcesPort.open("/CheckClassification/rigthArmForces:i"))
    {
        yError() << "cannot open the input port";
        return -1;
    }

    if (!inPutFlagPort.open("/CheckClassification/flagInput:i"))
    {
        yError() << "cannot open the flag input port";
        return -1;
    }

    if (!outPutFlagPort.open("/CheckClassification/rigthArmForces:i"))
    {
        yError() << "cannot open the input port";
        return -1;
    }
    go = false;
    return true;
}

bool Module::close ()
{
    // FILL IN THE CODE
    // hint: do any cleanup that is necessary
    externalForcesPort.close();
    return true;
}

