#include "objectRecognizerModule.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

ObjectRecognizerModule::ObjectRecognizerModule()
{
    imagePort = NULL;
}

bool ObjectRecognizerModule::configure(ResourceFinder &rf)
{

    Time::turboBoost();

    // module name
    string name = rf.find("name").asString().c_str();

    // input port
    imagePort = new ObjectRecognizerPort(rf);
    imagePort->open(("/"+name+"/img:i").c_str());

    // parameters
    int radius = rf.check("radius",Value(256)).asInt();

    imagePort->set_radius(radius);

    // rpc ports
    rpcPortHuman.open(("/"+name+"/human:io").c_str());

    rpcPort.open(("/"+name+"/rpc").c_str());
    attach(rpcPort);

    return true;
}

bool ObjectRecognizerModule::interruptModule()
{
    if (imagePort!=NULL)
    imagePort->interrupt();

    rpcPort.interrupt();
    rpcPortHuman.interrupt();

    return true;
}

bool ObjectRecognizerModule::close()
{
    if (imagePort!=NULL)
    {
        imagePort->close();
        delete imagePort;
    }

    rpcPort.close();
    rpcPortHuman.close();

    return true;
}

bool ObjectRecognizerModule::respond(const Bottle &command, Bottle &reply)
{
    return RFModule::respond(command,reply);
}

double ObjectRecognizerModule::getPeriod()    { return 1.0;  }

bool ObjectRecognizerModule::updateModule()
{
    bool ok = false;

    Bottle command,reply;
    rpcPortHuman.read(command,true);

    if (command.size()>0)
    {

        mutex.wait();

        switch(command.get(0).asVocab())
        {

            case CMD_HELP:
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString(" ");
                reply.addString(" ");
                reply.addString("set radius <value>       [ int>0  ]: sets the square radius if 'centroid' or 'fixed' mode is on");
                reply.addString(" ");
                reply.addString("get radius               : provides the radius of the square ROI, if 'radius' mode is on");

                ok = true;

            } break ;

            case CMD_SET:
            {
                if (command.size()>2)
                {
                    string property = command.get(1).asString().c_str();

                    if (property == "radius")
                    {
                        int r = command.get(2).asInt();
                        ok = imagePort->set_radius(r);
                    }
                    else
                    {
                        ok = false;
                        reply.addString("Unknown property.");
                        break;
                    }

                } else
                {
                    ok = false;
                    reply.addString("Syntax must be: set <prop> <value>");
                    break;
                }

                if (ok)
                reply.addVocab(ACK);
                else
                reply.addString("Cannot set property: check value.");

                break;

            }

            case CMD_GET:
            {
                if (command.size()>1)
                {
                    string property = command.get(1).asString().c_str();

                    if (property=="radius")
                    {
                        int r;
                        ok = imagePort->get_radius(r);
                        reply.addInt(r);
                        break;
                    }
                    else
                    {
                        ok = false;
                        reply.addString("Unknown property.");
                        break;
                    }
                }
                else
                {
                    ok = false;
                    reply.addString("Syntax must be: get <property>");
                    break;
                }

            }

            default:
            reply.addString("Unknown command!");

        }

        mutex.post();

        rpcPortHuman.reply(reply);

    }

    return true;
}
