#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl2.h>

#include <yarp/os/BufferedPort.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class ClientMod : public yarp::os::RFModule
{
private:

    // define one output port to comunicate to the server and
    // one input port to receive the trigger to start moving the
    // arm
    // hint: think about which port you need, buffered? simple? both?
    BufferedPort<Bottle> outputPortToServer;

    bool messageSentToServer = false;

public:

    // set the correct angle to send to the server and the period of the thread
    //ClientMod(): //angle(29.0), period(2.0), triggered(false)
    ClientMod(){}
    /****************************************************/
    bool configPorts()
    {
        // open all ports and check that everything is fine
        // output port: /client/output

        // Output port
        //BufferedPort<Bottle> outputPortToServer;
        if (!outputPortToServer.open("/objectPointing/position:o"))
        {
            yError()<<"error opening port from client to server";
            return false;
        }
        yInfo()<<"Success in opening the port";

        return true;
    }

    /****************************************************/
    bool configure(ResourceFinder &rf)
    {
        // configure the ports
        bool conf = configPorts();

        return conf;
    }

    /****************************************************/
    double getPeriod()
    {
        return 1.0; //period;
    }

    /****************************************************/
    bool close()
    {
        // close ports
        outputPortToServer.close();
        return true;
    }

    /****************************************************/
    bool interrupt()
    {
        // interrupt ports
        outputPortToServer.interrupt();
        return true;
    }

    /****************************************************/
    bool updateModule()
    {
        if (!messageSentToServer)
        {
            // output port
            Bottle &message=outputPortToServer.prepare();
            message.clear(); //important, objects get recycled
            double x = -0.7;
            double y = 0.5;
            double z = 1.2;
            message.addDouble(x);
            message.addDouble(y);
            message.addDouble(z);
            outputPortToServer.write();

            messageSentToServer = true;
        }
        else
        {
            yInfo()<<"Message already sent to the server";
        }

        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.setDefaultContext("assignment_motor-control");
    rf.configure(argc,argv);

    ClientMod mod;
    return mod.runModule(rf);
}

