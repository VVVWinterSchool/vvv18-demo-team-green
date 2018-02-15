#include "machine.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);
    //rf.setDefaultContext("MachineModule");

    MachineModule module;
    return module.runModule(rf);;
}
