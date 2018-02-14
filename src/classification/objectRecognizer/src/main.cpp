#include "objectRecognizerModule.h"
#include "objectRecognizerPort.h"

int main(int argc, char *argv[]){

    Network yarp;

    if (!yarp.checkNetwork())
    return 1;

    ResourceFinder rf;

    rf.setVerbose(true);

    rf.setDefaultContext("objectRecognizer");
    rf.setDefaultConfigFile("objectRecognizer.ini");

    rf.configure(argc,argv);

    rf.setDefault("name","objectRecognizer");

    ObjectRecognizerModule mod;

    return mod.runModule(rf);
}
