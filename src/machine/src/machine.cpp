#include "machine.h"

bool MachineModule::openPorts(const string &moduleName)
{
    bool status=false;
    status=commandPort.open("/"+moduleName+"/rpc:i");
    status&=speechRecogn.open("/userPreference/control/rpc:o");
    status&=noObjectIn.open("/segment_crop/valid_detection:o");
    status&=objectBottleIn.open("/objectRecognizer/position:i");
    status&=objectBottleOut.open("/objectPointing/position:o");
    status&=reachedPosIn.open("/high_five_ready:i");
    status&=rewardDetectorOut.open("/checkClassification/flag:o");
    status&=feedbackIn.open("/checkClassification/flag:i");
    //status&=homeOut.open("/homeOut/flag:o");
    //status&=homeIn.open("/homeIn/flag:i");
    status&=positionControl.open("/position:o");

    if(!status)
    {
        yError()<<"failed to open the ports";
        return false;
    }

    if(!attach(commandPort)) {
        yError()<<"cannot attach to the commandPort";
        return false;
    }

    return true;
}

bool MachineModule::configure(ResourceFinder &rf)
{
    yInfo()<<"configuring MachineModule...";
    string robot=rf.check("robot", Value("icubSim")).asString();
    string moduleName=rf.check("name", Value("MachineModule")).asString();

    if(!openPorts(moduleName))
    {
        return false;
    }

    state=IDLE;
    feedback=2;

    return true;
}

double MachineModule::getPeriod(void)
{
    return 0.5;
}

bool MachineModule::updateModule(void)
{
    switch(state)
    {
    case IDLE:
    {
        state=CMD;
        yInfo()<<"idle state...";
        break;
    }
    case CMD:
    {
        // read unblocking
        break;
    }
    case POINT_ACTION:
    {
        yInfo()<<"point_action state...";
        Bottle cmdBottle, replyBottle;
        cmdBottle.clear();
        cmdBottle.addString("send");
        speechRecogn.write(cmdBottle, replyBottle);
        yInfo()<<"sending vocal command and waiting for 3D position...";
        // fill: no object detected
        //Vector *posBottleIn=objectBottleIn.read(true);
        Bottle *posBottleIn=objectBottleIn.read(true);
        if(posBottleIn==NULL)
        {
            yWarning()<<"empty position bottle...";
            state=IDLE;
            return false;
        }
        //yInfo()<<"got "<<posBottleIn->operator[](0)<<" "<<posBottleIn->operator[](1)<<" "<<posBottleIn->operator[](2);
        if(posBottleIn->size()!=3)
        {
            yWarning()<<"posBottleIn not size 3...";
            state=IDLE;
            break;
        }
        yInfo()<<"got "<<posBottleIn->get(0).asDouble()<<" "<<posBottleIn->get(1).asDouble()<<" "<<posBottleIn->get(2).asDouble();
        cmdBottle.clear();
        cmdBottle.addString("Close_hand");
        replyBottle.clear();
        positionControl.write(cmdBottle, replyBottle);
        yInfo()<<"closing hand...";
        yInfo()<<"sending reaching position...";
        Bottle &posBottleOut=objectBottleOut.prepare();
        //posBottleOut.clear();
        //posBottleOut.addDouble(posBottleIn->operator[](0));
        //posBottleOut.addDouble(posBottleIn->operator[](1));
        //posBottleOut.addDouble(posBottleIn->operator[](2));
        posBottleOut=*posBottleIn;
        objectBottleOut.write();
        reachedPosIn.read(true);
        yInfo()<<"point action done...";
        Time::delay(5.0);
        cmdBottle.clear();
        cmdBottle.addString("HighFive");
        replyBottle.clear();
        positionControl.write(cmdBottle, replyBottle);
        Time::delay(3.0);
        yInfo()<<"arm in high five position";
        state=REWARD;
        break;
    }
    case REWARD:
    {
        Bottle cmdBottle, replyBottle;
        yInfo()<<"reward state...";
        yInfo()<<"sending command to start reward detector...";
        Bottle &startReward=rewardDetectorOut.prepare();
        startReward.clear();
        startReward.addInt(1);
        rewardDetectorOut.write();
        yInfo()<<"waiting for feedback...";
        Bottle *feedbackBottle=feedbackIn.read(true);
        if(feedbackBottle==NULL)
        {
            yWarning()<<"empty feedback...";
            state=IDLE;
            return false;
        }
        feedback=feedbackBottle->get(0).asInt();
        yInfo()<<"got "<<feedback;
        cmdBottle.clear();
        if(feedback==0)
            cmdBottle.addString("Shy");
        else if(feedback==1)
            cmdBottle.addString("Happy");
        else
            yInfo()<<"unkown cmd";
        replyBottle.clear();
        positionControl.write(cmdBottle, replyBottle);
        Time::delay(10.0);
        state=HOME;
        break;
    }
    case HOME:
    {
        yInfo()<<"sending home state...";
        Bottle cmdBottle, replyBottle;
        cmdBottle.clear();
        cmdBottle.addString("home");
        replyBottle.clear();
        positionControl.write(cmdBottle, replyBottle);
        Time::delay(3.0);
        //Bottle &startHome=homeOut.prepare();
        //startHome.clear();
        //startHome.addInt(feedback);
        yInfo()<<"sending feedback..."<<feedback;
        //homeOut.write();
        //yInfo()<<"waiting for done flag...";
        //if(homeIn.read(true)==NULL)
        //{
            //yWarning()<<"empty homeIn...";
            //state=IDLE;
            //return false;
        //}
        yInfo()<<"in home...";
        feedback=2;
        state=IDLE;
        break;
    }
    case QUIT:
    {
        yInfo()<<"quit state...";
        close();
        break;
    }
    }

    return true;
}

bool MachineModule::respond(const Bottle &command, Bottle &reply)
{
    cmdStr=command.get(0).asString();
    reply.clear();
    if(cmdStr=="help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands:");
        reply.addString("- classify");
        reply.addString("- home");
        reply.addString("- quit");
        return true;
    }

    if(state!=CMD)
    {
        reply.addString("not in cmd state");
        return true;
    }

    if(cmdStr=="quit")
    {
        reply.addString("ack");
        reply.addString("quitting...");
        return false;
    }
    else if(cmdStr=="home")
    {
        state=HOME;
        feedback=2;
        reply.addString("ack");
        reply.addString("going home...");
    }
    else if(cmdStr=="classify")
    {
        state=POINT_ACTION;
        reply.addString("ack");
        reply.addString("going to classify...");
    }
    else
    {
        reply.addString("unknown command");
    }

    return true;
}

bool MachineModule::interruptModule(void)
{
    yInfo()<<"interrupting machine module...";
    commandPort.interrupt();
    speechRecogn.interrupt();
    noObjectIn.interrupt();
    objectBottleIn.interrupt();
    objectBottleOut.interrupt();
    reachedPosIn.interrupt();
    rewardDetectorOut.interrupt();
    feedbackIn.interrupt();
    //homeOut.interrupt();
    //homeIn.interrupt();
    positionControl.interrupt();
    return true;
}

bool MachineModule::close(void)
{
    yInfo()<<"closing machine module...";
    commandPort.close();
    speechRecogn.close();
    noObjectIn.close();
    objectBottleIn.close();
    objectBottleOut.close();
    reachedPosIn.close();
    rewardDetectorOut.close();
    feedbackIn.close();
    //homeOut.close();
    //homeIn.close();
    positionControl.close();
    return true;
}
