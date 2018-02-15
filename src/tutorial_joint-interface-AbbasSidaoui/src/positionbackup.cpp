// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::os;
using namespace yarp::dev;


class CtrlModule: public RFModule
{
protected:
    PolyDriver         clienJoint;
    IControlLimits2   *ilim;
    IEncoders         *ienc;
    IControlMode2     *imod; //used
    IPositionControl2 *ipos;

    RpcServer rpc;
    int joint;
//*****************Added***************/
    IControlLimits2   *tdlim; //thumb distal
    IControlLimits2   *mdlim; //middle distal
    IControlLimits2   *pdlim; //pinky distal
    IEncoders         *tdenc;
    IEncoders         *mdenc;
    IEncoders         *pdenc;
    int thumb_p = 9;
    int thumb_d = 10;
    int middle_p = 13;
    int middle_d = 14;
    int pinky = 15;

    // select target
    double f_target(double f_encv, double f_min, double f_max, double f_range){
   double f_target;
   if (fabs(f_encv-f_min)<fabs(f_encv-f_max))
       f_target=f_max-0.1*f_range;
   else
       f_target=f_min+0.1*f_range;
    }
//***************Finished Addidng******/



    void go()
    {
        // retrieve joint bounds
        double min,max,range;
        ilim->getLimits(joint,&min,&max);
        range=max-min;


 //****************Added*************************/
        //thumb distal
        double tdmin,tdmax,tdrange;
        tdlim->getLimits(thumb_d,&tdmin,&tdmax);
        tdrange=tdmax-tdmin;
        //middle distal
        double mdmin,mdmax,mdrange;
        mdlim->getLimits(middle_d,&mdmin,&mdmax);
        mdrange=mdmax-mdmin;
        //pinky distal
        double pdmin,pdmax,pdrange;
        pdlim->getLimits(pinky,&pdmin,&pdmax);
        pdrange=pdmax-pdmin;

        //retrieve current joint position
        double tdencv,mdencv,pdencv;
        tdenc->getEncoder(thumb_d,&tdencv);
        mdenc->getEncoder(middle_p,&mdencv);
        pdenc->getEncoder(pinky,&mdencv);

        // select target
        double t_target,m_target,p_target;
        t_target = f_target(tdencv,tdmin,tdmax,tdrange); //thumb target pos
        m_target = f_target(mdencv,mdmin,mdmax,mdrange); //middle target pos
        p_target = f_target(pdencv,pdmin,pdmax,pdrange); //pinky target pos

        // set control mode
        imod->setControlMode(thumb_d,VOCAB_CM_POSITION);
        imod->setControlMode(middle_d,VOCAB_CM_POSITION);
        imod->setControlMode(pinky,VOCAB_CM_POSITION);

        // set up the speed in [deg/s]
        ipos->setRefSpeed(thumb_d,20.0);
        ipos->setRefSpeed(middle_d,20.0);
        ipos->setRefSpeed(pinky,20.0);

        // set up max acceleration in [deg/s^2]
        ipos->setRefAcceleration(thumb_d,100.0);
        ipos->setRefAcceleration(middle_d,100.0);
        ipos->setRefAcceleration(pinky,100.0);

        // yield the actual movement
        yInfo()<<"Yielding new thumb target: "<<t_target<<" [deg]";
        ipos->positionMove(thumb_d,t_target);
        ipos->positionMove(middle_d,m_target);
        ipos->positionMove(pinky,p_target);

 //***************Finished Addidng******/

        // retrieve current joint position done
        double enc;
        ienc->getEncoder(joint,&enc);

        // select target done
        double target;
        if (fabs(enc-min)<fabs(enc-max))
            target=max-0.1*range;
        else
            target=min+0.1*range;

        // set control mode done
        imod->setControlMode(joint,VOCAB_CM_POSITION);

        // set up the speed in [deg/s]     done
        ipos->setRefSpeed(joint,30.0);

        // set up max acceleration in [deg/s^2]   done
        ipos->setRefAcceleration(joint,100.0);

        // yield the actual movement              done
        yInfo()<<"Yielding new target: "<<target<<" [deg]";
        ipos->positionMove(joint,target);

        // wait (with timeout) until the movement is completed
        bool done=false;
        double t0=Time::now();
        while (!done&&(Time::now()-t0<10.0))
        {
            yInfo()<<"Waiting...";
            Time::delay(0.1);   // release the quantum to avoid starving resources
            ipos->checkMotionDone(&done);
        }

        if (done)
            yInfo()<<"Movement completed";
        else
            yWarning()<<"Timeout expired";
    }

public:
    virtual bool configure(ResourceFinder &rf)
    {
        // open a client interface to connect to the joint controller
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/icubSim/left_arm");
        optJoint.put("local","/position/left_arm");

        if (!clienJoint.open(optJoint))
        {
            yError()<<"Unable to connect to /icubSim/left_arm";
            return false;
        }

        // open views
        bool ok=true;
        ok=ok && clienJoint.view(ilim);
        ok=ok && clienJoint.view(ienc);
        ok=ok && clienJoint.view(imod);
        ok=ok && clienJoint.view(ipos);

        if (!ok)
        {
            yError()<<"Unable to open views";
            return false;
        }

        // elbow
        joint=3;

        // open rpc port
        rpc.open("/position");

        // attach the callback respond()
        attach(rpc);

        return true;
    }

    virtual bool close()
    {
        rpc.close();
        clienJoint.close();

        return true;
    }

    virtual bool respond(const Bottle &cmd, Bottle &reply)
    {
        if (cmd.get(0).asString()=="go")
        {
            go();
            reply.addString("ack");
        }
        else if (cmd.get(0).asString()=="enc")
        {
            double enc;
            ienc->getEncoder(joint,&enc);
            reply.addString("ack");
            reply.addDouble(enc);
        }
        else
            reply.addString("nack");

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
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
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}

