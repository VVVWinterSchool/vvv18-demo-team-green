// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>
#include <string>
#include <cmath>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlModule: public RFModule
{
protected:
    PolyDriver         clienJoint;
    PolyDriver         leftArm;
    PolyDriver         rightArm;
    PolyDriver         head;
    PolyDriver         torso;
    PolyDriver         c_l,c_r,c_h,c_t;
    IControlLimits2   *ilim;
    IEncoders         *ienc;
    IControlMode2     *imod, *imod_left, *imod_right, *imod_head, *imod_torso; //used
    IPositionControl2 *ipos, *ipos_left, *ipos_right, *ipos_head, *ipos_torso;
    ICartesianControl *ileft_arm, *iright_arm, *ihead, *itorso ;
    Vector l_pos,l_o,r_pos,r_o,t_pos,t_o,h_pos,h_o;
    RpcServer rpc;
    int joint;
//*****************Added***************/
    IControlLimits2   *tdlim; //thumb distal
    IControlLimits2   *mdlim; //middle distal
    IControlLimits2   *pdlim; //pinky distal
    IEncoders         *tdenc;
    IEncoders         *mdenc;
    IEncoders         *pdenc;

    //arm:
    int shoulder_pitch = 0;
    int shoulder_roll = 1;
    int shoulder_yaw = 2;
    int elbow = 3;
    int wrist_grosup = 4;
    int wrist_pitch = 5;
    int wrist_yaw = 6;
    int hand_finger = 7;
    int thumb_oppose = 8;
    int thumb_p = 9;
    int thumb_d = 10;
    int index_p = 11;
    int index_d = 12;
    int middle_p = 13;
    int middle_d = 14;
    int pinky = 15;

    //Torso
    int torso_yaw = 0;
    int torso_roll = 1;
    int torso_pitch = 2;

    //Head
    int neck_pitch = 0;
    int neck_roll = 1;
    int neck_yaw = 2;
    int eyes_tilt = 3;
    int eyes_version = 4;
    int eyes_vergence = 5;



    // select target
    double f_target(double f_encv, double f_min, double f_max, double f_range){
   double f_target;
   if (fabs(f_encv-f_min)<fabs(f_encv-f_max))
       f_target=f_max-0.1*f_range;
   else
       f_target=f_min+0.1*f_range;
   return f_target;
    }
/*  void getpose(){
      ileft_arm->getPose(l_pos,l_o);
      iright_arm->getPose(r_pos,r_o);
 yInfo()<<"l_pose"<<l_pos[0];
 yInfo()<<"r_pose"<<r_pos[0];


  }*/
  void HighFive(){
        JointConf();
       //position move
       ipos_right->positionMove(shoulder_pitch, -64.18);
       ipos_right->positionMove(shoulder_roll,17.4);
       ipos_right->positionMove(shoulder_yaw,12.0);
       ipos_right->positionMove(elbow,90.0);
       ipos_right->positionMove(wrist_grosup,90.0);
       ipos_right->positionMove(wrist_pitch,0);
       ipos_right->positionMove(wrist_yaw,0);
       ipos_right->positionMove(hand_finger,55.0);
       ipos_right->positionMove(thumb_oppose,20);
       ipos_right->positionMove(thumb_p,20);
       ipos_right->positionMove(thumb_d,0);
       ipos_right->positionMove(index_p,0);
       ipos_right->positionMove(index_d,10);
       ipos_right->positionMove(middle_p,10);
       ipos_right->positionMove(middle_d,10);
       ipos_right->positionMove(pinky,10);

       ipos_head->positionMove(neck_pitch,0.0);
       ipos_head->positionMove(neck_roll,0.0);
       ipos_head->positionMove(neck_yaw,0.0);

       ipos_torso->positionMove(torso_pitch,0.0);
       ipos_torso->positionMove(torso_roll,0.0);
       ipos_torso->positionMove(torso_yaw,0.0);

       bool head_done3=false;
       bool torso_done3=false;
       bool left_done3=false;
       bool right_done3=false;
       double hf_t0=Time::now();
       while (!(head_done3 || torso_done3 || left_done3 || right_done3)&&(Time::now()-hf_t0<10.0))
       {
           yInfo()<<"Waiting...";
           Time::delay(0.1);   // release the quantum to avoid starving resources
           ipos_head->checkMotionDone(&head_done3);
           ipos_torso->checkMotionDone(&torso_done3);
           ipos_left->checkMotionDone(&left_done3);
           ipos_right->checkMotionDone(&right_done3);
       }

       if (head_done3 && torso_done3 &&left_done3 && right_done3)
           yInfo()<<"Movement completed";
       else
           yWarning()<<"Timeout expired";

     }


 void Shy(){
    JointConf();
    //position move
    ipos_left->positionMove(shoulder_pitch,-23.4);
    ipos_left->positionMove(shoulder_roll,30.9);
    ipos_left->positionMove(shoulder_yaw,74.04);
    ipos_left->positionMove(elbow,44.9);
    ipos_left->positionMove(wrist_grosup,-1.83);
    ipos_left->positionMove(wrist_pitch,-13.44);
    ipos_left->positionMove(wrist_yaw,20.18);
    ipos_left->positionMove(hand_finger,55.00);
    ipos_left->positionMove(thumb_oppose,11.53);
    ipos_left->positionMove(thumb_p,60.78);
    ipos_left->positionMove(thumb_d,-3.9);
    ipos_left->positionMove(index_p,1.87);
    ipos_left->positionMove(index_d,1.25);
    ipos_left->positionMove(middle_p,4.59);
    ipos_left->positionMove(middle_d,0.7);
    ipos_left->positionMove(pinky,7.68);

    ipos_right->positionMove(shoulder_pitch,-23.4);
    ipos_right->positionMove(shoulder_roll,30.9);
    ipos_right->positionMove(shoulder_yaw,74.04);
    ipos_right->positionMove(elbow,44.9);
    ipos_right->positionMove(wrist_grosup,-1.83);
    ipos_right->positionMove(wrist_pitch,-13.44);
    ipos_right->positionMove(wrist_yaw,20.18);
    ipos_right->positionMove(hand_finger,55.00);
    ipos_right->positionMove(thumb_oppose,11.53);
    ipos_right->positionMove(thumb_p,60.78);
    ipos_right->positionMove(thumb_d,-3.9);
    ipos_right->positionMove(index_p,1.87);
    ipos_right->positionMove(index_d,1.25);
    ipos_right->positionMove(middle_p,4.59);
    ipos_right->positionMove(middle_d,0.7);
    ipos_right->positionMove(pinky,7.68);

    ipos_head->positionMove(neck_pitch,-29.79);
    ipos_head->positionMove(neck_roll,-1.20);
    ipos_head->positionMove(neck_yaw,2.993);


    ipos_torso->positionMove(torso_yaw,-29.0);
    ipos_torso->positionMove(torso_roll,-1.20);
     ipos_torso->positionMove(torso_pitch,20.0);


    bool head_done1=false;
    bool torso_done1=false;
    bool left_done1=false;
    bool right_done1=false;
    double shy_t0=Time::now();
    while (!(head_done1 || torso_done1 || left_done1 || right_done1)&&(Time::now()-shy_t0<20.0))
    {
        yInfo()<<"Waiting...";
        Time::delay(0.1);   // release the quantum to avoid starving resources
        ipos_head->checkMotionDone(&head_done1);
        ipos_torso->checkMotionDone(&torso_done1);
        ipos_left->checkMotionDone(&left_done1);
        ipos_right->checkMotionDone(&right_done1);
    }

    if (head_done1 && torso_done1 &&left_done1 && right_done1)
        yInfo()<<"Movement completed";
    else
        yWarning()<<"Timeout expired";

  }

 void Happy(){
    JointConf();
    //position move
    ipos_left->positionMove(shoulder_pitch,2.54);
    ipos_left->positionMove(shoulder_roll,108.86);
    ipos_left->positionMove(shoulder_yaw,-8.34);
    ipos_left->positionMove(elbow,15.29);
    ipos_left->positionMove(wrist_grosup,6.19);
    ipos_left->positionMove(wrist_pitch,-10.54);
    ipos_left->positionMove(wrist_yaw,-3.20);
    ipos_left->positionMove(hand_finger,55.00);
    ipos_left->positionMove(thumb_oppose,11.53);
    ipos_left->positionMove(thumb_p,60.78);
    ipos_left->positionMove(thumb_d,-3.9);
    ipos_left->positionMove(index_p,1.87);
    ipos_left->positionMove(index_d,1.25);
    ipos_left->positionMove(middle_p,4.59);
    ipos_left->positionMove(middle_d,0.7);
    ipos_left->positionMove(pinky,7.68);

    ipos_right->positionMove(shoulder_pitch,-69.27);
    ipos_right->positionMove(shoulder_roll,43.15);
    ipos_right->positionMove(shoulder_yaw,57.277);
    ipos_right->positionMove(elbow,86.28);
    ipos_right->positionMove(wrist_grosup,0.433);
    ipos_right->positionMove(wrist_pitch,15.023);
    ipos_right->positionMove(wrist_yaw,11.30);
    ipos_right->positionMove(hand_finger,55.00);
    ipos_right->positionMove(thumb_oppose,11.53);
    ipos_right->positionMove(thumb_p,60.78);
    ipos_right->positionMove(thumb_d,-3.9);
    ipos_right->positionMove(index_p,1.87);
    ipos_right->positionMove(index_d,1.25);
    ipos_right->positionMove(middle_p,4.59);
    ipos_right->positionMove(middle_d,0.7);
    ipos_right->positionMove(pinky,7.68);

    ipos_head->positionMove(neck_pitch,-29.79);
    ipos_head->positionMove(neck_roll,27.00);
    ipos_head->positionMove(neck_yaw,-35.00);

    ipos_torso->positionMove(torso_pitch,10.00);
    ipos_torso->positionMove(torso_roll,-10.20);
    ipos_torso->positionMove(torso_yaw,7);

    bool head_done2=false;
    bool torso_done2=false;
    bool left_done2=false;
    bool right_done2=false;
    double h_t0=Time::now();
    while (!(head_done2 || torso_done2 || left_done2 || right_done2)&&(Time::now()-h_t0<20.0))
    {
        yInfo()<<"Waiting...";
        Time::delay(0.1);   // release the quantum to avoid starving resources
        ipos_head->checkMotionDone(&head_done2);
        ipos_torso->checkMotionDone(&torso_done2);
        ipos_left->checkMotionDone(&left_done2);
        ipos_right->checkMotionDone(&right_done2);
    }

    if (head_done2 && torso_done2 &&left_done2 && right_done2)
        yInfo()<<"Movement completed";
    else
        yWarning()<<"Timeout expired";

  }

    void home(){
        JointConf();
        ipos_left->positionMove(shoulder_pitch,-25.0);
        ipos_left->positionMove(shoulder_roll,19.0);
        ipos_left->positionMove(shoulder_yaw,0);
        ipos_left->positionMove(elbow,49.7);
        ipos_left->positionMove(wrist_grosup,0);
        ipos_left->positionMove(wrist_pitch,0);
        ipos_left->positionMove(wrist_yaw,0);
        ipos_left->positionMove(hand_finger,58.0);
        ipos_left->positionMove(thumb_oppose,20.0);
        ipos_left->positionMove(thumb_p,19.8);
        ipos_left->positionMove(thumb_d,19.8);
        ipos_left->positionMove(index_p,9.9);
        ipos_left->positionMove(index_d,10.8);
        ipos_left->positionMove(middle_p,9.9);
        ipos_left->positionMove(middle_d,10.8);
        ipos_left->positionMove(pinky,10.8);

        ipos_right->positionMove(shoulder_pitch,-25.0);
        ipos_right->positionMove(shoulder_roll,19.0);
        ipos_right->positionMove(shoulder_yaw,0);
        ipos_right->positionMove(wrist_grosup,0);
        ipos_right->positionMove(elbow,49.7);
        ipos_right->positionMove(wrist_pitch,0);
        ipos_right->positionMove(wrist_yaw,0);
        ipos_right->positionMove(hand_finger,58.0);
        ipos_right->positionMove(thumb_oppose,20.0);
        ipos_right->positionMove(thumb_p,19.8);
        ipos_right->positionMove(thumb_d,19.8);
        ipos_right->positionMove(index_p,9.9);
        ipos_right->positionMove(index_d,10.8);
        ipos_right->positionMove(middle_p,9.9);
        ipos_right->positionMove(middle_d,10.8);
        ipos_right->positionMove(pinky,10.8);

        ipos_head->positionMove(neck_pitch,0.0);
        ipos_head->positionMove(neck_roll,0.0);
        ipos_head->positionMove(neck_yaw,0.0);
        ipos_torso->positionMove(torso_pitch,0.0);
        ipos_torso->positionMove(torso_roll,0.0);
        ipos_torso->positionMove(torso_yaw,0.0);


    }
    void Close_hand()
    {
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

        // yield the actual movement
        yInfo()<<"Yielding new thumb target: "<<t_target<<" [deg]";
        ipos_right->positionMove(thumb_d,t_target);
        ipos_right->positionMove(middle_d,m_target);
        ipos_right->positionMove(pinky,p_target);

        bool done=false;
        double t0=Time::now();
        while (!done&&(Time::now()-t0<10.0))
        {
            yInfo()<<"Waiting...";
            Time::delay(0.1);   // release the quantum to avoid starving resources
            ipos_right->checkMotionDone(&done);
        }

        if (done)
            yInfo()<<"Movement completed";
        else
            yWarning()<<"Timeout expired";
    }

public:
    virtual bool configure(ResourceFinder &rf)
    {
         string robot=rf.check("robot",Value("icub")).asString();
        // open a client interface to connect to the joint controller

        //Left arm
        Property optJoint2;
        optJoint2.put("device","remote_controlboard");
        optJoint2.put("remote","/"+robot+"/left_arm");
        optJoint2.put("local","/position/left_arm");

        //Right arm
        Property optJoint3;
        optJoint3.put("device","remote_controlboard");
        optJoint3.put("remote","/"+robot+"/right_arm");
        optJoint3.put("local","/position/right_arm");

        //head
        Property optJoint4;
        optJoint4.put("device","remote_controlboard");
        optJoint4.put("remote","/"+robot+"/head");
        optJoint4.put("local","/position/head");

        //torso
        Property optJoint5;
        optJoint5.put("device","remote_controlboard");
        optJoint5.put("remote","/"+robot+"/torso");
        optJoint5.put("local","/position/torso");

/*        Property optc_l;
        optc_l.put("device","cartesiancontrollerclient");
        optc_l.put("remote","/"+robot+"/cartesianController/left_arm");
        optc_l.put("local","/cartesian_client/left_arm");

        Property optc_r;
        optc_r.put("device","cartesiancontrollerclient");
        optc_r.put("remote","/"+robot+"/cartesianController/right_arm");
        optc_r.put("local","/cartesian_client/right_arm");

        Property optc_t;
        optc_t.put("device","cartesiancontrollerclient");
        optc_t.put("remote","/"+robot+"/cartesianController/torso");
        optc_t.put("local","/cartesian_client/torso");
*/

        if (!leftArm.open(optJoint2))
        {
            yError()<<"Unable to connect to /left_arm";
            return false;
        }
        if (!rightArm.open(optJoint3))
        {
            yError()<<"Unable to connect to /right_arm";
            return false;
        }
        if (!head.open(optJoint4))
        {
            yError()<<"Unable to connect to /head";
            return false;
        }
        if (!torso.open(optJoint5))
        {
            yError()<<"Unable to connect to /torso";
            return false;
        }
 /*       if (!c_r.open(optc_r))
        {
            yError()<<"Unable to connect to /c_r";
            return false;
        }
        if (!c_l.open(optc_l))
        {
            yError()<<"Unable to connect to /c_l";
            return false;
        }

*/
        // open views
        bool ok=true;

        ok=ok && rightArm.view(tdlim);
        ok=ok && rightArm.view(mdlim);
        ok=ok && rightArm.view(pdlim);
        ok=ok && rightArm.view(tdenc);
        ok=ok && rightArm.view(mdenc);
        ok=ok && rightArm.view(pdenc);


        ok=ok && leftArm.view(imod_left);
        ok=ok && leftArm.view(ipos_left);
 //       ok=ok && c_l.view(ileft_arm);


        ok=ok && rightArm.view(imod_right);
        ok=ok && rightArm.view(ipos_right);
 //       ok=ok && c_r.view(iright_arm);

        ok=ok && head.view(imod_head);
        ok=ok && head.view(ipos_head);


        ok=ok && torso.view(imod_torso);
        ok=ok && torso.view(ipos_torso);



        if (!ok)
        {
            yError()<<"Unable to open views";
            return false;
        }

        // open rpc port
        rpc.open("/position");

        // attach the callback respond()
        attach(rpc);




  /*       yInfo()<<"getting pose";
        getpose();
         yInfo()<<"finished configuration";*/
          return true;
    }

    virtual bool close()
    {
        rpc.close();
        clienJoint.close();
        leftArm.close();
        rightArm.close();
        head.close();
        torso.close();
//        c_l.close();
//        c_r.close();

        return true;
    }

    virtual bool respond(const Bottle &cmd, Bottle &reply)
    {
        if (cmd.get(0).asString()=="Close_hand")
        {
            Close_hand();
            reply.addString("Close_hand_done");
        }
        else if (cmd.get(0).asString()=="Shy")
        {
            Shy();
            reply.addString("Shy_done");
        }
        else if (cmd.get(0).asString()=="HighFive")
        {
            HighFive();
            reply.addString("HighFive_done");
        }
        else if (cmd.get(0).asString()=="home")
        {
            home();
            reply.addString("home_done");
        }
        else if (cmd.get(0).asString()=="Happy")
        {
            Happy();
            reply.addString("Happy_done");
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

    void JointConf(){

        imod_left->setControlMode(shoulder_pitch,VOCAB_CM_POSITION);
        imod_left->setControlMode(shoulder_roll,VOCAB_CM_POSITION);
        imod_left->setControlMode(shoulder_yaw,VOCAB_CM_POSITION);
        imod_left->setControlMode(elbow,VOCAB_CM_POSITION);
        imod_left->setControlMode(wrist_grosup,VOCAB_CM_POSITION);
        imod_left->setControlMode(wrist_pitch,VOCAB_CM_POSITION);
        imod_left->setControlMode(wrist_yaw,VOCAB_CM_POSITION);
        imod_left->setControlMode(hand_finger,VOCAB_CM_POSITION);
        imod_left->setControlMode(thumb_oppose,VOCAB_CM_POSITION);
        imod_left->setControlMode(thumb_p,VOCAB_CM_POSITION);
        imod_left->setControlMode(thumb_d,VOCAB_CM_POSITION);
        imod_left->setControlMode(index_p,VOCAB_CM_POSITION);
        imod_left->setControlMode(index_d,VOCAB_CM_POSITION);
        imod_left->setControlMode(middle_p,VOCAB_CM_POSITION);
        imod_left->setControlMode(middle_d,VOCAB_CM_POSITION);
        imod_left->setControlMode(pinky,VOCAB_CM_POSITION);

        imod_right->setControlMode(shoulder_pitch,VOCAB_CM_POSITION);
        imod_right->setControlMode(shoulder_roll,VOCAB_CM_POSITION);
        imod_right->setControlMode(shoulder_yaw,VOCAB_CM_POSITION);
        imod_right->setControlMode(elbow,VOCAB_CM_POSITION);
        imod_right->setControlMode(wrist_grosup,VOCAB_CM_POSITION);
        imod_right->setControlMode(wrist_pitch,VOCAB_CM_POSITION);
        imod_right->setControlMode(wrist_yaw,VOCAB_CM_POSITION);
        imod_right->setControlMode(hand_finger,VOCAB_CM_POSITION);
        imod_right->setControlMode(thumb_oppose,VOCAB_CM_POSITION);
        imod_right->setControlMode(thumb_p,VOCAB_CM_POSITION);
        imod_right->setControlMode(thumb_d,VOCAB_CM_POSITION);
        imod_right->setControlMode(index_p,VOCAB_CM_POSITION);
        imod_right->setControlMode(index_d,VOCAB_CM_POSITION);
        imod_right->setControlMode(middle_p,VOCAB_CM_POSITION);
        imod_right->setControlMode(middle_d,VOCAB_CM_POSITION);
        imod_right->setControlMode(pinky,VOCAB_CM_POSITION);

        imod_head->setControlMode(neck_pitch,VOCAB_CM_POSITION);
        imod_head->setControlMode(neck_roll,VOCAB_CM_POSITION);
        imod_head->setControlMode(neck_yaw,VOCAB_CM_POSITION);

        imod_torso->setControlMode(torso_pitch,VOCAB_CM_POSITION);
        imod_torso->setControlMode(torso_roll,VOCAB_CM_POSITION);
        imod_torso->setControlMode(torso_yaw,VOCAB_CM_POSITION);
        //ref speed
        ipos_left->setRefSpeed(shoulder_pitch,20.0);
        ipos_left->setRefSpeed(shoulder_roll,20.0);
        ipos_left->setRefSpeed(shoulder_yaw,20.0);
        ipos_left->setRefSpeed(elbow,20.0);
        ipos_left->setRefSpeed(wrist_grosup,20.0);
        ipos_left->setRefSpeed(wrist_pitch,20.0);
        ipos_left->setRefSpeed(wrist_yaw,20.0);
        ipos_left->setRefSpeed(hand_finger,20.0);
        ipos_left->setRefSpeed(thumb_oppose,20.0);
        ipos_left->setRefSpeed(thumb_p,20.0);
        ipos_left->setRefSpeed(thumb_d,20.0);
        ipos_left->setRefSpeed(index_p,20.0);
        ipos_left->setRefSpeed(index_d,20.0);
        ipos_left->setRefSpeed(middle_p,20.0);
        ipos_left->setRefSpeed(middle_d,20.0);
        ipos_left->setRefSpeed(pinky,20.0);

        ipos_right->setRefSpeed(shoulder_pitch,20.0);
        ipos_right->setRefSpeed(shoulder_roll,20.0);
        ipos_right->setRefSpeed(shoulder_yaw,20.0);
        ipos_right->setRefSpeed(elbow,20.0);
        ipos_right->setRefSpeed(wrist_grosup,20.0);
        ipos_right->setRefSpeed(wrist_pitch,20.0);
        ipos_right->setRefSpeed(wrist_yaw,20.0);
        ipos_right->setRefSpeed(hand_finger,20.0);
        ipos_right->setRefSpeed(thumb_oppose,20.0);
        ipos_right->setRefSpeed(thumb_p,20.0);
        ipos_right->setRefSpeed(thumb_d,20.0);
        ipos_right->setRefSpeed(index_p,20.0);
        ipos_right->setRefSpeed(index_d,20.0);
        ipos_right->setRefSpeed(middle_p,20.0);
        ipos_right->setRefSpeed(middle_d,20.0);
        ipos_right->setRefSpeed(pinky,20.0);

        ipos_head->setRefSpeed(neck_pitch,20.0);
        ipos_head->setRefSpeed(neck_roll,20.0);
        ipos_head->setRefSpeed(neck_yaw,20.0);

        ipos_torso->setRefSpeed(torso_pitch,20.0);
        ipos_torso->setRefSpeed(torso_roll,20.0);
        ipos_torso->setRefSpeed(torso_yaw,20.0);

        //ref acceleration
        ipos_left->setRefAcceleration(shoulder_pitch,100.0);
        ipos_left->setRefAcceleration(shoulder_roll,100.0);
        ipos_left->setRefAcceleration(shoulder_yaw,100.0);
        ipos_left->setRefAcceleration(elbow,100.0);
        ipos_left->setRefAcceleration(wrist_grosup,100.0);
        ipos_left->setRefAcceleration(wrist_pitch,100.0);
        ipos_left->setRefAcceleration(wrist_yaw,100.0);
        ipos_left->setRefAcceleration(hand_finger,100.0);
        ipos_left->setRefAcceleration(thumb_oppose,100.0);
        ipos_left->setRefAcceleration(thumb_p,100.0);
        ipos_left->setRefAcceleration(thumb_d,100.0);
        ipos_left->setRefAcceleration(index_p,100.0);
        ipos_left->setRefAcceleration(index_d,100.0);
        ipos_left->setRefAcceleration(middle_p,100.0);
        ipos_left->setRefAcceleration(middle_d,100.0);
        ipos_left->setRefAcceleration(pinky,100.0);

        ipos_right->setRefAcceleration(shoulder_pitch,100.0);
        ipos_right->setRefAcceleration(shoulder_roll,100.0);
        ipos_right->setRefAcceleration(shoulder_yaw,100.0);
        ipos_right->setRefAcceleration(elbow,100.0);
        ipos_right->setRefAcceleration(wrist_grosup,100.0);
        ipos_right->setRefAcceleration(wrist_pitch,100.0);
        ipos_right->setRefAcceleration(wrist_yaw,100.0);
        ipos_right->setRefAcceleration(hand_finger,100.0);
        ipos_right->setRefAcceleration(thumb_oppose,100.0);
        ipos_right->setRefAcceleration(thumb_p,100.0);
        ipos_right->setRefAcceleration(thumb_d,100.0);
        ipos_right->setRefAcceleration(index_p,100.0);
        ipos_right->setRefAcceleration(index_d,100.0);
        ipos_right->setRefAcceleration(middle_p,100.0);
        ipos_right->setRefAcceleration(middle_d,100.0);
        ipos_right->setRefAcceleration(pinky,100.0);


        ipos_head->setRefAcceleration(neck_pitch,100.0);
        ipos_head->setRefAcceleration(neck_roll,100.0);
        ipos_head->setRefAcceleration(neck_yaw,100.0);

        ipos_torso->setRefAcceleration(torso_pitch,100.0);
        ipos_torso->setRefAcceleration(torso_roll,100.0);
        ipos_torso->setRefAcceleration(torso_yaw,100.0);

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
