// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Cartesian Interface to control a limb
// in the operational space.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/BufferedPort.h>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlModule: public RFModule
{
    BufferedPort<Bottle> inputObjectPositionPort;
    // port for reading user feedback
    BufferedPort<Bottle> outputPort;

    PolyDriver         drvArm, drvGaze;
    ICartesianControl *arm;
    IGazeControl      *igaze;

    Vector xd;
    Vector od;

    int startup_context_id;
    int startup_context_gaze;

    double t;
    double t0;
    double t1;
    double trajectoryTime = 5.0;
    int rateThread = 1000; // 1s
    double fingerAddedLength;

    bool threadCalled = false;

public:

    // Fixate eyes
    void fixate(const Vector &x)
    {
        // FILL IN THE CODE +++
        yInfo()<<"Fixating on the 3d point: "<<x[0]<<","<<x[1]<<","<<x[2];
        //igaze->lookAtFixationPointSync(x);
        igaze->lookAtFixationPoint(x);
        yInfo()<<"Waitinf for the end of visual fixation...";
        // igaze->waitMotionDone();
        yInfo()<<"Fixation done.";
    }

    void extendIndexFingerLength(double lengthToAdd)
    {
        yInfo()<<"Index finger length extention (attachTipFrame)... ";
        yarp::sig::Vector position(3);
        // Vector in axis-angle representation
        yarp::sig::Vector orientation(4);

        position[0] = lengthToAdd; //0.1; // in meters
        position[1] = -0.02;
        position[2] = 0.0;

        // Add this length to a total cumulator
        fingerAddedLength += lengthToAdd;

        // Orietnation with respect to the original frame
        orientation[0] = 0.0;
        orientation[1] = 0.0;
        orientation[2] = 0.0;
        orientation[3] = 0.0;
        //od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;

        arm->attachTipFrame(position, orientation);
        yInfo()<<"done!";
    }

    void pointIndexFingerToTarget(yarp::sig::Vector position)
    {
        yInfo()<<"Pointing finger to object... ";
        //arm->goToPositionSync(position);
        arm->goToPosition(position);

        bool motion_done;
        arm->checkMotionDone(&motion_done);
        double startTime = Time::now();
        while ((!motion_done) && (Time::now() - startTime < trajectoryTime))
        {
            yInfo()<<"Waiting for motion to finish...";
            Time::delay(1.0);
            arm->checkMotionDone(&motion_done);
        }

//        if (!motion_done)
//        {
//            arm->stopControl();
//        }
        yInfo()<<"done!";
    }

    yarp::sig::Vector getShoulderPosition()
    {
        int linkID = 4; // or 5
        yarp::sig::Vector x, o;
        arm->getPose(linkID, x, o);
        yInfo()<<"Shoulder position: "<<x.toString();
        // should be around 0, 20,40
        return x;
    }

    // Get closest reachable pose to object (on the sphere shell which is the limit of reachability space)
    yarp::sig::Vector getClosestReachablePoseToObject(yarp::sig::Vector objectPosition,
                                                       yarp::sig::Vector shoulderPosition,
                                                       double armLength)
    {
        yarp::sig::Vector closestPointPosition(3);
        yarp::sig::Vector closestPointOrientation(3);

        yarp::sig::Vector lineSlant(3);
        yarp::sig::Vector lineOffsets(3); // this is shoulder position

        lineSlant[0] = shoulderPosition[0] - objectPosition[0];
        lineSlant[1] = shoulderPosition[1] - objectPosition[1];
        lineSlant[2] = shoulderPosition[2] - objectPosition[2];

        // Compute the line offsets
        // it's the shoulder position

        // Compute the intersection of the line with the sphre centered on the shoulder
        closestPointPosition[0] = armLength * lineSlant[0] + shoulderPosition[0];
        closestPointPosition[1] = armLength * lineSlant[1] + shoulderPosition[1];
        closestPointPosition[2] = armLength * lineSlant[2] + shoulderPosition[2];

        closestPointOrientation[0] = lineSlant[0];
        closestPointOrientation[1] = lineSlant[1];
        closestPointOrientation[2] = lineSlant[2];

        yarp::sig::Vector closestReachablePose(6);

        closestReachablePose[0] = closestPointPosition[0];
        closestReachablePose[0] = closestPointPosition[1];
        closestReachablePose[0] = closestPointPosition[2];

        closestReachablePose[0] = closestPointOrientation[0];
        closestReachablePose[0] = closestPointOrientation[1];
        closestReachablePose[0] = closestPointOrientation[2];

        return closestPointPosition;
    }

    double getArmLength()
    {
        return 0.4;
    }


    double getInterPointDistance(yarp::sig::Vector p1, yarp::sig::Vector p2)
    {
        return std::sqrt(
                    std::pow(p1[0] - p2[0], 2) +
                    std::pow(p1[1] - p2[1], 2) +
                    std::pow(p1[2] - p2[2], 2));
    }


//    void printStatus()
//    {
//        if (t-t1>=PRINT_STATUS_PER)
//        {
//            Vector x,o,xdhat,odhat,qdhat;

//            // we get the current arm pose in the
//            // operational space
//            if (!arm->getPose(x,o))
//                return;

//            // we get the final destination of the arm
//            // as found by the solver: it differs a bit
//            // from the desired pose according to the tolerances
//            if (!arm->getDesired(xdhat,odhat,qdhat))
//                return;

//            double e_x=norm(xdhat-x);
//            double e_o=norm(odhat-o);

//            yInfo()<<"+++++++++";
//            yInfo()<<"xd          [m] = "<<xd.toString();
//            yInfo()<<"xdhat       [m] = "<<xdhat.toString();
//            yInfo()<<"x           [m] = "<<x.toString();
//            yInfo()<<"od        [rad] = "<<od.toString();
//            yInfo()<<"odhat     [rad] = "<<odhat.toString();
//            yInfo()<<"o         [rad] = "<<o.toString();
//            yInfo()<<"norm(e_x)   [m] = "<<e_x;
//            yInfo()<<"norm(e_o) [rad] = "<<e_o;
//            yInfo()<<"---------";

//            t1=t;
//        }
//    }


    yarp::sig::Vector generateTarget()
    {
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
        // and frequency 0.1 Hz

        //xd[0] = std::min((std::rand() * MAX_DIST), MIN_DIST);
         xd[0]=-2.3 + (std::rand()% 2);
         xd[1]=-1.3 + (std::rand()% 10)*0.17; //0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
         xd[2]=+0.7 + (std::rand()%10)*0.12; //0.1; //0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));

        yInfo()<<"New target: "<<xd[0]<<", "<<xd[1]<<", "<<xd[2];

        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        //od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=0; //M_PI;
        return xd;
    }

    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    bool configPorts()
    {
        // open the input port
        if (!inputObjectPositionPort.open("/objectPointing/position:i"))
        {
            yError()<<"error opening input port for client";
            return false;
        }

        // open the output port
        if (!outputPort.open("/objectPointing/done:o"))
        {
            yError()<<"error opening output port for sending object pointing done signal";
            return false;
        }

        return true;
    }


    virtual bool configure(ResourceFinder &rf)
    {
        // retrieve command line options
        //double period=rf.check("period",Value(CTRL_THREAD_PER)).asDouble();

        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch: iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch: yarprobotinterface --context simCartesianControl)
        //
        // 3 - the cartesian solver for the left arm is running too
        //     (launch: iKinCartesianSolver --context simCartesianControl --part left_arm)
        //
        Property option;
        option.put("device","cartesiancontrollerclient");
        option.put("remote","/icubSim/cartesianController/right_arm");
        option.put("local","/cartesian_client/right_arm");
        yDebug()<<"cartesian control property OK";

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/tracker/gaze");
        yInfo()<<"optGaze declared";

        yInfo()<<"Configuring ports...";
        if (!configPorts())
        {
            yError()<<"Failed to configure ports";
            return false;
        }

        // let's give the controller some time to warm up
        if (drvArm.open(option))
        {
            yInfo()<<"Successfully opened Cartesian controller!";
        }
        else
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        if (drvGaze.open(optGaze))
        {
            yInfo()<<"Successfully opened Gaze controller!";
        }
        else
        {
            yInfo()<<"Failed to open Cartesian controller!";
            return false;
        }

        yInfo()<<"Opening the view";
        // open the view
        drvArm.view(arm);
        drvGaze.view(igaze);


        yInfo()<<"Storing the context";
        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        arm->storeContext(&startup_context_id);
        igaze->storeContext(&startup_context_gaze);


        // set trajectory time
        yDebug()<<"Setting gaze tracking mode true";
        igaze->setTrackingMode(true);
        yDebug()<<"Setting neck and eyes trajectory time";
        igaze->setNeckTrajTime(0.6);
        igaze->setEyesTrajTime(0.4);

        yInfo()<<"Setting trajectory time";
        // set trajectory time
        arm->setTrajTime(trajectoryTime);
        yDebug()<<"TrajectoryTime: "<<trajectoryTime;
        yDebug()<<__LINE__;

        yInfo()<<"Getting new torso dofs";
        // get the torso dofs
        Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;
        newDof[1]=0; // roll
        newDof[2]=1;


        yInfo()<<"Set new torso dofs";
        yDebug()<<__LINE__;
        // send the request for dofs reconfiguration
        arm->setDOF(newDof,curDof);


        yDebug()<<"Limiting torso pitch...";
        yDebug()<<__LINE__;
        // impose some restriction on the torso pitch
        limitTorsoPitch();

        xd.resize(3);
        od.resize(4);

        fingerAddedLength += 0.1;
        extendIndexFingerLength(fingerAddedLength);
        yDebug()<<"Total added finger length: "<<fingerAddedLength;

        yInfo()<<"Thread started successfully";
        t=t0=t1=Time::now();

        return true;
    }

    /****************************************************/
    bool closePorts()
    {
        inputObjectPositionPort.close();
        outputPort.close();
        // also close other ports of Abbas
    }

    virtual bool close()
    {
        yInfo()<<"Closing... ";
        // we require an immediate stop
        // before closing the client for safety reason
        arm->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        arm->restoreContext(startup_context_id);

        closePorts();

        drvArm.close();
        drvGaze.close();

        yInfo()<<"done!";
        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        yInfo()<<"UpdateModule is called!";
        if (!threadCalled)
        {
            yDebug()<<"Doing the task...";
            yarp::sig::Vector targetObjectPosition(3);

            yDebug()<<"Reading from the port";
            Bottle* bottle = NULL;
            bottle = inputObjectPositionPort.read(); // blocking read
            if (bottle==NULL)
            {
                yError()<<"No bottle received through input object position port.";
                return false;
            }
            else if (bottle!=NULL)
            {
                if (bottle->size() != 3)
                {
                    yError()<<"Received not 3 coordinates (but something else)";
                }
                yInfo()<<"Receiving the bottle. Coordinates OK";
                targetObjectPosition[0] = bottle->get(0).asDouble();
                targetObjectPosition[1] = bottle->get(1).asDouble();
                targetObjectPosition[2] = bottle->get(2).asDouble();
            }
            yDebug()<<"Done reading from the port";

            yInfo()<<"Pointing operations start!";
            threadCalled = true;

//            t=Time::now();
            // Generate a new target every (N) milliseconds
            // targetObjectPosition = generateTarget(); // fills GD

//            yarp::sig::Vector shoulderPosition = getShoulderPosition();
//            double distanceFromShoulderToObject = getInterPointDistance(shoulderPosition, targetObjectPosition);
//            double armLength = getArmLength();

//            // Tells if a 3d position is reachable by the robot's arm
//            bool objectIsInsideReachableSphere = (distanceFromShoulderToObject < armLength);


//            if (true) //(objectIsInsideReachableSphere)// (true)
//            {
//                yInfo()<<"Point is inside reachable sphere";
//                fingerAddedLength += 0.1;
//                extendIndexFingerLength(fingerAddedLength);

                yDebug()<<"Fixating eyes";
                fixate(targetObjectPosition);
                yInfo()<<"Pointing";
                pointIndexFingerToTarget(targetObjectPosition);
//                extendIndexFingerLength(-fingerAddedLength);
//                fingerAddedLength -= fingerAddedLength;
//                yDebug()<<"Total added finger length: "<<fingerAddedLength;

                yDebug()<<"Writing in the output port that the action is done";
                Bottle &message=outputPort.prepare();
                message.clear(); //important, objects get recycled
                message.addInt(1); // true
                outputPort.write();
                yDebug()<<"Done";

                threadCalled = false;

                // threadCalled = false;
//            }
//            else
//            {
//                yInfo()<<"Point is outside reachable sphere";

//                // Pose = 6d
//                yarp::sig::Vector closestReachablePose = getClosestReachablePoseToObject(
//                                                                    targetObjectPosition,
//                                                                    shoulderPosition,
//                                                                    armLength);

//                yarp::sig::Vector position(3);
//                yarp::sig::Vector orientation(3);
//                position[0] = closestReachablePose[0];
//                position[1] = closestReachablePose[1];
//                position[2] = closestReachablePose[2];

//                orientation[0] = closestReachablePose[3];
//                orientation[1] = closestReachablePose[4];
//                orientation[2] = closestReachablePose[5];

//                arm->goToPoseSync(position, orientation);

//                bool motion_done;
//                arm->checkMotionDone(&motion_done);
//                while (!motion_done)
//                {
//                    yInfo()<<"Waiting for motion to finish...";
//                    // some verbosity
//                    //printStatus();
//                    Time::delay(1.0);
//                    arm->checkMotionDone(&motion_done);
//                }

//                yInfo()<<"done!";
//            }
            yInfo()<<"Pointing done!";
        }

        return true;
    }


    bool interrupt()
    {
        inputObjectPositionPort.interrupt();
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

    CtrlModule mod;
    ResourceFinder rf;
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
