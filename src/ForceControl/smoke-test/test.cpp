/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <cmath>
#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Core>

#include <yarp/rtf/TestCase.h>
#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/ITorqueControl.h>

using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

struct JointError{
    unsigned index;
    double value;
    double expected;
    double error;
};

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

std::vector<std::string> getiCubJointsControlledInTorque()
{
    std::vector<std::string> axesList;

    axesList.push_back("torso_pitch");
    axesList.push_back("torso_roll");
    axesList.push_back("torso_yaw");

    // Left arm
    axesList.push_back("l_shoulder_pitch");
    axesList.push_back("l_shoulder_roll");
    axesList.push_back("l_shoulder_yaw");
    axesList.push_back("l_elbow");

    // Right arm
    axesList.push_back("r_shoulder_pitch");
    axesList.push_back("r_shoulder_roll");
    axesList.push_back("r_shoulder_yaw");
    axesList.push_back("r_elbow");

    // Left leg
    axesList.push_back("l_hip_pitch");
    axesList.push_back("l_hip_roll");
    axesList.push_back("l_hip_yaw");
    axesList.push_back("l_knee");
    axesList.push_back("l_ankle_pitch");
    axesList.push_back("l_ankle_roll");

    // Right leg
    axesList.push_back("r_hip_pitch");
    axesList.push_back("r_hip_roll");
    axesList.push_back("r_hip_yaw");
    axesList.push_back("r_knee");
    axesList.push_back("r_ankle_pitch");
    axesList.push_back("r_ankle_roll");

    return axesList;
}

/**********************************************************************/
class TestAssignmentComputedTorque : public yarp::rtf::TestCase
{
    BufferedPort<Vector> portReference;
    Vector referencesInRad;

    yarp::dev::PolyDriver robotDevice;

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IControlLimits2   *ilim{nullptr};
    yarp::dev::IEncoders         *ienc{nullptr};
    yarp::dev::IControlMode2     *imod{nullptr};
    yarp::dev::ITorqueControl    *itrq{nullptr};

    void convertDegToRad(const yarp::sig::Vector& vecDeg, yarp::sig::Vector& vecRad)
    {
        if (vecDeg.size() != vecRad.size()) {
            RTF_ASSERT_FAIL("convertDegToRad: wrong vector size");
            return;
        }

        for (size_t i=0; i < vecDeg.size(); i++) {
            vecRad[i] = (M_PI/180.0)*vecDeg[i];
        }
    }


public:
    /******************************************************************/
    TestAssignmentComputedTorque() :
        yarp::rtf::TestCase("TestAssignmentComputedTorque")
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentComputedTorque()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        ResourceFinder rf = ResourceFinder::getResourceFinderSingleton();

        //retrieve the joint list
        std::vector<std::string> iCubMainJoints = getiCubJointsControlledInTorque();

        unsigned actuatedDOFs = iCubMainJoints.size();
        RTF_TEST_REPORT(Asserter::format("DOFS: %d", actuatedDOFs));

        // Open the remotecontrolboardremapper
        Property options;
        options.put("device", "remotecontrolboardremapper");

        addVectorOfStringToProperty(options, "axesNames", iCubMainJoints);

        Bottle remoteControlBoards;
        Bottle & remoteControlBoardsList = remoteControlBoards.addList();

        std::string robotPortPrefix = rf.check("robot", yarp::os::Value("icubSim"), "Port prefix used for the controlboards").asString();
        remoteControlBoardsList.addString("/"+robotPortPrefix+"/torso");
        remoteControlBoardsList.addString("/"+robotPortPrefix+"/left_arm");
        remoteControlBoardsList.addString("/"+robotPortPrefix+"/right_arm");
        remoteControlBoardsList.addString("/"+robotPortPrefix+"/left_leg");
        remoteControlBoardsList.addString("/"+robotPortPrefix+"/right_leg");

        options.put("remoteControlBoards",remoteControlBoards.get(0));
        options.put("localPortPrefix","/TestAssignmentComputedTorque");
        Property & remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteControlBoardsOpts.put("writeStrict","on");

        bool ok = robotDevice.open(options);
        if (!ok) {
            RTF_ASSERT_FAIL("Could not create remotecontrolboardremapper object.");
            return false;
        }

        // Try to obtain the interfaces
        ok=ok && robotDevice.view(ilim);
        ok=ok && robotDevice.view(ienc);
        ok=ok && robotDevice.view(imod);
        ok=ok && robotDevice.view(itrq);

        if (!ok) {
            RTF_ASSERT_FAIL("Could not create remotecontrolboardremapper object.");
            return false;
        }

        referencesInRad.resize(actuatedDOFs, 0.0);
        yarp::sig::Vector referencesInDeg(referencesInRad.size(), 0.0);
        
        if (!portReference.open("/TestAssignmentComputedTorque/qDes:o")) {
            RTF_ASSERT_FAIL("Could not open reference port.");
            return false;
        }

        // get initial configuration
        // Make sure that we are reading data from the robot before proceeding
        bool readEncoderSuccess = false;
        for (int i=0; i < 10 && !readEncoderSuccess; i++) {
            readEncoderSuccess = ienc->getEncoders(referencesInDeg.data());
            if (!readEncoderSuccess) {
                yarp::os::Time::delay(0.1);
            }
        }

        if (!readEncoderSuccess) {
            RTF_ASSERT_FAIL("Unable to read encoders, exiting.");
            return false;
        }

        convertDegToRad(referencesInDeg, referencesInRad);

        // as refernce sum 40 degs to the first joint of the torso
        referencesInRad(0) += ((40.0 * M_PI) / 180.0);

        RTF_ASSERT_FAIL_IF_FALSE(Network::connect("/TestAssignmentComputedTorque/qDes:o", "/computed-torque/qDes:i"), "Failed to connect ports");
        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        RTF_TEST_REPORT("Closing Ports");
        portReference.close();
        robotDevice.close();
    }

    /******************************************************************/

    /******************************************************************/
    virtual void run()
    {
        RTF_ASSERT_FAIL_IF_FALSE(robotDevice.isValid(), "remotecontrolboardremapper is not valid. Test internal failure");

        Time::delay(5.0);
        // sending references to the port
        Vector& outReference = portReference.prepare();
        outReference = referencesInRad;
        portReference.write();

        RTF_TEST_REPORT("Waiting the controller to adapt to the new reference");
        Time::delay(10.0);

        Vector currentConfigurationVectorInDeg(referencesInRad.size(), 0.0);
        Vector currentConfigurationVectorInRad(referencesInRad.size(), 0.0);

        RTF_ASSERT_ERROR_IF_FALSE(ienc->getEncoders(currentConfigurationVectorInDeg.data()), "Failed to retrieve robot configuration");
        convertDegToRad(currentConfigurationVectorInDeg, currentConfigurationVectorInRad);

        double maxJointError = 5.0 * M_PI / 180.0;

        using namespace Eigen;
        Map<VectorXd> currentConfiguration(currentConfigurationVectorInRad.data(), currentConfigurationVectorInRad.size());
        Map<VectorXd> referenceConfiguration(referencesInRad.data(), referencesInRad.size());

        VectorXd jointsError = (currentConfiguration - referenceConfiguration).cwiseAbs(); //.array().abs();

        std::vector<JointError> errors;

        for (unsigned i = 0; i < jointsError.size(); ++i) {
            if (jointsError(i) > maxJointError) {
                JointError error;
                error.index = i;
                error.value = currentConfiguration(i);
                error.expected = referenceConfiguration(i);
                error.error = jointsError(i);
                errors.push_back(error);
            }
        }

        for (std::vector<JointError>::const_iterator it = errors.begin();
            it != errors.end(); ++it) {
                RTF_TEST_REPORT(Asserter::format("Joint[%d]. Expected %lf - Actual %lf [rad]",
                                                 it->index, it->expected, it->value));
            }

        RTF_ASSERT_ERROR_IF_FALSE(errors.empty(), "Error in tracking reference");

    }
};

PREPARE_PLUGIN(TestAssignmentComputedTorque)
