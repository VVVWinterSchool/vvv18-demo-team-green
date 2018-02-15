#include <iostream>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

class userPreference:public RFModule
{
    RpcServer control_rpc_in; //A port to handle messages from the controller
    std::string name_control_rpc_in = "/userPreference/control/rpc:i";

    RpcClient label_rpc_out; // Port sending data to the clasifier
    std::string name_label_rpc_out = "/userPreference/user_label/rpc:o";

    BufferedPort<Bottle> text_from_speech; // Label from yarpjs Speak to text module //@Todo : is a bufferedPort better?
    std::string name_text_from_speech = "/userPreference/text_from_speech:i";

    std::vector<std::string> list_label{"ball", "car", "mug", "cube"};

    bool readytoSend = false; //Define if userPreference is able to send data

    double period=1.0; //default value
    std::string classifier_rpc_name = "/objectRecognizer/userLabel/rpc:i"; //default value

    float max_timeout_rpc_reply = 5.0f; // maximal waiting time for a timeout reply
    int nbTRial = 3; // nb of trial before trigering
    
public:

    double getPeriod()
    {
        return period; //module periodicity (seconds)
    }

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule()
    {

        Bottle *input = text_from_speech.read();
        if (input != NULL)
        {
            std::string str = input->get(0).asString();
            if(str.compare("")!=0 )
            {
                yInfo()<<"input from STT :"<<str;
                isPreference(str);
            }
            else
            {
                yInfo()<<"no text";
            }
        }
        else
        {
           yInfo()<<"input from STT : is NULL";
           return false;
        }
        return true;
    }

    /*
    * Message handler. Just echo all received messages.
    */
    bool respond(const Bottle& command, Bottle& reply)
    {
        yInfo()<<"Responding to command rpc";
        //just echo back the command
        std::string str = command.get(0).asString();
        cout<<"  get rpc : "<<str<<endl;
        if ((str=="Send" || str=="send"))
        {
           //return false;
            readytoSend = true;
            reply.addString("Activate userPreference reading");
        }
        else if((str =="Stop" || str =="stop"))
        {
            readytoSend = true;
            reply.addString("Desactivate userPreference reading");
        }
        else
        {
            reply.addString("not an existing command");
        }
        return true;
    }

    /* Check if word in inside the list and send */
    bool checkIfExist(std::string str)
    {

//        if (std::find(std::begin(list_label), std::end(list_label), *str) != std::end(list_label))
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }

        for (int i =0; i<list_label.size(); i++)
        {
            yInfo()<<"--exist : "<< list_label[i] <<" -- "<<str;
            if(list_label[i].compare(str)==0)
            {
                return true;
            }
        }
        return false;
    }

    bool isPreference(std::string str)
    {
        if(!checkIfExist(str))
            return false;

        //@todo : do the check

        Bottle cmd;
        Bottle response;
        cmd.addString(str);
        string rps;
        int i=1;
        for(i;i<=nbTRial;i++)
        {

            if(label_rpc_out.write(cmd,response) == true)
            {
                rps = response.toString().c_str();
                yInfo()<<"rpc answer :"<<rps;
                break; //get out of the condition
            }
            else
            {
                yInfo()<<"Fail to call : trial "<<i;
                if(i==nbTRial)
                {
                    yInfo()<<"The classifier did not answered";
                    // @todo trigger event to controller
                    return false;
                }
                Time::delay(1);
            }
         }

        // Redirection case based on the classifier feedback

        if(rps.compare("label_ok")==0)
        {
            yInfo()<<"The label ("<< str <<") was received by the classifier :";
            readytoSend = false;
        }
        else if (rps.compare("label_invalid")==0)
        {
            yInfo()<<"Unkown label ("<< str <<") for the classifier";
            //@todo : Trigger beahaviour : Reask for question
         }
        else
        {
            yInfo()<<"Unkown response -- clasifier returned :"<< rps;
        }

        return true;


        //label_rpc_out.(cmd,response);
        //readytoSend = false;
    }

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */

    bool configure(yarp::os::ResourceFinder &rf)
    {

        if(!control_rpc_in.open(name_control_rpc_in))
            {
                yError()<<"Unable to open : "<< name_control_rpc_in;
                return false;
            }
        attach(control_rpc_in);

        if(!label_rpc_out.open(name_label_rpc_out))
        {
            yError()<<"Unable to open : "<< name_label_rpc_out;
            return false;
        }
        label_rpc_out.asPort().setTimeout(max_timeout_rpc_reply);// set the maximum timeout to timeout_rpc_reply

        if(!text_from_speech.open(name_text_from_speech))
        {
            yError()<<"Unable to open : "<< name_text_from_speech;
            return false;
        }

        //user ressource finder to parse parameter --period --rpc_classifier_server

        if (rf.check("rpc_classifier_server"))
            classifier_rpc_name=rf.find("rpc_classifier_server").asString();
        
        if (rf.check("period"))
            period=rf.find("period").asDouble();


//        // @Todo : Strore here the alowed label from the file
//        if(!readJsonFile())
//        {
//            yError()<<"Unable to open JSONfile : "<< name_control_rpc_in;
//            return false;
//        }

        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        yInfo()<<"Interrupting your module, for port cleanup";

        control_rpc_in.interrupt();
        yInfo()<<"1";
        label_rpc_out.interrupt();
        yInfo()<<"2";
        text_from_speech.interrupt();
        return true;
    }

    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        yInfo()<<"Calling close function";
        control_rpc_in.close();
        label_rpc_out.close();
        text_from_speech.close();

        yInfo()<<"Closing userPreference";

        return true;
    }

private:

//    bool readJsonFile()
//    {
//        // @Todo
//        return true;
//    }

};

int main(int argc, char * argv[])
{
    Network yarp;

    userPreference userPref;
    ResourceFinder rf;
    rf.configure(argc, argv);
    // rf.setVerbose(true);

    yInfo()<<"Configure module userPreference";
    userPref.configure(rf);
    yInfo()<<"Start module userPreference";
    userPref.runModule();

    yInfo()<<"Main returning userPreference";
    return 0;
}


