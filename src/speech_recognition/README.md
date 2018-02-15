# Module yarpjs

This module is based on the module *[yarp.js](https://github.com/robotology/yarp.js)*

In order to run properly: <span style="color:red">node.js need to be installed</span> ( [installation instruction](https://github.com/vvv-school/vvv18-demo-team-green/tree/master/src/speech_recognition/yarpjs)).


This mode run locally a node.js server and display a webpage [http://localhost:3000](http://localhost:3000)

*Ports*

<ul>
  <li><b>/yarpjs/speech/rec:o</b>  Output recognised word by the STT (speak to text) engine provided by yarpjs</li>
  <li><b>/yarpjs/speech/tts:i</b>  Input port for TTS Synthesis</li>
</ul> 


# userPreference module

This module compare a recovered label on [ */userPreference/text_from_speech:i* ], to an authorised database (**user_list.json**).

If the label match the database, the label is send to the classifier trough [ */userPreference/user_label:rpc_out* ]

*Ports*

<ul>
  <li><b>/userPreference/text_from_speech:i</b>  Output recognised word by the STT (speak to text) engine provided by yarpjs</li>
  <li><b>/userPreference/user_label/rpc</b>  Port where the correct user *label*(string) is send to the classifier</li>
  <li><b>/userPreference/control/rpc:i</b>  Input port for TTS Synthesis</li>
</ul> 

# Installation

**From the main folder (speech_recognition):**

```
mkdir build;
cd build
cmake ..;
make;
make install
```
**In yarpjs**

```
cd yarpjs;
npn install;
./node_modules/cmake-js/bin/cmake-js;
```
*No need of a make , cmake-js is doing the job*

**Set speech_recognition.xml file**

In the file update this line:  *(my_folder_path)*/vvv18-demo-team-green/src/speech_recognition/build/yarpjs
  

# Run the modules

In order to run the modules and interconnect them here is the procedure :

<ol>
    <li> Lauch yarpserver <li>
    <li>Run the nodeJs server of YarpsJs: In yarpjs open one terminal and type: <b>node server/run_server.js</b></li>
    <li>Launch userPreferenceModule. In associated folder : <b>./userPreferenceModule</b> </li>
    <li>Open chrome/chromium at : <b>http://localhost:3000</b></li>
    <li>Connect <b>/yarpjs/speech/rec:o</b> => <b>/userPreference/text_from_speech:i</b> </li>
    <li>Connect <b>/userPreference/user_label/rpc:o</b> => <b>/objectRecognizer/userLabel/rpc:i</b> (from classifer module) </li>
    <li>Enjoy !! </li>
</ol>
