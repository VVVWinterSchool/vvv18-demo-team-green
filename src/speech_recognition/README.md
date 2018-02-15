# Module yarpjs

This module is based on the module *[yarp.js](https://github.com/robotology/yarp.js)*

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

# Run the modules

In order to run the modules and interconnect them here is the procedure :

<ol>
    <li>Run the nodeJs server of YarpsJs: In yarpjs open one terminal and type: <b>node server/run_server.js</b>
        </li>
    <li>Launch userPreferenceModule. In associated folder : <b>./userPreferenceModule</b> </li>
    <li>Open chrome/chromium at : <b>http://localhost:3000</b></li>
    <li>Connect <b>/yarpjs/speech/rec:o</b> => <b>/userPreference/text_from_speech:i</b> </li>
    <li>Connect <b>/userPreference/user_label/rpc</b> => <b>classifier rpc in server port</b> </li>
    <li>Enjoy !! </li>
</ol>