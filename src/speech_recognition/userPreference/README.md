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

**Ports**

<ul>
  <li><b>/userPreference/text_from_speech:i</b>  Output recognised word by the STT (speak to text) engine provided by yarpjs</li>
  <li><b>/userPreference/user_label/rpc</b>  Port where the correct user *label*(string) is send to the classifier</li>
  <li><b>/userPreference/control/rpc:i</b>  Input port for TTS Synthesis</li>
</ul> 

**rpc command feedback**

from */userPreference/control/rpc:i*
<ul>
  <li><b>"send"</b>  Set the module in 'sendind' configuration</li>
  <li><b>"stop"</b>  Set the module in 'idle' configuration</li>
</ul> 

from */objectRecognizer/userLabel/rpc:i*
<ul>
  <li><b>"label_ok"</b>  </li>
  <li><b>"label_invalid"</b>  </li>
</ul> 



