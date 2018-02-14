# userPrefence module

This module compare a recovered label on [ */userPrefence/text_from_speech:i* ], to an authorised database (**user_list.json**).

If the label match the database, the label is send to the classifier trough [ */userPrefence/user_label:rpc_out* ]

*Ports*

# userPrefence module

This module compare a recovered label on [ */userPrefence/text_from_speech:i* ], to an authorised database (**user_list.json**).

If the label match the database, the label is send to the classifier trough [ */userPrefence/user_label:rpc_out* ]

*Ports*

<ul>
  <li><b>/userPrefence/text_from_speech:i</b>  Output recognised word by the STT (speak to text) engine provided by yarpjs</li>
  <li><b>/userPrefence/user_label/rpc</b>  Port where the correct user *label*(string) is send to the classifier</li>
  <li><b>/userPrefence/control/rpc:i</b>  Input port for TTS Synthesis</li>
</ul> 

*rpc command*

<ul>/userPrefence/control/rpc:i
  <li><b>"send"</b>  Set the module in 'sendind' configuration</li>
</ul> 



