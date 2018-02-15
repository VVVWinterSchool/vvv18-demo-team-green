# Basic Read/Write Example (Client Side)

This example shows how to send and receive messages with yarp.js. It opens two ports: **/yarpjs/simple_example:o** and **/yarpjs/simple_example:i**. 

Whenever a message is written from the yarp network to **/yarpjs/simple_example:i**, the message is printed on screen.

Whenever a message is written in the text box above (and the user sends it), the corresponding message is sent to any port on the yarp network reading connected to **/yarpjs/simple_example:o**
