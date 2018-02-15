
<a name='example-inertial-data'></a>
# Transmitting Inertial Data

<p align='center'>
<img src="https://github.com/robotology/yarp.js/blob/master/images/example_browser_inertial_data.png" width="60%">
</p>

This demo shows how information can be sent over the YARP network and visualized on the browser using different tools. In particular we will use [Google Charts](https://developers.google.com/chart/) and [Three.js](https://github.com/mrdoob/three.js/) (a wrapper for [WebGL](https://en.wikipedia.org/wiki/WebGL?oldformat=true)) to visualize the current orientation of a device streaming data over YARP.

**You will need two devices**: one equipped with inertial sensing capabilities, which will send the 3 dimensional vector containing the device orientation over the YARP network. A second device will act as a receiver, reading this information from the network and visualizing it on screen.

Follow the setup instructions [here](examples) to run the yarp.js server. Then, on both devices, go to the example page on the browser and click on the `inertial data` tab. On the device sending data, click on the `Send Data` button: you should start seeing something like this as you move the device around:

<p align='center'>
<img src="https://github.com/robotology/yarp.js/blob/master/images/visualize-data.gif" width="60%">
</p>



<a name='code-overview'></a>
## Implementation

In this section we describe how to send the device inertial information across the YARP network in practice.

### Setup

Before writing application specific code, we need to first setup the `yarp` javascript object in the browser. To do so, we will need to load the `socket.io` and `yarp.js` libraries (both served by the yarp.js server we run from [here](examples)) and then initialize the `yarp` object with the `io` websocket.
```html
<html>
  <head>
  
    <--! your head here --> 

    <script src="/socket.io/socket.io.js"></script>
    <script src="/yarp.js"></script>

  </head>
  
  <body>
    <--! your body here --> 
  </body>
  
  <script>
    var socket = io();
    yarp.init(socket);
    
    yarp.onInit( function () {
        <--! your code here -->
    });
    
  </script>

</html>
```
Mind the position where these instruction are. In particular we follow the practice of loading libraries within the head tag, while putting application-specific scripts after the body tag.

**Note.** All code related to opening and setting onRead callbacks for ports needs to be put within the initialization function `yarp.onInit`. This is needed because the yarp javascript object can start connecting with the yarp.js server only when the `io` object has been fully loaded. We plan to remove this requirement in the future. 

### Transmitting Inertial Data

**Within the `yarp.onInit` callback**, open the ports we will use for this application and connect them.
```js
var port_orientation_out = yarp.PortHandler.openPort('/yarpjs/device/orientation:o');
var port_orientation_in = yarp.PortHandler.openPort('/yarpjs/device/orientation:i');

yarp.Network.connect('/yarpjs/device/orientation:o','/yarpjs/device/orientation:i');
```
From a YARP user perspecgtive, the last instruction could seem odd, since we are connecting two ports opened withing on the same application. However, notice ports are shared across multiple browsers via the yarp.js server and in this application, the browser running on the device will use `orientation:o` port while the other browser will use the `orientation:i` port regardless of which one opened and connected them. **In a real application the best practice is to divide the two functionalities in two distinct pages, one for the device streaming inertial data out and the other for receivers.**

Let's add a callback for the port reading inertial data so that we can visualize this information within the webpage.
```js
port_orientation_in.onRead(function(bottle){
var orientation = bottle.content;
// your data visualization code here
```
then the variable `orientation` will contain the 3 cordinates containing the orientation fo the device as soon as they are read from the corresponding port.

Then, we use the [Web APIs](https://developer.mozilla.org/en-US/docs/Web/Events/deviceorientation) to get the current device orientation and then write it onto the YARP network.
```js
window.addEventListener("deviceorientation", function () {
  
  var alpha = event.alpha;
  var beta = event.beta;
  var gamma = event.gamma;

  port_orientation_out.write([alpha,beta,gamma]);
  
}, true );
```
and that's it! We are now sending and receiving device orientation over YARP!
