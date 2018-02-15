

<a name='tutorials'></a>
# Tutorials

We have prepared a series of tutorials and examples on how to use yarp.js. To get things started, in this section you will find the instruction to write and run a minimalistic node server that uses yarp.js. A website will be locally served on your network, showcasing the following demo applications:

* [Simple Example](/examples/simple_example)
* [Speech Recognition and Synthesis](/examples/speech_recognition)
* [Visualization with WebGL - Point Cloud](/examples/point_cloud) 
* [Transmitting Inertial Data](/examples/inertial_data)
* [Stream Audio](/examples/stream_audio)
* [Stream Video (a yarpview with yarp.js)](/examples/stream_video) 
* [Face Tracking (and "Teleoperation"!)](/examples/face_tracker) 
* [Secure Domains (HTTPS)](/examples#secure-domains)

**Note.** Due to some recent changes in Google Chrome's privacy settings, applications that require to access the device camera/microphone need to be on a secure domain (`https` rather than `http`). Go to [Secure Domains (HTTPS)](#secure-domains) for instruction on how to run the same applications with a self-signed SSL certificate. This workaround is not (yet?) needed for Firefox.


<a name='setup'></a>
## Setup

Throughout, we will assume that a YARP server is running. If that's not your case, run `$> yarp server` on a machine where YARP is installed. Then, from the folder where you cloned yarp.js, run the examples script
```
$> node examples/examples.js
```
Open the browser on any device connected to your network and go to the address
```
your.machine.ip.address:3000 
// Please *do not use* localhost:3000 (although it works if you are on the same machine where you run the examples.js script).
```
(You can get your device ip by typing `$> ifconfig` on a shell on the machine where the examples is running). You should see a webpage like the one below

<p align='center'>
<img style='border:1px solid green; box-shadow: 0 0 10px rgba(0,0,0, .65);' src="https://github.com/robotology/yarp.js/blob/master/images/example_browser_landing.png" width="60%">
</p>

The available examples will be shown in the header.

## Server

The script `examples.js` is a minimalistic example of node server running yarp.js. We start by `require`-ing the necessary dependencies to run a HTTP server and get some basic functionalities (e.g. websockets), namely
```js
var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);
```
We load the yarp.js library by  
```js
var yarp = require('../yarp');
```
In order to handle yarp.js browser-server communication, we created a wrapper for the socket.io `io` object in yarp,js, which will manage these things for you. We setup this by
```js
yarp.browserCommunicator(io);
```


Then we set up the static folders that will be made accessible to the client. These will simply contain static data (e.g. html files, images, etc.). For more info on serving data, consult the [express.js](https://expressjs.com/) library. We set up `index.html` to be the default page served to a connecting client
```js
app.get('/', function(req, res){
  res.sendFile('index.html',{ root : __dirname});
});
```
and we finally start the server
```js
http.listen(3000, function(){
  console.log('listening on *:3000');
});
```
We are ready to go! (of course you can change the port on which the server runs).


<a name='secure-domains'></a>
## Secure Domains (HTTPS)

Recently, Google Chrome has changed its privacy settings, requiring applications that access the device microphone or camera to run from a secure origin (HTTPS). We have provided a self-signed certificate to run local applications from an HTTPS domain rather than HTTP, therefore circumventing this issue. **However we have not yet determined whether the HTTPS will introduce latencies in yarp.js data transmission**

To start the "secure" yarp.js server, simply run
```
$> node examples/examples_https.js
```
then go to
```
https://your.machine.ip.address:3000  // <--- mind the https at the beginning!!
```
**Note**. Mind 
1. the *https* in the beginning
2. you cannot use `localhost` in place of `your.machine.ip.address`

You should get a **warning**, telling you that "your connection is not private". Click on the link `Advanced` and then `Proceed to (address)`. 

You should now be able to see the usual starting page of yarp.js examples. But you are now running on HTTPS!



