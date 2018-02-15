
<a name='example-face-tracking'></a>
# Face Tracker 

<p align='center'>
<img src="https://github.com/robotology/yarp.js/blob/master/images/example_browser_face_tracking.png" width="60%">
</p>

This examples uses a face tracking routine from <a href='https://trackingjs.com/'>tracking.js</a> to  tele-operate the head of a robot (e.g. iCub or iCub simulator) via the <a href='http://wiki.icub.org/brain/group__iKinGazeCtrl.html'>iKinGazeCtrl</a>. The position of the user face is obtained from the device camera and sent as a 3D position to the iKinGaze controller. Images acquired from the robot camera can also be streamed by connecting to the corresponding port (default /icubSim/cam/left).

**Limited Support**: for security reasons, Chrome does not allow to access the audio/video stream from unsecure hosts (https Vs http). To bypass this issue you can use a **self-signed SSL** secure domain (see [here](https://github.com/robotology/yarp.js/tree/master/examples#secure-domains))!
