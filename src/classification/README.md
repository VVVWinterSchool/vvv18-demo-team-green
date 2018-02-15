# Module Object Classification

This module is based on CNN to classify two objects on a stream of images. It receives bounded boxes of the two objects and crops the image. Then it classifies each object. When it is triggered by a rpc client, it sends positions of the object name provided by the user. 


*Input :*

<ul>
    <li><b>/objectRecognizer/img:i</b> Buffered port receiving frames from the camera</li>
  <li><b>/objectRecognizer/crops:i</b> Buffered port receiving a list of bounded boxes around the objects and their position from the segmentation module</li>
  <li><b>/objectRecognizer/userLabel/rpc:i</b> RPC port receiving the detected label as a string from the voice recognition module - send back "label_ok" or "label_invalid"</li>
  <li></li>
</ul> 

*Output :*

<ul>
  <li><b>/objectRecognizer/position:o</b> Buffered port sending the position of the object provided by the user</li>
  <li><b>/objectRecognizer/score:o</b> Port sending the scores of the classified objects for debugging</li>
  <li><b>/objectRecognizer/view:o</b> Buffered port sending the original image with the bounded boxes and the classification labels for debugging</li>
</ul> 

# Run the modules

In order to run the module separately from the other modules, here is the procedure:

<ol>
    <li>Build and install the module</li>
    <li>Run the yarpserver, yarpmanager and yarpserver --run /lh</li>
    <li>Check that the flag FLAG_UNIT_TEST to true (to test with the webcam)</li>
    <li>Run and connect everything in the yarpmanager</b></li>
    <li>Enjoy !! </li>
</ol>
