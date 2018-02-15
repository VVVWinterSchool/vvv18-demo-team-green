# Module segment_crop

This module takes as input a disparity map and outputs the bounding box of 2 brightest blobs (represent the two closest objects) and their respective Cartesian positions in the root frame.

*Ports* :
<ul>
    <li><b>/segment_crop/disparity:i</b> : Input port for receiving the disparity map from the SFM</li>
    <li><b>/segment_crop/crops:o</b> : Output port for sending the bounding boxes and the Cartesian positions of the two closest objects in a Bottle.
    <li><b>/segment_crop/valid_detection:o</b> : Ouput port for sending timout</li>
    <li><b>/segment_crop/rpcSFM</b> : RPC port used to interact with SFM to retrieve 3D positions of some single pixels</li>
    <li><b>/segment_crop/service</b> : RPC port used by a client to change the value used by the threshold</li>
</ul>

<b> Structure of the Bottle containing the bounding  boxes & positions : </b>  `Bounding_box_1` `3D_position_1` `Bounding_box_2` `3D_position_2`
