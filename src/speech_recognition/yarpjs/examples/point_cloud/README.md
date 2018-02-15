# Visualization with WebGL (and Three.js) - Point Cloud

<p align='center'>
<img src="https://github.com/robotology/yarp.js/blob/master/images/example_browser_point_cloud.png" width="60%">
</p>

This example shows how to visualize 3D point clouds with with yarp.js using the [Three.js](https://github.com/mrdoob/three.js/) library (a wrapper for [WebGL](https://en.wikipedia.org/wiki/WebGL?oldformat=true)). (Demo suggested by [Pattacini](https://github.com/pattacini) in issue [#3](https://github.com/robotology/yarp.js/issues/3)). 

To set up the demo, follow the setup instructions in [here](examples).

The application opens a port `/yarpjs/3Dpoints:i` where 3D points can be sent and visualized in the browser. You can write to this port by running the command 
```
$> yarp write ... /yarpjs/3Dpoints:i
```
on a shell. You can then add new points to the scene visualized in the browser by sending a bottle with format
```
>> (x y z R G B size)
```
where `x y z` are the cartesian coordinates of the point, the R G B colour defaults to black, namely `(0 0 0)` and the point size defaults to `5` pixels. 
You can send multiple points at the same time by concatenating bottles (e.g. `(b1) (b2) ... (bn)`). Try it out!


#### Example of dynamic creation of point cloud

Run the shell command `$> node examples/point_cloud/send_point_cloud.js` from the directory where you cloned yarp.js. (Thanks to [Seanfa](https://github.com/seanfa) for the point cloud data).


