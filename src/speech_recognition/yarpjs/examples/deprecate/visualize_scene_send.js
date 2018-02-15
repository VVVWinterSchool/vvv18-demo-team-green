
var yarp = require('../yarp');

var fs = require('fs');

var data = JSON.parse(fs.readFileSync('examples/pcloud.txt', 'utf8'));

var port = new yarp.Port('bottle');

port.open('/yarpjs/3Dpoints:o');

yarp.Network.connect('/yarpjs/3Dpoints:o','/yarpjs/3Dpoints:i');



var curr_idx = 0;

setTimeout(function sendData() {

    // if it finished looking at data, close
    if( curr_idx >= data.length)
        return;

    // console.log('writing '+[ data[curr_idx], data[curr_idx+1], data[curr_idx+2] ]);
    port.write([ data[curr_idx], data[curr_idx+1], data[curr_idx+2], 0, 0, 0, 5] );

    curr_idx+=3;

    setTimeout(sendData,3);

},500);



