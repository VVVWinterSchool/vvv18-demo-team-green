

var yarp = require('./yarp');

var ynet = new yarp.Network();

var port = new yarp.BufferedPort('bottle');


port.open('/yarpjs/example');


// writing from port
var bottle = new yarp.Bottle();
bottle.fromString('This is a Bottle');

port.write(bottle);

// alternatively 
var prepared_bottle = port.prepare();
prepared_bottle.fromString('This is another Bottle');

port.write();


// reading from port
port.onRead(function(msg){
    console.log('Message received: ' + msg.toString());
});



//reading from port
port.onRPC(function(msg){
    console.log(msg.toString());

    var bottle = new yarp.Bottle();
    bottle.fromString('Reply (' + msg.toString() + ')');

    port.reply(bottle);
});




