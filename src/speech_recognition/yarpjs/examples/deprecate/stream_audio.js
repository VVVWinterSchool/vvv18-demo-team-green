
var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);



var yarp = require('../yarp');

app.use(express.static('js'));

app.get('/', function(req, res){
  res.sendFile('stream_audio.html',{ root : __dirname});
});





var port = new yarp.Port('sound');
port.open('/yarpjs/mic:o');


io.on('connection', function(socket){

  socket.on('stream-audio', function (data) {

    var sound = port.prepare();

    sound.fromBinary(data.buffer);

    console.log(sound.toBinary());

    port.write();

  });

});


http.listen(3000, function(){
  console.log('listening on *:3000');
});


yarp.browserCommunicator(io);







