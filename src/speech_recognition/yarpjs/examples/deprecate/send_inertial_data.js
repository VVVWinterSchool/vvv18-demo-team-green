// hello.js

var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);


var yarp = require('../yarp');

app.use(express.static('examples/images'));
app.use(express.static('js'));

app.get('/', function(req, res){
  res.sendFile('send_inertial_data.html',{ root : __dirname});
});

app.get('/receive', function(req, res){
  res.sendFile('receive_inertial_data.html',{ root : __dirname});
});


http.listen(3000, function(){
  console.log('listening on *:3000');
});


yarp.browserCommunicator(io);







