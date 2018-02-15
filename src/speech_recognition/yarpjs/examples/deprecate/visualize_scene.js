// hello.js

var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);

var yarp = require('../yarp');
// var yarp = require('./yarp');



app.use(express.static('node_modules/three'));
app.use(express.static('js'));


app.get('/', function(req, res){
  res.sendfile('examples/visualize_scene.html');
});


app.get('/yarp.js', function(req, res){
  res.sendfile('/js/yarp.js');
});


http.listen(3000, function(){
  console.log('listening on *:3000');
});


yarp.browserCommunicator(io);



