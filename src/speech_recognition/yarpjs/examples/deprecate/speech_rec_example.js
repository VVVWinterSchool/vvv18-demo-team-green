// hello.js

var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);


var yarp = require('../yarp');


app.use(express.static('node_modules'));


app.get('/', function(req, res){
  res.sendfile('examples/speech_rec_example.html');
  // res.sendFile('speech_rec_example.html',{ root : __dirname});
});

app.get('/yarp.js', function(req, res){
  res.sendfile('js/yarp.js');
});





http.listen(3000, function(){
  console.log('listening on *:3000');
});


yarp.browserCommunicator(io);



