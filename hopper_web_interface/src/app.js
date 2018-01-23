#!/usr/bin/env node

var express = require('express'); 
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var rosnodejs = require('rosnodejs')

app.use(express.static(__dirname + '/src'));
app.use(express.static(__dirname + '/pages'));
app.use(express.static(__dirname + '/public'));
app.use('/vue', express.static(__dirname + '/node_modules/vue/dist'));
app.use('/nipplejs', express.static(__dirname + '/node_modules/nipplejs/dist'));

app.get('/', function(req, res){
    res.sendFile(__dirname + "/index.html");
});
var publisher;

rosnodejs.initNode('node_server')
.then((node) => {
   publisher = node.advertise('quadruped_command', 'geometry_msgs/Twist');
});
function sendCommandToRobot(x, y){
    console.log(`Quadruped moving X:${x.toFixed(2)} Y:${y.toFixed(2)}`);
    publisher.publish({linear: {x: x, y: y, z:0}, angular: {x:0, y:0, z:0}});
}

io.on('connection', function(socket){
    console.log('user connected!');
    socket.on('joystickUpdate', function(msg){
        if (msg.MessageType === 'movement'){
            if (msg.joystickType === 'direction'){
                const size = Math.abs(msg.distance / 50);
                if (size < 0.2){
                    sendCommandToRobot(0, 0);
                    return;
                }
                let y = -Math.cos(msg.angle);
                let x = Math.sin(msg.angle);
                const vectorLength = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                x = (x / vectorLength) * (size * 6);
                y = (y / vectorLength) * (size * 6);
                sendCommandToRobot(x, y);
            }
        }
    });
    socket.on('disconnected', function(){
        console.log('user disconnected');
    });
});

http.listen(3000, function(){
    console.log('Listening');
});

