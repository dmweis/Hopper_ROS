#!/usr/bin/env node

const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const robot = require('./quadruped_controller.js');

app.use(express.static(__dirname + '/web_app'));
app.use(express.static(__dirname + '/pages'));
app.use(express.static(__dirname + '/public'));
app.use('/vue', express.static(__dirname + '/node_modules/vue/dist'));
app.use('/nipplejs', express.static(__dirname + '/node_modules/nipplejs/dist'));

app.get('/', function (req, res) {
    res.sendFile(__dirname + "/index.html");
});

app.get('/stance', function (req, res) {
    res.sendFile(__dirname + "/stance.html");
});

app.get('/fastStance', function (req, res) {
    res.sendFile(__dirname + "/fastStance.html");
});

robot.registerForTelemetrics(function(msg){
    io.sockets.emit('telemetrics', msg);
});

io.on('connection', function (socket) {
    robot.log(`User connected from ${socket.request.connection.remoteAddress}`);
    var lastCommand = { x: 0, y: 0, rot: 0 };
    socket.on('direction', function (msg) {
        lastCommand.x = msg.x * 6;
        lastCommand.y = msg.y * 6;
        robot.sendCommandToRobot(lastCommand.x, lastCommand.y, lastCommand.rot);
    });
    socket.on('rotation', function (msg) {
        lastCommand.rot = msg.y * -10;
        robot.sendCommandToRobot(lastCommand.x, lastCommand.y, lastCommand.rot);
    });
    socket.on('translationUpdate', function (msg) {
        robot.sendUpdateStanceCommand(msg.transform, msg.rotation);
    });
    socket.on('walkingModeUpdate', function (msg) {
        robot.sendWalkingModeUpdate(msg.staticSpeedMode, msg.liftHeight);
    });
    socket.on('disconnected', function () {
        robot.log(`User disconnected from ${socket.request.connection.remoteAddress}`);
    });
});

http.listen(3000, function () {
    robot.log('Listening');
});

