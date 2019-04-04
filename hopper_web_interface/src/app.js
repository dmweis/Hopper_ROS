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

require('socketio-auth')(io, {
    authenticate: function (socket, data, callback) {
        //get credentials sent by the client
        var username = data.username;
        var password = data.password;
        console.log(`User auth request ${username} passed: ${password}`);
        callback(null, robot.validateUser(username, password));
    }
});

app.get('/', function (req, res) {
    res.sendFile(__dirname + "/index.html");
});

app.get('/stance', function (req, res) {
    res.sendFile(__dirname + "/stance.html");
});

app.get('/fastStance', function (req, res) {
    res.sendFile(__dirname + "/fastStance.html");
});

robot.registerForTelemetry(function (msg) {
    io.sockets.emit('telemetry', msg);
});

io.on('connection', function (socket) {
    robot.log(`User connected from ${socket.request.connection.remoteAddress}`);
    var lastCommand = { x: 0, y: 0, rot: 0 };
    var liftHeight = 3;
    socket.on('direction', function (msg) {
        lastCommand.x = msg.x * .1;
        lastCommand.y = msg.y * .1;
        robot.sendCommandToRobot(lastCommand.x, lastCommand.y, lastCommand.rot, liftHeight);
    });
    socket.on('rotation', function (msg) {
        lastCommand.rot = msg.y * 0.7;
        robot.sendCommandToRobot(lastCommand.x, lastCommand.y, lastCommand.rot, liftHeight);
    });
    socket.on('translationUpdate', function (msg) {
        robot.sendUpdateStanceCommand(msg.transform, msg.rotation);
    });
    socket.on('walkingModeUpdate', function (msg) {
        liftHeight = msg.liftHeight;
    });
    socket.on('disconnected', function () {
        robot.log(`User disconnected from ${socket.request.connection.remoteAddress}`);
    });
});

http.listen(3000, function () {
    robot.log('Listening');
});

