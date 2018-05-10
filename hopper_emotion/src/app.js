#!/usr/bin/env node

const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const faceDetectionInterface = require('./facedetection_interface.js');

app.use(express.static(__dirname + '/web_app'));
app.use(express.static(__dirname + '/static'));

app.get('/', function (req, res) {
    res.sendFile(__dirname + "/index.html");
});

var lastReceivedImage = null;

app.get('/image.jpeg', function (req, res) {
    res.contentType('image/jpeg');
    if (lastReceivedImage){
        res.end(lastReceivedImage.data, 'binary');
    }
});

faceDetectionInterface.registerForFaceDetection(function(msg){
    lastReceivedImage = msg.image;
    io.sockets.emit('newImage', msg.detected_faces);
});

io.on('connection', function (socket) {
    faceDetectionInterface.log(`User connected from ${socket.request.connection.remoteAddress}`);
    socket.on('disconnected', function () {
        faceDetectionInterface.log(`User disconnected from ${socket.request.connection.remoteAddress}`);
    });
});

http.listen(3001, function () {
    faceDetectionInterface.log('Listening');
});
