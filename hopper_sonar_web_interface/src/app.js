#!/usr/bin/env node

const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);
app.use(express.static(__dirname + '/public'));

const SerialPort = require('serialport');
const parsers = SerialPort.parsers;

app.get('/', function (req, res) {
  res.sendFile(__dirname + "/index.html");
});

const parser = new parsers.Readline({
  delimiter: '\r\n'
});
const port = new SerialPort('/dev/ttyUSB0', {
  baudRate: 9600
});
port.pipe(parser);

port.on('open', () => console.log('Port open'));

parser.on('data', function (msg) {
  try {
    io.sockets.emit('data', JSON.parse(msg));
  }
  catch (e) {
    console.log("Parse error");
  }
});

http.listen(3001, function () {
  console.log('Listening');
});
