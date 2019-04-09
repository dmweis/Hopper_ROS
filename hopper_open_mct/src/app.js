#!/usr/bin/env node

const express = require('express');
const app = express();

app.use(express.static(__dirname + '/web_app'));

app.use('/lib', express.static(__dirname + '/lib'));
app.use('/openmct', express.static(__dirname + '/node_modules/openmct/dist'));
app.use('/roslib', express.static(__dirname + '/node_modules/roslib/build'));
app.use('/eventemitter2', express.static(__dirname + '/node_modules/eventemitter2/lib'));

app.get('/', function (req, res) {
    res.sendFile(__dirname + "index.html");
});

var port = process.env.PORT || 3001

app.listen(port, function () {});