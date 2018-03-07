const isLinux = process.platform.toLowerCase() === "linux";

var hopperFaceSubscriber;
var faceDetectionSubscribers = [];

function faceDetectionHandler(msg){
    faceDetectionSubscribers.forEach(function(subscriber){
        subscriber(msg);
    });
}

if (isLinux) {
    var rosnodejs = require('rosnodejs')
    rosnodejs.initNode('hopper_web_emotion_interface')
        .then((node) => {
            hopperFaceSubscriber = node.subscribe('/detected_faces', 'hopper_emotion_core/FaceDetectionImage', faceDetectionHandler);
        });
}

exports.registerForFaceDetection = function(callback){
    if (isLinux) {
        faceDetectionSubscribers.push(callback);
    }
    else {
        console.log("Can not listen for topics if not on ROS platform");
    }
}

exports.log = function (message) {
    if (rosnodejs) {
        rosnodejs.log.info(message);
    }
    else {
        console.log(message);
    }
}
