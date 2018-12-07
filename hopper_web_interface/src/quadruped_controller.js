const isLinux = process.platform.toLowerCase() === "linux";

var hopperMoveCommandPublisher;
var hopperStanceTranslateCommandPublisher;
var hopperWalkingModePublisher;
var hopperTelemetricsSubscriber;
var telemetricsSubscribers = [];

function telemetricsHandler(msg) {
    telemetricsSubscribers.forEach(function (subscriber) {
        subscriber(msg);
    });
}

if (isLinux) {
    var rosnodejs = require('rosnodejs')
    rosnodejs.initNode('hopper_web_interface')
        .then((node) => {
            hopperMoveCommandPublisher = node.advertise('/hopper/move_command', 'hopper_msgs/HopperMoveCommand');
            hopperStanceTranslateCommandPublisher = node.advertise('/hopper/stance_translate', 'geometry_msgs/Twist');
            hopperTelemetricsSubscriber = node.subscribe('/hopper_telemetrics', 'hopper_msgs/HexapodTelemetrics', telemetricsHandler);
        });
}

exports.registerForTelemetrics = function (callback) {
    if (isLinux) {
        telemetricsSubscribers.push(callback);
    }
    else {
        console.log("Can not listen for topics if not on ROS platform");
    }
}

exports.sendCommandToRobot = function (x, y, rotation, liftHeight) {
    if (isLinux) {
        if (hopperMoveCommandPublisher) {
            hopperMoveCommandPublisher.publish(
                {
                    direction: {
                        linear: { x: x, y: y, z: 0 },
                        angular: { x: 0, y: 0, z: rotation }
                    },
                    lift_height: liftHeight,
                    cycle_time: 1
                });
        }
        else {
            rosnodejs.log.info("Publisher not connected");
        }
    }
    else {
        console.log(`Robot moving to X:${x.toFixed(2)} Y:${y.toFixed(2)}, Rotation:${rotation.toFixed(2)}`);
    }
}

exports.sendUpdateStanceCommand = function (transform, rotation) {
    if (isLinux) {
        if (hopperStanceTranslateCommandPublisher) {
            hopperStanceTranslateCommandPublisher.publish({ linear: transform, angular: rotation });
        }
        else {
            rosnodejs.log.info("Publisher not connected");
        }
    }
    else {
        console.log(`Stance updating:\n` +
            `    Transform: x:${transform.x.toFixed(2)}, y:${transform.y.toFixed(2)}, z:${transform.z.toFixed(2)}\n` +
            `    Rotation: x:${rotation.x.toFixed(2)}, y:${rotation.y.toFixed(2)}, z:${rotation.z.toFixed(2)}`);
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