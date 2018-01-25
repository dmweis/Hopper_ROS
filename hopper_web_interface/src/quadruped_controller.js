const isLinux = process.platform.toLowerCase() === "linux";

var publisher;

if (isLinux) {
    var rosnodejs = require('rosnodejs')
    rosnodejs.initNode('hopper_web_interface')
        .then((node) => {
            publisher = node.advertise('/quadruped_command', 'geometry_msgs/Twist');
        });

}

exports.sendCommandToRobot = function (x, y, rotation) {
    if (isLinux) {
        if (publisher) {
            publisher.publish({ linear: { x: x, y: y, z: 0 }, angular: { x: rotation, y: 0, z: 0 } });
        }
        else {
            rosnodejs.log.info("Publisher not connected");
        }
    }
    else {
        console.log(`Robot moving to X:${x.toFixed(2)} Y:${y.toFixed(2)}, Rotation:${rotation.toFixed(2)}`);
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