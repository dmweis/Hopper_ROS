function createStanceJoystick(element, color, joystickData, deadZone) {
    nipplejs.create({ zone: element, color: color }).on('added',
        function (evt, nipple) {
            nipple.on('move',
                function (evt, arg) {
                    const size = arg.distance / 50;
                    if (size < deadZone) {
                        joystickData.x = 0;
                        joystickData.y = 0;
                        return;
                    }
                    joystickData.x = Math.sin(arg.angle.radian) * size;
                    joystickData.y = -Math.cos(arg.angle.radian) * size;
                });
            nipple.on('start',
                function () {
                    joystickData.x = 0;
                    joystickData.y = 0;
                });
            nipple.on('end',
                function () {
                    joystickData.x = 0;
                    joystickData.y = 0;
                });
        });
}

function convertToTwist(translationJoystick, heightJoystick, rotationJoystick) {
    const multiplier = 5;
    return {
        transform: {
            x: translationJoystick.x * multiplier,
            y: translationJoystick.y * multiplier,
            z: heightJoystick.x * multiplier
        },
        rotation: {
            x: -rotationJoystick.y * multiplier,
            y: rotationJoystick.x * multiplier,
            z: -heightJoystick.y * multiplier
        }
    }
}

var socket = io();

const translationViewModel = new Vue({
    el: "#app",
    data: {
        connected: false,
        translationJoystick: {
            x: 0,
            y: 0
        },
        heightJoystick: {
            x: 0,
            y: 0
        },
        rotationJoystick: {
            x: 0,
            y: 0
        }
    },
    watch: {
        translationJoystick: {
            handler: function (newValue, oldValue) {
                socket.emit("translationUpdate", convertToTwist(this.translationJoystick, this.heightJoystick, this.rotationJoystick));
            },
            deep: true
        },
        heightJoystick: {
            handler: function (newValue, oldValue) {
                socket.emit("translationUpdate", convertToTwist(this.translationJoystick, this.heightJoystick, this.rotationJoystick));
            },
            deep: true
        },
        rotationJoystick: {
            handler: function (newValue, oldValue) {
                socket.emit("translationUpdate", convertToTwist(this.translationJoystick, this.heightJoystick, this.rotationJoystick));
            },
            deep: true
        },
        rotation: {
            handler: function (newValue, oldValue) {
                socket.emit("translationUpdate", {
                    transform: this.transform,
                    rotation: this.rotation
                });
            },
            deep: true
        },
        connected: function () {
            console.log("Connection status changed to " + this.connected);
        }
    }
})

createStanceJoystick(document.getElementById("translation_joystick_zone"), "red", translationViewModel.translationJoystick, 0.2);
createStanceJoystick(document.getElementById("height_joystick_zone"), "navy", translationViewModel.heightJoystick, 0.2);
createStanceJoystick(document.getElementById("rotate_joystick_zone"), "navy", translationViewModel.rotationJoystick, 0.2);

setInterval(function () {
    translationViewModel.connected = socket.connected;
}, 250);
