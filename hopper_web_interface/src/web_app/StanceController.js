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

var socket = createSocketIOConnection();

const translationViewModel = new Vue({
    el: "#app",
    data: {
        connected: false,
        transform: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0 },
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
        transform: {
            handler: function (newValue, oldValue) {
                socket.emit("translationUpdate", {
                    transform: this.transform,
                    rotation: this.rotation
                });
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
    },
    methods: {
        reset: function () {
            this.transform = { x: 0, y: 0, z: 0 };
            this.rotation = { x: 0, y: 0, z: 0 };
        }
    }
})

createStanceJoystick(document.getElementById("translation_joystick_zone"), "red", translationViewModel.translationJoystick, 0.2);
createStanceJoystick(document.getElementById("height_joystick_zone"), "navy", translationViewModel.heightJoystick, 0.2);
createStanceJoystick(document.getElementById("rotate_joystick_zone"), "navy", translationViewModel.rotationJoystick, 0.2);

setInterval(function () {
    const multiplier = 0.001;
    translationViewModel.transform.x += translationViewModel.translationJoystick.x * multiplier;
    translationViewModel.transform.y += translationViewModel.translationJoystick.y * multiplier;
    translationViewModel.transform.z += translationViewModel.heightJoystick.x * multiplier;
    translationViewModel.rotation.y += translationViewModel.rotationJoystick.x * multiplier;
    translationViewModel.rotation.x -= translationViewModel.rotationJoystick.y * multiplier;
    translationViewModel.rotation.z -= translationViewModel.heightJoystick.y * multiplier;
}, 20);

setInterval(function () {
    translationViewModel.connected = socket.connected;
}, 250);
