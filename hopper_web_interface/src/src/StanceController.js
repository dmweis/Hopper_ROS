var socket = io();

const telemetricsVm = new Vue({
    el: '#telemetrics_display',
    data: {
        averageTemperature: 0,
        averageVoltage: 0
    }
});

socket.on('telemetrics', function (msg) {
    telemetricsVm.averageVoltage = msg.AverageVoltage;
    telemetricsVm.averageTemperature = msg.AverageTemperature;
});

const translationJoystick = createJoystick({
    elementId: "#translation_joystick_zone",
    color: "red",
    name: "translation",
    socket: socket
});

const heightJoystick = createJoystick({
    elementId: "#height_joystick_zone",
    color: "navy",
    name: "height",
    socket: socket
});

const bodyRotationJoystick = createJoystick({
    elementId: "#rotate_joystick_zone",
    color: "navy",
    name: "bodyRotation",
    socket: socket
});
