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

const movementJoystick = createJoystick({
    elementId: "#move_joystick_zone",
    color: "red",
    name: "direction",
    socket: socket
});

const rotationJoystick = createJoystick({
    elementId: "#rotate_joystick_zone",
    color: "navy",
    name: "rotation",
    socket: socket
});
