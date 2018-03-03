
function setGamepadWatchdog(socket, deadzone) {
    function axisEquals(left, right) {
        return left.lx === right.lx &&
            left.ly === right.ly &&
            left.rx === right.rx &&
            left.ry === right.ry;
    };
    function applyDeadzone(gamepadData){
        if (Math.abs(gamepadData.lx) < deadzone) gamepadData.lx = 0;
        if (Math.abs(gamepadData.ly) < deadzone) gamepadData.ly = 0;
        if (Math.abs(gamepadData.rx) < deadzone) gamepadData.rx = 0;
        if (Math.abs(gamepadData.ry) < deadzone) gamepadData.ry = 0;
    };
    let lastGamepadData = {
        lx: 0,
        ly: 0,
        rx: 0,
        ry: 0,
        id: 0
    };
    function processGamepadData(gamepadData) {
        applyDeadzone(gamepadData);
        if (!axisEquals(lastGamepadData, gamepadData)) {
            lastGamepadData = gamepadData;
            socket.emit("direction", {
                x: -gamepadData.lx,
                y: -gamepadData.ly
            });
            socket.emit("rotation", {
                x: gamepadData.rx,
                y: -gamepadData.ry
            });
        }
    };

    function queryGamepads() {
        var gamepads = navigator.getGamepads();
        for (let index = 0; index < gamepads.length; index++) {
            const gamepad = gamepads[index];
            if (gamepad) {
                let gamepadData = {
                    lx: gamepad.axes[1],
                    ly: gamepad.axes[0],
                    rx: 0,
                    ry: gamepad.axes[2],
                    id: gamepad.index
                };
                processGamepadData(gamepadData);
                return;
            }
        }
    };

    setInterval(queryGamepads, 50);
}

