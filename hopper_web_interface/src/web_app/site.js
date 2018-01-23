function createJoystick(data) {
    const vm = new Vue({
        el: data.elementId,
        data: {
            angle: 0,
            distance: 0,
            x: 0,
            y: 0,
            deadZone: 0.2,
            color: data.color,
            name: data.name,
            touchDown: false
        },
        mounted: function () {
            const context = this;
            nipplejs.create({ zone: this.$el, color: this.color }).on('added',
                function (evt, nipple) {
                    nipple.on('move',
                        function (evt, arg) {
                            context.angle = arg.angle.radian;
                            context.distance = arg.distance;
                        });
                    nipple.on('start',
                        function () {
                            context.angle = 0;
                            context.distance = 0;
                            context.touchDown = true;
                        });
                    nipple.on('end',
                        function () {
                            context.angle = 0;
                            context.distance = 0;
                            context.touchDown = false;
                        });
                });
        },
        watch: {
            touchDown: function () {
                data.socket.emit('joystickUpdate', {
                    joystickType: this.name,
                    MessageType: this.touchDown ? 'start' : 'stop'
                });
            }
        }
    });

    vm.$watch(function () {
        return this.angle + this.distance;
    }, function (newValue, oldValue) {
        const size = this.distance / 50;
        if (size < this.deadZone) {
            this.x = 0;
            this.y = 0;
            return;
        }
        this.y = -Math.cos(this.angle) * size;
        this.x = Math.sin(this.angle) * size;
    });
    vm.$watch(function () {
        return this.x + this.y;
    }, function (newValue, oldValue) {
        data.socket.emit(this.name, {
            x: this.x,
            y: this.y
        });
    });

    return vm;
}