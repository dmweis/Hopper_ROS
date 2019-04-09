function getObjectDescription() {
    return http.get('/objects.json')
        .then(function (result) {
            return result.data;
        });
}


var objectProvider = {
    get: function (identifier) {
        return getObjectDescription().then(function (objects) {
            if (identifier.key === 'hopper') {
                return {
                    identifier: identifier,
                    name: objects.name,
                    type: 'folder',
                    location: 'ROOT'
                };
            } else {
                var measurement = objects.measurements.filter(function (m) {
                    return m.key === identifier.key;
                })[0];
                
                return {
                    identifier: identifier,
                    name: measurement.name,
                    type: 'hopper.telemetry',
                    telemetry: {
                        values: measurement.values
                    },
                    location: 'hopper.taxonomy:hopper'
                };
            }
        });
    }
};

var compositionProvider = {
    appliesTo: function (domainObject) {
        return domainObject.identifier.namespace === 'hopper.taxonomy' &&
            domainObject.type === 'folder';
    },
    load: function (domainObject) {
        return getObjectDescription()
            .then(function (objects) {
                return objects.measurements.map(function (m) {
                    return {
                        namespace: 'hopper.taxonomy',
                        key: m.key
                    };
                });
            });
    }
};


function HopperPlugin() {
    return function install(openmct) {
        openmct.objects.addRoot({
            namespace: 'hopper.taxonomy',
            key: 'hopper'
        });

        openmct.objects.addProvider('hopper.taxonomy', objectProvider);

        openmct.composition.addProvider(compositionProvider);

        openmct.types.addType('hopper.telemetry', {
            name: 'hopper telemetry Point',
            description: 'Telemetry point for hopper',
            cssClass: 'icon-telemetry'
        });
    };
};