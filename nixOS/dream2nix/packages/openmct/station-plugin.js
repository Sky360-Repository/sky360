function getStation() {
    return http.get('/station.json')
        .then(function (result) {
            return result.data;
        });
}


var objectStation = {
    get: function (identifier) {
        return getStation().then(function (station) {
            if (identifier.key === 'sky360station') {
                return {
                    identifier: identifier,
                    name: station.name,
                    type: 'folder',
                    location: 'ROOT'
                };
            } else {
                var measurement = station.measurements.filter(function (m) {
                    return m.key === identifier.key;
                })[0];
                return {
                    identifier: identifier,
                    name: measurement.name,
                    type: 'example.telemetry',
                    telemetry: {
                        values: measurement.values
                    },
                    location: 'sample2.taxonomy:station'
                };
            }
        });
    }
};

var compositionProvider = {
    appliesTo: function (domainObject) {
        return domainObject.identifier.namespace === 'sample2.taxonomy' &&
               domainObject.type === 'folder';
    },
    load: function (domainObject) {
        return getStation()
            .then(function (station) {
                return station.measurements.map(function (m) {
                    return {
                        namespace: 'sample2.taxonomy',
                        key: m.key
                    };
                });
            });
    }
};



function StationPlugin() {
    return function install(openmct) {
        //console.log("I've been installed -yay!!");
        openmct.objects.addRoot({
        	namespace: 'sample2.taxonomy',
        	key: 'sky360station'
        });
        
        openmct.objects.addProvider('sample2.taxonomy', objectStation);
        
        openmct.composition.addProvider(compositionProvider);

        openmct.types.addType('example.telemetry', {
            name: 'Station Data',
            description: 'Temeletry points from Sky360 Station and Online APIs.', 
            cssClass: 'icon-telemetry'
        });        
    }
};
