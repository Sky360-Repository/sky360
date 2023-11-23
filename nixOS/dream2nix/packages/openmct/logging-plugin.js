function getLogs() {
    return http.get('/logging.json')
        .then(function (result) {
            return result.data;
        });
}


var objectLogging = {
    get: function (identifier) {
        return getLogs().then(function (logs) {
            if (identifier.key === 'logging') {
                return {
                    identifier: identifier,
                    name: logs.name,
                    type: 'folder',
                    location: 'ROOT'
                };
            }
        });
    }
};


function LoggingPlugin() {
    return function install(openmct) {
        //console.log("I've been installed -yay!!");
        openmct.objects.addRoot({
        	namespace: 'sample3.taxonomy',
        	key: 'logging'
        });
        
        openmct.objects.addProvider('sample3.taxonomy', objectLogging);
    }
};
