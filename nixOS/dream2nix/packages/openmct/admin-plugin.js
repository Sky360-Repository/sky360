function getAdmin() {
    return http.get('/admin.json')
        .then(function (result) {
            return result.data;
        });
}


var objectAdmin = {
    get: function (identifier) {
        return getAdmin().then(function (admin) {
            if (identifier.key === 'administration') {
                return {
                    identifier: identifier,
                    name: admin.name,
                    type: 'folder',
                    location: 'ROOT'
                };
            }
        });
    }
};


function AdminPlugin() {
    return function install(openmct) {
        //console.log("I've been installed -yay!!");
        openmct.objects.addRoot({
        	namespace: 'sample.taxonomy',
        	key: 'administration'
        });
        
        openmct.objects.addProvider('sample.taxonomy', objectAdmin);
    }
};
