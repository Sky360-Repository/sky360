/**
 * Basic implementation of a history and realtime server.
 */

var Spacecraft = require('./spacecraft'); // tutorial file
var RealtimeServer = require('./realtime-server'); // tutorial file
var HistoryServer = require('./history-server'); // tutorial file
var StaticServer = require('./static-server'); // tutorial file


var expressWs = require('express-ws');
var app = require('express')();
expressWs(app);

var spacecraft = new Spacecraft();
var realtimeServer = new RealtimeServer(spacecraft);
var historyServer = new HistoryServer(spacecraft);
var staticServer = new StaticServer();



app.use('/realtime', realtimeServer);
app.use('/history', historyServer);
app.use('/', staticServer);

var port = process.env.PORT || 8080

app.listen(port, function () {
    console.log('Open MCT hosted at http://localhost:' + port);
    console.log('History hosted at http://localhost:' + port + '/history');
    console.log('Realtime hosted at ws://localhost:' + port + '/realtime');
});
