
exports.go = function (){
    const express = require('express');
    const app = express();
    const server = require('http').Server(app);
    const io = require('socket.io')(server);
    const hbs = require('hbs');
    const ttn = require('ttn');
    const fs = require('fs');
    const Path = require('path');
    
    // improved database
    const dbFile = Path.join(__dirname, 'db.json');

    //
    var functions = {};
    
    // Some options for express (node.js web app library)
    hbs.registerPartials(__dirname + '/views/partials');
    app.use(express.static(__dirname + '/public'));
    app.set('view engine', 'html');
    app.set('views', __dirname + '/views');
    app.engine('html', hbs.__express);
    
    // Store some state about all applications
    let applications = {};
    
    // Store some state about all devices
    let devices = {};
    
    // Note here what data you're interested in, and how it can be obtained from a payload message
    const config = {
        // Replace this with your own key
        mapsApiKey: 'AIzaSyBVHcLKC3ja-ZCsyWQLe0eBB3q28F7V6X0',
    
        title: 'Decibel monitor',
        dataMapping: {
            decibel: {
                graphTitle: 'Decibel',
                yAxisLabel: 'Decibel',
                minY: 20, // suggested numbers, if numbers out of this range are received the graph will adjust
                maxY: 120,
                numberOfEvents: 60, // no. of events we send to the client
                data: payload => payload.payload_fields.decibel
            },
            // want more properties? just add more objects here
        },
        mapCenter: {
            lat: 52.370216,
            lng: 4.895168
        }
    };
    
    const dataMapping = config.dataMapping;
    const mapCenter = config.mapCenter;
    
    // And handle requests
    app.get('/', function (req, res, next) {
        let d = Object.keys(devices).map(k => {
    
            let keys = k.split(/\:/g);
            let o = {
                appId: keys[0],
                devId: keys[1],
                eui: devices[k].eui,
                lat: devices[k].lat,
                lng: devices[k].lng,
            };
    
            for (let mapKey of Object.keys(dataMapping)) {
                devices[k][mapKey] = devices[k][mapKey] || [];
    
                // grab last X events from the device
                o[mapKey] = devices[k][mapKey].slice(Math.max(devices[k][mapKey].length - (dataMapping[mapKey].numberOfEvents || 30), 1));
            }
    
            return o;
        })
        // Render index view, with the devices based on mapToView function
        res.render('index', {
            devices: JSON.stringify(d),
            config: JSON.stringify(config),
            title: config.title,
            mapsApiKey: config.mapsApiKey
        });
    });

    //
    functions.addDecibel = function (devId, payload) {
        console.log('[%s] Received uplink', appId, devId, payload.payload_fields);

        let key = appId + ':' + devId;
        let d = devices[key] = devices[key] || {};
        d.eui = payload.hardware_serial;

        for (let mapKey of Object.keys(dataMapping)) {
            d[mapKey] = d[mapKey] || [];
        }
         d.lat = payload.payload_fields.lon;
            d.lng = payload.payload_fields.lat;


        for (let mapKey of Object.keys(dataMapping)) {
            let v;
            try {
                v = dataMapping[mapKey].data(payload);
            }
            catch (ex) {
                console.error('dataMapping[' + mapKey + '].data() threw an error', ex);
                throw ex;
            }
            console.log(v, typeof v);

            if (typeof v !== 'undefined') {
                d[mapKey].push({
                    ts: new Date(payload.metadata.time),
                    value: v
                });

                io.emit('value-change', mapKey, {
                    appId: appId,
                    devId: devId,
                    eui: d.eui,
                    lat: d.lat,
                    lng: d.lng
                }, payload.metadata.time, v);
            }
        }
    }
    
    io.on('connection', socket => {
        /*
        socket.on('connect-application', (appId, accessKey) => {
            console.log('Connecting to application', appId, accessKey);
            connectApplication(appId, accessKey)
                .then(() => socket.emit('connected', appId))
                .catch(err => socket.emit('connect-failed', JSON.stringify(err)));
        });*/
    
        socket.on('location-change', (appId, devId, lat, lng) => {
            let key = appId + ':' + devId;
            if (!devices[key]) {
                console.error('Device not found', appId, devId);
                return;
            }
    
            console.log('Location changed', appId, devId, lat, lng);
    
            let d = devices[key];
            d.lat = lat;
            d.lng = lng;
    
            io.emit('location-change', {
                appId: appId,
                devId: devId,
                eui: d.eui,
                lat: d.lat,
                lng: d.lng
            }, lat, lng);
        });
    });
    
    var port = 8081;
    server.listen(port, function () {
        console.log('Web server listening on port %s!', port);
    });
    
    function connectApplication(appId, accessKey) {
        if (applications[appId]) {
            if (!applications[appId].client) {
                throw 'Already connecting to app ' + appId;
            }
            applications[appId].client.close();
            delete applications[appId];
        }
    
        applications[appId] = {
            accessKey: accessKey
        }
    
        console.log('[%s] Connecting to TTN', appId);
        return new Promise((resolve, reject) => {
    
            return ttn.DataClient(appId, accessKey, 'eu.thethings.network:1883').then(client => {
                applications[appId].client = client;
    
                client.on('error', (err) => {
                    if (err.message === 'Connection refused: Not authorized') {
                        console.error('[%s] Key is not correct', appId);
                        client.close();
                        delete applications[appId];
                    }
                    reject(err);
                });
    
                client.on('connect', () => {
                    console.log('[%s] Connected over MQTT', appId);
                    resolve();
                });
    
                client.on('uplink', (devId, payload) => {
                    // on device side we did /100, so *100 here to normalize
                    if (typeof payload.payload_fields.analog_in_1 !== 'undefined') {
                        payload.payload_fields.analog_in_1 *= 100;
                    }
    
                    console.log('[%s] Received uplink', appId, devId, payload.payload_fields);
    
                    let key = appId + ':' + devId;
                    let d = devices[key] = devices[key] || {};
                    d.eui = payload.hardware_serial;
    
                    for (let mapKey of Object.keys(dataMapping)) {
                        d[mapKey] = d[mapKey] || [];
                    }
    
                    if (!d.lat) {
                        d.lat = mapCenter.lat + (Math.random() / 10 - 0.05);
                    }
                    if (!d.lng) {
                        d.lng = mapCenter.lng + (Math.random() / 10 - 0.05);
                    }
    
                    for (let mapKey of Object.keys(dataMapping)) {
                        let v;
                        try {
                            v = dataMapping[mapKey].data(payload);
                        }
                        catch (ex) {
                            console.error('dataMapping[' + mapKey + '].data() threw an error', ex);
                            throw ex;
                        }
                console.log(v, typeof v);
    
                        if (typeof v !== 'undefined') {
                            d[mapKey].push({
                                ts: new Date(payload.metadata.time),
                                value: v
                            });
    
                            io.emit('value-change', mapKey, {
                                appId: appId,
                                devId: devId,
                                eui: d.eui,
                                lat: d.lat,
                                lng: d.lng
                            }, payload.metadata.time, v);
                        }
                    }
                });
    
                console.log('[%s] Acquired MQTT client, connecting...', appId);
            }).catch(err => {
                console.error('[%s] Could not connect to The Things Network', appId, err);
                delete applications[appId];
                reject(err);
            });
        });
    }
    
    function exitHandler(options, err) {
        if (err) {
            console.error('Application exiting...', err);
        }
    
        let db = {
            devices: devices,
            applications: {}
        }
        for (appId in applications) {
            if (applications.hasOwnProperty(appId)) {
                db.applications[appId] = applications[appId].accessKey;
            }
        }
        fs.writeFileSync(dbFile, JSON.stringify(db), 'utf-8');
    
        if (options.exit) {
            process.exit();
        }
    }
    
    process.on('exit', exitHandler.bind(null, { cleanup: true }));
    process.on('SIGINT', exitHandler.bind(null, { exit: true }));
    process.on('SIGUSR1', exitHandler.bind(null, { exit: true }));
    process.on('SIGUSR2', exitHandler.bind(null, { exit: true }));
    process.on('uncaughtException', exitHandler.bind(null, { exit: true }));
    
    return functions;
}
