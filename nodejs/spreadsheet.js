exports.app = function(){
  var spreadsheet = require('google-spreadsheet');
  var async = require('async');
  
  // spreadsheet key is the long id in the sheets URL
  var doc = new spreadsheet('1wF8Mqemvph9G4DmANDbDHapTUVBIWyL9EKNetHuucso');
  var sheet;
  
  var log = {
    devices : []
  };
  
  var setAuth = function setAuth(step) {
    // see notes below for authentication instructions!
    var creds = require('./google-generated-creds.json');
    // OR, if you cannot save the file locally (like on heroku)
    var creds_json = {
      client_email: 'timski@wiijtimski.com',
      private_key: 'your long private key stuff here'
    }
  
    doc.useServiceAccountAuth(creds, step);
  };

  
  return {
    addPayload : function (json) {
      // add it to the log
      var deviceLog = log.devices[json.dev_id];
      if (deviceLog == undefined) {
        deviceLog = {
          decibels: []
        };
        log.devices[json.dev_id];
      }

      deviceLog.decibels.push(json.payload_fields.decibel);

      async.series([
        setAuth,
        function getInfoAndWorksheets(step) {
          doc.getInfo(function (err, info) {
            console.log('Loaded doc: ' + info.title + ' by ' + info.author.email);
            sheet = info.worksheets[0];
            console.log('sheet 1: ' + sheet.title + ' ' + sheet.rowCount + 'x' + sheet.colCount);

            // add it to google sheet
            doc.addRow(1, {
              time: json.metadata.time,
              device_id: json.dev_id,
              decibel: json.payload_fields.decibel,
              voltage: json.payload_fields.voltage,
              validLocation: json.payload_fields.validLocation,
              lat: json.payload_fields.lat,
              lon: json.payload_fields.lon,
              payload: json.payload_raw,
              framecounter: json.counter,
              metadata: JSON.stringify(json.metadata)
            }, function (err) {
              if (err) {
                console.log('Error: ' + err);
              }
            });
          });
        }
      ], function (err) {
        if (err) {
          console.log('Error: ' + err);
        }
      });
    }
  }
}

