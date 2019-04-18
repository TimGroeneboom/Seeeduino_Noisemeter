var http = require('http');
var spreadsheet = require('google-spreadsheet');
var async = require('async');

// spreadsheet key is the long id in the sheets URL
var doc = new spreadsheet('1wF8Mqemvph9G4DmANDbDHapTUVBIWyL9EKNetHuucso');
var sheet;

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

const server = http.createServer(function(request, response) {
  console.dir(request.param)

  if (request.method == 'POST') {
    console.log('POST')
    var body = ''
    request.on('data', function(data) {
      body += data
      //console.log('Partial body: ' + body)

      try
      {
        var json = JSON.parse(data);
        if( json ){
          console.log(json);
  
          async.series([
            setAuth,
            function getInfoAndWorksheets(step) {
              doc.getInfo(function(err, info) {
                console.log('Loaded doc: '+info.title+' by '+info.author.email);
                sheet = info.worksheets[0];
                console.log('sheet 1: '+sheet.title+' '+sheet.rowCount+'x'+sheet.colCount);
  
                doc.addRow(1, {
                  time : json.metadata.time,
                  device_id : json.dev_id,
                  decibel : json.payload_fields.decibel,
                  voltage : json.payload_fields.voltage,
                  validLocation : json.payload_fields.validLocation,
                  lat : json.payload_fields.lat,
                  lon : json.payload_fields.lon,
                  payload : json.payload_raw,
                  framecounter : json.counter,
                  metadata : JSON.stringify( json.metadata )
                }, function(err){
                  if( err ) {
                    console.log('Error: '+err);
                  }
                });
              });
            }
          ], function(err){
            if( err ) {
              console.log('Error: '+err);
            }
          });
        }
      }catch(error)
      {
        console.log(error);
      }
    });
    request.on('end', function() {
      console.log('Body: ' + body)
      response.writeHead(200, {'Content-Type': 'text/html'})
      response.end('post received')
    })
  } 
})

const port = 8080
server.listen(port)
console.log(`Listening at http://${port}`)
