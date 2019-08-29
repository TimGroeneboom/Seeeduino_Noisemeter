
var app = function(){
    var webserver = require('./webserver.js').app();
    var spreadsheet = require('./spreadsheet.js').app();
    
    var http = require('http');

    const server = http.createServer(function(request, response) {
        console.dir(request.param)

        if (request.method == 'POST') {
          console.log('POST')
          var body = ''
          request.on('data', function(data) {
            body += data

            try{
                var json = JSON.parse(data);
                webserver.addPayload(data);
                spreadsheet.addPayload(data);
            }catch(error){
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
      
      const port = process.env.PORT_THINGSNETWORK || 8081
      server.listen(port)
      console.log(`Listening at http://${port}`);
}
app();

  
