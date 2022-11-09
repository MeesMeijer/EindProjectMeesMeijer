// import { WebSocketServer } from 'ws';

// const wss = new WebSocketServer({ port: 8080 });


let esps = [
    {
        id: 1,
        locationId: 1,
        connected: false
    }
]

let locations = {
  1: "Koelkast"
}


function getEsp(id){
  for (const esp of esps){ 
    if (esp.id == id){
      return esp 
    }
  }
}


function getLocation(id){
    const esp = getEsp(id);
    return esp ? locations[esp.locationId] : undefined
}


function changeLocation(espId, locationId){
  let esp = getEsp(espId);
  esp.locationId = locationId
  console.log(esps);
}

console.log (getLocation(1))

// wss.on('connection', function connection(ws) {
//   ws.on('message', function message(data) { 


//     console.log('received: %s', data);

//   });

//   ws.send('something');
// });