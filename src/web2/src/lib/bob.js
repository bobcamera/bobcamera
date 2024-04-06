// Define the Websockets address of the ROS2 environment
var websocketsURL = 'ws://' + window.location.hostname + ':9090';

// Set up the connection to the ROS2 environment using the roslib.js library
function rosConnect(ros, websocketsURL, retry=false){
    if(ros) {
        ros.connect(websocketsURL);
    } else {
        console.log('Creating new ros connection...');
        var ros = new ROSLIB.Ros();
        ros.connect(websocketsURL);
    }
    ros.on('connection', function(){
        //console.log('Connected to websocket server.')
    });
    ros.on('error', function(){
        //console.log('Error connecting to websocket server: ', error);
    });
    ros.on('close', function(){            
        // Try to reconnect in 5 seconds
        if (retry) {
            //console.log('Connection to websocket server closed, trying in 5 seconds.');
            setTimeout(function(){rosConnect(ros, websocketsURL)}, 5000);
        }
    });
}
