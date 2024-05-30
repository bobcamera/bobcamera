// Define the Websockets address of the ROS2 environment
var websocketsURL = 'ws://' + window.location.hostname + ':9090';
//var ros; // Global ROS connection
var subscribedTopics = [];

// Set up the connection to the ROS2 environment using the roslib.js library
function rosConnect(ros, websocketsURL, retry=false) {
    if (!ros) {
        console.log('Creating new ros connection...');
        ros = new ROSLIB.Ros();
        ros.hasListeners = false; // Flag to track if listeners are set
        ros.isConnected = false;
    }

    if (ros.isConnected) {
        console.log('Already connected to websocket server.');
        return ros;
    }

    if (!ros.hasListeners) { // Ensure listeners are only set once
        ros.on('connection', function() {
            console.log('Connected to websocket server.');
            ros.isConnected = true;
        });

        ros.on('error', function(error) {
            console.log('Error connecting to websocket server:', error);
            ros.isConnected = false;
        });

        ros.on('close', function() {
            console.log('Connection to websocket server closed.');
            ros.isConnected = false;
            if (retry) {
                console.log('Connection lost, attempting to reconnect...');
                setTimeout(() => rosConnect(ros, websocketsURL, retry), 1000); // Retry every second
            }
        });

        ros.hasListeners = true;
    }

    ros.connect(websocketsURL);
    return ros;
}

// Close the ROS2 connection
function disconnectRos(ros) {
    if (ros && ros.isConnected) {
        ros.close();
        console.log('Disconnected from ROS.');
        ros.isConnected = false; // Ensure the state is updated
    } else {
        console.log('No active ROS connection to disconnect.');
    }
}

function unsubscribeTopics(subscribedTopics) {
    subscribedTopics.forEach(topic => {
        topic.unsubscribe();
        console.log('Unsubscribed from ' + topic.name);
    });
    subscribedTopics = [];
}

// // Define the Websockets address of the ROS2 environment
// var websocketsURL = 'ws://' + window.location.hostname + ':9090';

// // Set up the connection to the ROS2 environment using the roslib.js library
// function rosConnect(ros, websocketsURL, retry=false){
//     if(ros) {
//         ros.connect(websocketsURL);
//     } else {
//         console.log('Creating new ros connection...');
//         var ros = new ROSLIB.Ros();
//         ros.connect(websocketsURL);
//     }
//     ros.on('connection', function(){
//         //console.log('Connected to websocket server.')
//     });
//     ros.on('error', function(){
//         //console.log('Error connecting to websocket server: ', error);
//     });
//     ros.on('close', function(){            
//         // Try to reconnect in 5 seconds
//         if (retry) {
//             //console.log('Connection to websocket server closed, trying in 5 seconds.');
//             setTimeout(function(){rosConnect(ros, websocketsURL)}, 5000);
//         }
//     });
// }