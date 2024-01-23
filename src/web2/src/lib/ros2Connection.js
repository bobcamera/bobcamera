// Connecting to the ROS2 environment using roslib.js library
// Roslib.js uses Rosbridge Server to communicate with the ROS2 environment over websockets
var ros = new ROSLIB.Ros({url: 'ws://localhost:9090'});
ros.on('connection', function () {console.log('Connected to websocket server.');});
ros.on('error', function (error) {console.log('Error connecting to websocket server: ', error);});
ros.on('close', function () {console.log('Connection to websocket server closed.');});