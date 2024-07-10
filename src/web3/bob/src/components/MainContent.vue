<template>
  <main class="main-content">
    <router-view></router-view>
  </main>
</template>

<script>
// export default {
//   name: 'MainContent',
// };

// var ros = new ROSLIB.Ros();
// rosConnect(ros, websocketsURL, true);

// var topicStream = new ROSLIB.Topic({
//     ros: ros,
//     name: '/bob/frames/annotated/resized/compressed',
//     messageType: 'sensor_msgs/msg/CompressedImage'
// });

// // Stream Messages Published From a ROS2 Topic (On each new message arrival, the function is called)
// topicStream.subscribe((message) => {
//     // Display Image In imageDisplayWindow element
//     console.log(message)
//     displayImage(message);
// });

// // Declare a function to display the image in the imageDisplayWindow element
// function displayImage(message) {
//     const imageElement = this.$refs.imageDisplayWindow;
//     imageElement.src = "data:image/jpeg;base64," + message.data;
//     console.log(imageElement)
// }

// window.onbeforeunload = function(event) {

//     console.log('Cleaning up resources...');

//     // Unsubscribe from the topic
//     if (topicStream) {
//         topicStream.unsubscribe();
//         console.log('Unsubscribed from topic.');
//     }

//     // Close the ROS2 connection
//     // ros.close();

//     // console.log('ROS connection closed.');

//     // Close the ROS2 connection
//     disconnectRos(ros);
// }
</script>

<style>
.main-content {
  padding: 20px;
  overflow-y: auto;
}
</style>
