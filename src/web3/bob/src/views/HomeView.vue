<template>
  <main>
    <img ref="imageDisplayWindow" id="imageDisplayWindow" style="width: 100%"/>
  </main>
</template>

<script>
export default {
  name: 'Annotated View',
  data() {
    return {
      connected: false,
      message: null,
      ros: null,
      listener: null,
    };
  },
  mounted() {
    this.connectRosWebSocket();
  },
  beforeDestroy() {
    this.disconnectRosWebSocket();
  },
  methods: {
    connectRosWebSocket() {
      // Create a new ROS connection
      this.ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090',
      });

      // Handle connection open
      this.ros.on('connection', () => {
        this.connected = true;
        console.log('ROS connection opened');

        // Create a new ROS topic listener
        this.listener = new ROSLIB.Topic({
          ros: this.ros,
          name: 'bob/frames/annotated/resized/compressed',
          messageType: 'sensor_msgs/msg/CompressedImage',
        });

        // Subscribe to the topic
        this.listener.subscribe((message) => {
          this.message = message.data;
          const imageElement = this.$refs.imageDisplayWindow;
          imageElement.src = "data:image/jpeg;base64," + message.data;
          //console.log('Message received:', message.data);
        });
      });

      // Handle connection close
      this.ros.on('close', () => {
        this.connected = false;
        console.log('ROS connection closed');
      });

      // Handle errors
      this.ros.on('error', (error) => {
        console.error('ROS connection error:', error);
      });
    },
    disconnectRosWebSocket() {
      if (this.listener) {
        this.listener.unsubscribe();
      }
      if (this.ros) {
        this.ros.close();
      }
    },
  },
};
</script>