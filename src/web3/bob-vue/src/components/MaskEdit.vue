<template>
  <div style="width: 100%; height: 100%;">
    <FrameDisplay viewName="Detection Mask Edit" :cameraTopic="maskTopic" :displayOverlay=false />
  </div>
</template>
  
<script>
  import config from '@/config';
  
  export default {
    name: 'Mask Edit',
    data() {
      return {
        bob: null,
      };
    },
    props: {
      maskTopic: {
        type: String,
        required: true
      }
    },
    mounted() {
      // this.bob = new BobRos(config.websocketsUrl);
      // this.bob.connect(true);
      // this.bob.subscribeTopic(this.cameraTopic, 'sensor_msgs/msg/CompressedImage',
      //   (message) => {
      //     try {
      //       //console.log(message)
      //       const imageElement = this.$refs.imageDisplayWindow;
      //       imageElement.src = "data:image/jpeg;base64," + message.data;
      //       this.compressedSize = { width: imageElement.naturalWidth, height: imageElement.naturalHeight }
      //       this.calculateFPS();
      //     } catch (error) {
      //       // Ignore, lost connection
      //     }
      //   });
    },
    beforeUnmount() {
      this.bob && this.bob.disconnect();
    },
    methods: {
    },
  };
</script>
  
<style scoped lang="scss">
</style>
  