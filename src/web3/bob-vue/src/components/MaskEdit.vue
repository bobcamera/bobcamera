<template>
  <div>
    <div class="image-container">
      <img ref="imageDisplayWindow" id="imageDisplayWindow" class="background-image" />
    </div>
        <!-- <Features class="features" /> -->
  </div>
</template>
  
<script>
  import config from '@/config';
  // import Features from '../components/Features.vue';
  
  export default {
    name: 'Mask Edit',
    data() {
      return {
        bob: null,
        fps: 0,
        frameCount: 0,
        startTime: null,
        fpsText: '',
        frameSize: { width: 0, height: 0 },
        compressedSize: { width: 0, height: 0 },
        cameraFps: 0
      };
    },
    props: {
      cameraTopic: {
        type: String,
        required: true
      },
      infoTopic: {
        type: String,
        required: true
      },
    },
    mounted() {
      this.bob = new BobRos(config.websocketsUrl);
      this.bob.connect(true);
      this.bob.subscribeTopic(this.cameraTopic, 'sensor_msgs/msg/CompressedImage',
        (message) => {
          try {
            //console.log(message)
            const imageElement = this.$refs.imageDisplayWindow;
            imageElement.src = "data:image/jpeg;base64," + message.data;
            this.compressedSize = { width: imageElement.naturalWidth, height: imageElement.naturalHeight }
            this.calculateFPS();
          } catch (error) {
            // Ignore, lost connection
          }
        });
      this.bob.subscribeTopic(this.infoTopic, 'bob_camera/msg/CameraInfo',
        (message) => {
          try {
            //console.log(message)
            this.frameSize = { width: message.frame_width, height: message.frame_height }
            this.cameraFps = message.fps
          } catch (error) {
            // Ignore, lost connection
          }
        });
    },
    beforeUnmount() {
      this.bob.disconnect();
    },
    methods: {
    },
  };
</script>
  
<style scoped lang="scss">
  .image-container {
    position: relative;
    display: inline-block;
  }
  
  .background-image {
    display: block;
    max-width: 100%;
    max-height: 100%;
    object-fit: contain;
    // border: 1px solid red;
  }
</style>
  