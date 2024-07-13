<template>
  <main>
    <img ref="imageDisplayWindow" id="imageDisplayWindow" style="width: 100%"/>
  </main>
</template>

<script>
import config from '@/config';

export default {
  name: 'BGS View',
  data() {
    return {
      bob: null,
    };
  },
  mounted() {
    this.bob = new BobRos(config.websocketsUrl);
    this.bob.connect(true);
    this.bob.subscribeTopic('bob/frames/foreground_mask/resized/compressed', 'sensor_msgs/msg/CompressedImage',
        (message) => {
          try {
            const imageElement = this.$refs.imageDisplayWindow;
            imageElement.src = "data:image/jpeg;base64," + message.data;
          } catch (error) {
            // Ignore, lost connection
          }
        });
  },
  beforeRouteLeave(to, from, next) {
    this.bob.disconnect();
    next();
  },
  beforeDestroy() {
    this.bob.disconnect();
  },
  methods: {
  },
};
</script>