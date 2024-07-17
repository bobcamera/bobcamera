<template>
  <div>
    <div>
      <img ref="imageDisplayWindow" id="imageDisplayWindow" class="imgDisplay" style="max-width: 100%;
      max-height: 100%;
      object-fit: contain;" />
    </div>
    <!-- <Features class="features" /> -->
  </div>
</template>

<script>
import config from '@/config';
// import Features from '../components/Features.vue';

export default {
  name: 'Annotated View',
  data() {
    return {
      bob: null,
    };
  },
  mounted() {
    this.bob = new BobRos(config.websocketsUrl);
    this.bob.connect(true);
    this.bob.subscribeTopic(config.bgsTopic.topic, config.bgsTopic.type,
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

