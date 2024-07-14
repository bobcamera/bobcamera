<template>
  <main class="main-content">
    <Splitter :gutterSize="8">
      <SplitterPanel class="flex items-center justify-center" :size="80">
        <div class="imgDisplayDiv">
            <img ref="imageDisplayWindow" id="imageDisplayWindow" class="imgDisplay" />
        </div>
      </SplitterPanel>
      <SplitterPanel class="flex items-center justify-center features" :size="20">
        <Features class="features" />
      </SplitterPanel>
    </Splitter>
  </main>
</template>

<script>
import config from '@/config';
import Features from '../components/Features.vue';
import Splitter from 'primevue/splitter';
import SplitterPanel from 'primevue/splitterpanel';

export default {
  name: 'Annotated View',
  data() {
    return {
      bob: null,
    };
  },
  components: {
    Features,
    Splitter,
    SplitterPanel
  },
  mounted() {
    this.bob = new BobRos(config.websocketsUrl);
    this.bob.connect(true);
    this.bob.subscribeTopic('bob/frames/annotated/resized/compressed', 'sensor_msgs/msg/CompressedImage',
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
