<template>
  <main id="main" class="default-theme">
    <splitpanes vertical class="default-theme">
      <pane size="80" class="default-theme">
        <div style="height: 100%;">
            <img ref="imageDisplayWindow" id="imageDisplayWindow" style="width: 100%"/>
        </div>
      </pane>
      <pane size="20" class="default-theme">
        <Features class="default-theme" />
      </pane>
    </splitpanes>
  </main>
</template>

<script>
import config from '@/config';
import Features from '../components/Features.vue';
import { Splitpanes, Pane } from 'splitpanes'
import 'splitpanes/dist/splitpanes.css'

export default {
  name: 'Annotated View',
  data() {
    return {
      bob: null,
    };
  },
  components: {
    Features,
    Splitpanes,
    Pane
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

<style scoped>
#main {
    height: 100%;
}
.splitpanes__pane {
  display: flex;
  justify-content: center;
  align-items: top;
  box-shadow: 0 0 3px rgba(0, 0, 0, .2) inset;
}

.features {
  flex: 1;
  overflow-y: auto;
  height: auto;
  width: 200px;
}
</style>
