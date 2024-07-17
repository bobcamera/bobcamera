<template>
  <div>
    <div class="image-container">
      <img ref="imageDisplayWindow" id="imageDisplayWindow" class="background-image" />
      <v-expansion-panels class="overlay-panels">
        <v-expansion-panel class="overlay-panel">
          <v-expansion-panel-title>
            <v-row align="center" no-gutters>
              <v-icon class="mr-2">mdi-speedometer</v-icon>
              <span class="headline">{{ fps.toFixed(2) }} fps</span>
              <v-spacer></v-spacer>
              <span class="subheading">{{ compressedSize.width }}x{{ compressedSize.height }}</span>
            </v-row>
          </v-expansion-panel-title>
          <v-expansion-panel-text>
            <v-row>
              <v-col>
                <v-row align="center" no-gutters>
                  <v-icon class="mr-2">mdi-camera</v-icon>
                  {{ cameraFps.toFixed(2) }} fps
                  <v-spacer></v-spacer>
                  <span class="detail">{{ frameSize.width }}x{{ frameSize.height }}</span>
                </v-row>
                <v-divider class="my-3"></v-divider>
                <v-row align="center" no-gutters>
                  <v-icon class="mr-2">mdi-monitor</v-icon>
                  {{ fps.toFixed(2) }} fps
                  <v-spacer></v-spacer>
                  <span class="detail">{{ compressedSize.width }}x{{ compressedSize.height }}</span>
                </v-row>
              </v-col>
            </v-row>
          </v-expansion-panel-text>
        </v-expansion-panel>
      </v-expansion-panels>
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
      fps: 0,
      frameCount: 0,
      startTime: null,
      fpsText: '',
      frameSize: { width: 0, height: 0 },
      compressedSize: { width: 0, height: 0 },
      cameraFps: 0
    };
  },
  mounted() {
    this.bob = new BobRos(config.websocketsUrl);
    this.bob.connect(true);
    this.bob.subscribeTopic(config.annotatedTopic.topic, config.annotatedTopic.type,
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
    this.bob.subscribeTopic('bob/camera/all_sky/camera_info', 'bob_camera/msg/CameraInfo',
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
  beforeRouteLeave(to, from, next) {
    this.bob.disconnect();
    next();
  },
  beforeDestroy() {
    this.bob.disconnect();
  },
  methods: {
    calculateFPS() {
      const currentTime = Date.now();
      this.frameCount++;
      const elapsedTime = (currentTime - this.startTime) / 1000;

      if (elapsedTime >= 1) {
        this.fps = this.frameCount / elapsedTime;
        this.startTime = currentTime;
        this.frameCount = 0;
      }
    }
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

.overlay-panels {
  position: absolute;
  text-align: center;
  top: 20px;
  right: 20px;
  z-index: 10;
  width: 250px;
}

.overlay-panel {
  background-color: rgba(116, 116, 7, 0.65); /* Semi-transparent background */
  border-radius: 8px;
  margin: 10px 0;
}

.headline {
  font-weight: bold;
  color: #798bf4;
}

.subheading {
  color: #c2c2c2;
}

.title {
  font-weight: 500;
  color: #1e88e5;
}

.detail {
  color: #ffffff;
}

.v-icon {
  color: #6db4f1;
}
</style>
