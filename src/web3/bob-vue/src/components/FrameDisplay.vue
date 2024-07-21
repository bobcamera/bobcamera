<template>
  <div ref="divImage" class="image-container">
    <img ref="imageDisplayWindow" id="imageDisplayWindow" class="background-image" />
    <div ref="overlayPanels" class="div-overlay-panels">
    <v-expansion-panels ref="overlayPanels" class="overlay-panels" v-show="displayOverlay">
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
              <v-row align="center" no-gutters v-show="infoTopic">
                <v-icon class="mr-2">mdi-video</v-icon>
                {{ cameraFps.toFixed(2) }} fps
                <v-spacer></v-spacer>
                <span class="detail">{{ frameSize.width }}x{{ frameSize.height }}</span>
              </v-row>
              <v-divider class="my-3" v-show="infoTopic"></v-divider>
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
  </div>
</template>

<script>
import config from '@/config';

export default {
  name: 'Frame View',
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
      default: null
    },
    displayOverlay: {
      type: Boolean,
      default: true
    }
  },
  mounted() {
    this.bob = new BobRos(config.websocketsUrl);
    this.bob.connect(true);
    this.bob.subscribeTopic(this.cameraTopic, 'sensor_msgs/msg/CompressedImage',
      (message) => {
        try {
          const imageElement = this.$refs.imageDisplayWindow;
          const divImage = this.$refs.divImage;
          const overlayPanels = this.$refs.overlayPanels;
          const ar = imageElement.naturalWidth / imageElement.naturalHeight;
          imageElement.src = "data:image/jpeg;base64," + message.data;
          if (imageElement.width) {
            overlayPanels.style.width = imageElement.width + "px"
            overlayPanels.style.height = (imageElement.width / ar) + "px"
            overlayPanels.style.top = ((divImage.clientHeight - (imageElement.width / ar)) / 2) + "px"
          }
          this.compressedSize = { width: imageElement.naturalWidth, height: imageElement.naturalHeight }
          this.calculateFPS()
        } catch (error) {
          // Ignore, lost connection
        }
      });
    if (this.infoTopic) {
      this.bob.subscribeTopic(this.infoTopic, 'bob_camera/msg/CameraInfo',
        (message) => {
          try {
            // console.log(message)
            this.frameSize = { width: message.frame_width, height: message.frame_height }
            this.cameraFps = message.fps
          } catch (error) {
            // Ignore, lost connection
          }
        });
    }
  },
  beforeUnmount() {
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
    },
    imageLoaded() {
      console.log(5)
    }
  },
};
</script>

<style scoped lang="scss">
.image-container {
  position: relative;
  width: 100%;
  height: 100%;
  display: inline-block;
}

.background-image {
  position: absolute;
  top: 0px;
  left: 0px;

  display: block;
  max-width: 100%;
  max-height: 100%;
  width: auto;
  height: 100%;
  object-fit: contain;
  // border: 1px solid red;
}

.div-overlay-panels {
  position: absolute;
  left: 0;
  top: 0;
}

.overlay-panels {
  // position: absolute;
  // top: 0px;
  // left: 0px;
  width: 100%;
  height: 100%;
}

.overlay-panel {
  position: absolute;
  text-align: center;
  top: 20px;
  right: 20px;
  z-index: 10;
  width: 250px;
  background-color: rgba(116, 116, 7, 0.65);
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