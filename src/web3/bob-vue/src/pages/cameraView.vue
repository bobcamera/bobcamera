<template>
  <FrameDisplay viewName="Camera View" :cameraTopic="cameraTopic" :infoTopic="cameraInfo" :displayOverlay=true /> 
</template>

<script>
import config from '@/config';

export default {
  name: 'Camera View',
  data() {
    return {
      cameraTopic: config.plainCameraTopic.topic, 
      cameraInfo: config.plainCameraTopic.info
    };
  },
}
</script>

<style scoped lang="scss">
</style>
