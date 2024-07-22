<template>
  <FrameDisplay viewName="Foreground View" :cameraTopic="cameraTopic" :infoTopic="cameraInfo" :displayOverlay=true />
</template>

<script>
import config from '@/config';

export default {
  name: 'BGS View',
  data() {
    return {
      cameraTopic: config.bgsTopic.topic, 
      cameraInfo: config.bgsTopic.info
    };
  },
}
</script>

<style scoped lang="scss">
</style>