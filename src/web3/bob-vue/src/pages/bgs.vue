<template>
  <FrameDisplay :cameraTopic="cameraTopic" :infoTopic="cameraInfo" />
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