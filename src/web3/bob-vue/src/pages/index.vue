<template>
  <FrameDisplay :cameraTopic="cameraTopic" :infoTopic="cameraInfo" />
</template>

<script>
import config from '@/config';

export default {
  name: 'Annotated View',
  data() {
    return {
      cameraTopic: config.annotatedTopic.topic, 
      cameraInfo: config.annotatedTopic.info
    };
  },
}
</script>

<style scoped lang="scss">
</style>
