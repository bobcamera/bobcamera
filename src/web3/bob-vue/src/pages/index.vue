<template>
  <FrameDisplay viewName="Annotated View" :cameraTopic="cameraTopic" :infoTopic="cameraInfo" :displayOverlay=true /> 
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
