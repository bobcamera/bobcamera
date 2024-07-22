<template>
  <MaskEdit :maskTopic="maskBGSTopic" />
</template>

<script>
import config from '@/config';

export default {
  name: 'Mask BGS View',
  data() {
    return {
      maskBGSTopic: config.plainCameraTopic.topic, 
    };
  },
}
</script>

<style scoped lang="scss">
</style>