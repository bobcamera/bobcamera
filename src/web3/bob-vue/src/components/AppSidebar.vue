<template>
  <v-navigation-drawer v-model="drawerVisible" @update:visible="$emit('update:visible', $event)" temporary>
     <!-- expand-on-hover rail> -->
    <div class="content">
      <v-list>
        <v-list-item subtitle="uapbob@gmailcom" title="BOB Camera"></v-list-item>
      </v-list>
      <v-divider></v-divider>
      <v-list color="primary" variant="plain" class="content">
        <div v-for="section in menuData" :key="section.title">
          <v-list-subheader>{{ section.title }}</v-list-subheader>
          <v-list-item v-for="item in section.items" :key="item.name" :prepend-icon="item.icon" :title="item.name"
            @click="handleClick(item)" :class="{ 'external-link': item.external }"></v-list-item>
        </div>
      </v-list>
      <div class="background-container">
        <div class="gradient-overlay"></div>
        <v-img src="../assets/bob_logo.png" class="background-image"></v-img>
      </div>
    </div>
  </v-navigation-drawer>
</template>

<script>
export default {
  props: {
    menuData: {
      type: Array,
      required: false
    },
    visible: {
      type: Boolean,
      default: true
    }
  },
  data() {
    return {
      drawerVisible: this.visible
    };
  },
  methods: {
    handleClick(item) {
      if (item.external) {
        window.open(item.link, '_blank');
      } else {
        this.$router.push(item.link);
      }
    }
  },
  watch: {
    visible(newVal) {
      this.drawerVisible = newVal;
    }
  }
};
</script>

<style scoped lang="scss">
.v-navigation-drawer {
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  position: relative;
  overflow: hidden;
}

.content {
  z-index: 2; /* Ensure content stays above the gradient and image */
}

.background-container {
  position: absolute;
  bottom: 0;
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: flex-end;
  z-index: 1;
}

.background-image {
  width: 100%;
  height: auto;
  opacity: 0.03; 
  position: absolute;
  bottom: 0;
}

.gradient-overlay {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: linear-gradient(to bottom, rgba(255, 255, 255, 0.05), rgba(255, 255, 255, 0.025), rgba(0, 0, 0, 0.0)); /* Gradient from top to bottom */
}
</style>
