<template>
  <v-navigation-drawer v-model="drawerVisible" @update:visible="$emit('update:visible', $event)" temporary>
    <v-list>
      <v-list-item subtitle="bob@gmailcom" title="BOB Camera"></v-list-item>
    </v-list>
    <v-divider></v-divider>
    <v-list color="primary" variant="plain">
      <div v-for="section in menuData" :key="section.title">
        <v-list-subheader>{{ section.title }}</v-list-subheader>
        <v-list-item v-for="item in section.items" :key="item.name" :prepend-icon="item.icon" :title="item.name" 
          @click="handleClick(item)" :class="{ 'external-link': item.external }"></v-list-item>
      </div>
    </v-list>
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


<style scoped lang="sass">
</style>

<!-- :title="item.name" :value="item.name" -->