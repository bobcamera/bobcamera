<template>
  <v-app-bar :elevation="2">
    <template v-slot:prepend>
      <v-app-bar-nav-icon @click="$emit('toggle-sidebar')"></v-app-bar-nav-icon>
    </template>
    <v-app-bar-title></v-app-bar-title>
    <template v-slot:append>
      <img src="../assets/bob_logo_horizontal.png" height="52" style="padding: 5px 5px 0 0;" />
    </template>    
  </v-app-bar>
</template>

<script setup>
</script>

<style scoped lang="scss">
.v-app-bar-nav-icon {
  margin-left: 20px;
}
</style>