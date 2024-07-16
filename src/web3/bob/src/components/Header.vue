<template>
    <!-- <v-app-bar :elevation="2">
      <template v-slot:prepend> -->
        <v-app-bar-nav-icon></v-app-bar-nav-icon>
      <!-- </template>

      <v-app-bar-title>BOB the Universal Object Tracker</v-app-bar-title>
    </v-app-bar> -->
    <div>
    <img src="../assets/bob_logo.png" height="64" />
    <div class="logo">BOB the Universal Object Tracker</div>
    <div class="title"></div>
    <ThemeSwitcher />
  </div>
</template>

<script setup>
import ThemeSwitcher from './ThemeSwitcher.vue';
</script>
