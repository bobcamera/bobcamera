<template>
  <div id="bob-app" class="bob-app">
      <Header />
      <div class="p-splitterpanel">
        <!-- <Sidebar :menuData="menuData" v-model:visible="menuVisible" /> -->
        <!-- <Button icon="pi pi-arrow-right" @click="menuVisible = true" style="margin: 10px; background-color: var(--p-content-hover-background);" /> -->
        <MainContent class="main-content" />
      </div>
      <Footer class="footer" />
  </div>
</template>

<script>
import { RouterLink, RouterView } from 'vue-router'

import Header from './components/Header.vue';
import Sidebar from './components/Sidebar.vue';
import MainContent from './components/MainContent.vue';
import Footer from './components/Footer.vue';

export default {
  name: 'App',
  components: {
    Header,
    Sidebar,
    MainContent,
    Footer,
  },
  data() {
    return {
      menuVisible: false,
      menuData: [
        {
          title: "STREAMING",
          items: [
            { name: "Info Panel", link: "/", external: false, icon: "bi-camera" },
            { name: "Foreground Mask", link: "/bgs", external: false, icon: "bi-camera" }
          ]
        },
        {
          title: "MASK CONTROL",
          items: [
            { name: "Privacy Mask", link: "#", external: false, icon: "bi-mask" },
            { name: "Detection Mask", link: "#", external: false, icon: "bi-mask" }
          ]
        },
        {
          title: "RECORDINGS",
          items: [
            { name: "Recording Navigation", link: "#", external: false, icon: "bi-camera-video" }
          ]
        },
        {
          title: "SUPPORT",
          items: [
            { name: "Youtube", link: "https://www.youtube.com/@BobUniversalObjectTracker", external: true, icon: "bi-youtube" },
            { name: "Facebook", link: "https://www.facebook.com/groups/7533264616717176", external: true, icon: "bi-facebook" }
          ]
        },
        {
          title: "GITHUB",
          items: [
            { name: "Code", link: "https://github.com/bobcamera/bobcamera", external: true, icon: "bi-github" },
            { name: "Report Bug", link: "https://github.com/bobcamera/bobcamera/issues/new/choose", external: true, icon: "bi-github" },
            { name: "Request Feature", link: "https://github.com/bobcamera/bobcamera/issues/new/choose", external: true, icon: "bi-github" }
          ]
        }
      ]
    };
  }
}
</script>

