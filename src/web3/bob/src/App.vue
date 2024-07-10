<template>
  <div id="app" class="default-theme">
    <Header class="default-theme" />
    <splitpanes horizontal class="default-theme">
      <pane size="80" class="default-theme">
        <Sidebar class="default-theme" :menuData="menuData" />
        <MainContent class="default-theme" />
        <Features class="default-theme" />
      </pane>
      <pane size="20" class="default-theme">
        <Footer class="default-theme" />
      </pane>
    </splitpanes>
  </div>
</template>

<script>
import { RouterLink, RouterView } from 'vue-router'

import Header from './components/Header.vue';
import Sidebar from './components/Sidebar.vue';
import MainContent from './components/MainContent.vue';
import Features from './components/Features.vue';
import Footer from './components/Footer.vue';
import { Splitpanes, Pane } from 'splitpanes'
import 'splitpanes/dist/splitpanes.css'

export default {
  name: 'App',
  components: {
    Header,
    Sidebar,
    MainContent,
    Features,
    Footer,
    Splitpanes,
    Pane
  },
  data() {
    return {
      menuData: [
        {
          title: "STREAMING",
          items: [
            { name: "Info Panel", link: "/", icon: "bi-camera" },
            { name: "Wide View", link: "#", icon: "bi-camera" },
            { name: "Foreground Mask", link: "/bgs", icon: "bi-camera" }
          ]
        },
        {
          title: "MASK CONTROL",
          items: [
            { name: "Privacy Mask", link: "#", icon: "bi-mask" },
            { name: "Detection Mask", link: "#", icon: "bi-mask" }
          ]
        },
        {
          title: "RECORDINGS",
          items: [
            { name: "Recording Navigation", link: "#", icon: "bi-camera-video" }
          ]
        },
        {
          title: "SUPPORT",
          items: [
            { name: "Youtube", link: "#", icon: "bi-youtube" },
            { name: "Facebook", link: "#", icon: "bi-facebook" }
          ]
        },
        {
          title: "GITHUB",
          items: [
            { name: "Code", link: "#", icon: "bi-github" },
            { name: "Report Bug", link: "#", icon: "bi-github" },
            { name: "Request Feature", link: "#", icon: "bi-github" }
          ]
        }
      ]
    };
  }
}
</script>

<style scoped>
#app {
  display: flex;
  flex-direction: column;
  height: 100vh;
  width: 100vw;
  margin: 0;
  overflow: hidden;
}

.splitpanes__pane {
  display: flex;
  justify-content: center;
  align-items: top;
  box-shadow: 0 0 3px rgba(0, 0, 0, .2) inset;
}

.header {
  flex-shrink: 0;
}

.footer {
  width: 100%;
  height: 100%; /* Fixed height for the footer */
}

.sidebar, .main-content, .features {
  flex: 1;
  overflow-y: auto;
}

.sidebar {
  width: 200px;
  min-width: 200px;
  overflow: hidden;
}

.features {
  width: 200px;
}

.main-content {
  flex: 4;
  width: 100%;
}
</style>
