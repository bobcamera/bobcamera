<template>
  <v-app>
    <AppHeader @toggle-sidebar="toggleSidebar" />
    <v-main>
      <AppSidebar v-model:visible="sidebarVisible" :menuData="menuData" />
      <router-view />
    </v-main>
    <AppFooter />
  </v-app>
</template>

<script>
export default {
  data() {
    return {
      sidebarVisible: false,
      menuData: [
        {
          title: "STREAMING",
          items: [
            { name: "Tracking Feed", link: "/", external: false, icon: "mdi-video" },
            { name: "Foreground Mask", link: "/bgs", external: false, icon: "mdi-video-image" }
          ]
        },
        {
          title: "MASK CONTROL",
          items: [
            { name: "Privacy Mask", link: "#", external: false, icon: "mdi-drama-masks" },
            { name: "Detection Mask", link: "#", external: false, icon: "mdi-transition-masked" }
          ]
        },
        {
          title: "RECORDINGS",
          items: [
            { name: "Recording Navigation", link: "#", external: false, icon: "mdi-record-rec" }
          ]
        },
        {
          title: "SUPPORT",
          items: [
            { name: "Youtube", link: "https://www.youtube.com/@BobUniversalObjectTracker", external: true, icon: "mdi-youtube" },
            { name: "Facebook", link: "https://www.facebook.com/groups/7533264616717176", external: true, icon: "mdi-facebook" }
          ]
        },
        {
          title: "GITHUB",
          items: [
            { name: "Code", link: "https://github.com/bobcamera/bobcamera", external: true, icon: "mdi-github" },
            { name: "Report Bug/Feature", link: "https://github.com/bobcamera/bobcamera/issues/new/choose", external: true, icon: "mdi-github" },
          ]
        },
        {
          title: "CONFIGURATION",
          items: [
            { name: "Configuration", link: "#", external: false, icon: "mdi-account-settings" },
          ]
        }
      ]
    }
  },
  methods: {
    toggleSidebar() {
      this.sidebarVisible = !this.sidebarVisible
    }
  }
}
</script>

<!-- expand-on-hover
rail -->
