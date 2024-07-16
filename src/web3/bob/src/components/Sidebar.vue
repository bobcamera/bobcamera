<!-- <template>
    <Drawer v-model:visible="drawerVisible" @update:visible="$emit('update:visible', $event)" header="Menu">
        <div class="menu">
            <div v-for="section in menuData" :key="section.title" class="menu-section">
                <div class="section-title">{{ section.title }}</div>
                <template v-for="item in section.items" :key="item.name">
                    <router-link v-if="!item.external" :to="item.link" class="menu-item">
                        <i :class="['menu-icon', item.icon]"></i> {{ item.name }}
                    </router-link>
                    <a v-else :href="item.link" class="menu-item" target="_blank" rel="noopener noreferrer">
                        <i :class="['menu-icon', item.icon]"></i> {{ item.name }}
                    </a>
                </template>
            </div>
        </div>
    </Drawer>
</template> -->
<template>
    <v-card>
      <v-layout style="z-index: 900; position: relative; overflow: hidden;">
        <v-navigation-drawer
          expand-on-hover
          rail
        >
          <v-list>
            <v-list-item
              prepend-avatar="https://randomuser.me/api/portraits/women/85.jpg"
              subtitle="sandra_a88@gmailcom"
              title="Sandra Adams"
            ></v-list-item>
          </v-list>
  
          <v-divider></v-divider>
  
          <v-list density="compact" nav>
            <v-list-item prepend-icon="mdi-folder" title="My Files" value="myfiles"></v-list-item>
            <v-list-item prepend-icon="mdi-account-multiple" title="Shared with me" value="shared"></v-list-item>
            <v-list-item prepend-icon="mdi-star" title="Starred" value="starred"></v-list-item>
          </v-list>
        </v-navigation-drawer>
  
        <v-main style="height: 250px"></v-main>
      </v-layout>
    </v-card>
  </template>

<script>
export default {
  name: 'Sidebar',
  components: {
  },
  props: {
    menuData: {
      type: Array,
      required: true
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
  watch: {
    visible(newVal) {
      this.drawerVisible = newVal;
    }
  }
};
</script>

<!-- <template>
    <v-card>
      <v-layout>
        <v-navigation-drawer
          expand-on-hover
          rail
        >
          <v-list>
            <v-list-item
              prepend-avatar="https://randomuser.me/api/portraits/women/85.jpg"
              subtitle="sandra_a88@gmailcom"
              title="Sandra Adams"
            ></v-list-item>
          </v-list>
  
          <v-divider></v-divider>
  
          <v-list density="compact" nav>
            <v-list-item prepend-icon="mdi-folder" title="My Files" value="myfiles"></v-list-item>
            <v-list-item prepend-icon="mdi-account-multiple" title="Shared with me" value="shared"></v-list-item>
            <v-list-item prepend-icon="mdi-star" title="Starred" value="starred"></v-list-item>
          </v-list>
        </v-navigation-drawer>
  
        <v-main style="height: 250px"></v-main>
      </v-layout>
    </v-card>
  </template> -->