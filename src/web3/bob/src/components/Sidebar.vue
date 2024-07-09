<template>
  <aside class="sidebar">
    <div class="menu">
      <div v-for="section in menuData" :key="section.title" class="menu-section">
        <div class="section-title">{{ section.title }}</div>
        <a v-for="item in section.items" :key="item.name" :href="item.link" class="menu-item">
          <i :class="['menu-icon', item.icon]"></i> {{ item.name }}
        </a>
      </div>
    </div>
  </aside>
</template>

<script>
export default {
  name: 'Sidebar',
  props: {
    menuData: {
      type: Array,
      required: true
    }
  },
  data() {
    return {
    };
  },
};
</script>

<style>
.sidebar {
  background-color: #eaeaea;
  padding: 20px;
  overflow-y: auto;
  border-color: rgb(222, 226, 230);
  border-width: 1;
}
.menu {
    width: 200px;
  }
  .menu-item {
    display: flex;
    align-items: center;
    padding: 10px;
    text-decoration: none;
    color: #007bff;
  }
  .menu-item:hover {
    background-color: #f8f9fa;
  }
  .menu-icon {
    margin-right: 10px;
  }
  .section-title {
    padding: 10px;
    font-weight: bold;
    color: #6c757d;
  }
</style>
