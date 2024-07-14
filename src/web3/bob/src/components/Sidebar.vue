<template>
    <aside>
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