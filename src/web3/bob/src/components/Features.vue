<template>
  <aside>
    <ul>
      <li v-for="feature in features" :key="feature">{{ feature }}</li>
    </ul>
  </aside>
</template>

<script>
export default {
  name: 'Features',
  data() {
    return {
      features: [
        'Feature 1', 'Feature 2', 'Feature 3', 'Feature 4', 'Feature 5', 'Feature 6'
      ],
    };
  },
};
</script>