<template>
  <div>
      <Button label="Toggle Color Scheme" @click="toggleColorScheme()" />
  </div>
</template>
  
<script setup>
  import { ref } from 'vue';
  import Button from 'primevue/button';

  const toggleColorScheme = () => {
    const element = document.querySelector('html');
    element.classList.toggle('my-app-dark');
  }
</script>