<template>
  <header class="header">
    <img src="../assets/Bob_only.png" height="64" />
    <div class="logo">BOB the Universal Object Tracker</div>
    <div class="title"></div>
  </header>
</template>

<script>
export default {
  name: 'Header',
}
</script>

<style>
.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0 20px;
  background-color: #333;
  color: white;
  height: 80px; /* Fixed height for the header */
}
.logo {
  font-size: 24px;
}
.title {
  font-size: 18px;
}
</style>
