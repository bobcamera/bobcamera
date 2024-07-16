import './assets/bob.css'

import { createApp } from 'vue'
import { createPinia } from 'pinia'

import App from './App.vue'
import router from './router'

import 'vuetify/styles'
import { createVuetify } from 'vuetify'
import * as components from 'vuetify/components'
import * as directives from 'vuetify/directives'

const vuetify = createVuetify({
    components,
    directives,
    theme: {
      defaultTheme: 'light',
      //
    },
  })

const app = createApp(App)

app.use(createPinia())
  .use(vuetify)
  .use(router)
app.mount('#app')
