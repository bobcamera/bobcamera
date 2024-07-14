import './assets/bob.css'

import { createApp } from 'vue'
import { createPinia } from 'pinia'

import PrimeVue from 'primevue/config'
import Aura from '@primevue/themes/aura';
import Lara from '@primevue/themes/lara';
import Nora from '@primevue/themes/nora';
import { definePreset } from '@primevue/themes';
import App from './App.vue'
import router from './router'

const app = createApp(App)

app.use(createPinia())
app.use(router)

const BobPreset = definePreset(Aura, {
    semantic: {
        primary: {
            50: '{neutral.50}',
            100: '{neutral.100}',
            200: '{neutral.200}',
            300: '{neutral.300}',
            400: '{neutral.400}',
            500: '{neutral.500}',
            600: '{neutral.600}',
            700: '{neutral.700}',
            800: '{neutral.800}',
            900: '{neutral.900}',
            950: '{neutral.950}'
        },
    },
    colorScheme: {
        light: {
            surface: {
                0: '#ffffff',
                50: '{neutral.50}',
                100: '{neutral.100}',
                200: '{neutral.200}',
                300: '{neutral.300}',
                400: '{neutral.400}',
                500: '{neutral.500}',
                600: '{neutral.600}',
                700: '{neutral.700}',
                800: '{neutral.800}',
                900: '{neutral.900}',
                950: '{neutral.950}'
            },
            primary: {
                color: '{neutral.50}',
                inverseColor: '{neutral.950}',
                hoverColor: '{neutral.100}',
                activeColor: '{neutral.200}'
            },
            highlight: {
                background: 'rgba(250, 250, 250, .16)',
                focusBackground: 'rgba(250, 250, 250, .24)',
                color: 'rgba(255,255,255,.87)',
                focusColor: 'rgba(255,255,255,.87)'
            },
        },
        dark: {
            surface: {
                0: '#000000',
                50: '{neutral.950}',
                100: '{neutral.900}',
                200: '{neutral.800}',
                300: '{neutral.700}',
                400: '{neutral.600}',
                500: '{neutral.500}',
                600: '{neutral.400}',
                700: '{neutral.300}',
                800: '{neutral.200}',
                900: '{neutral.100}',
                950: '{neutral.50}'
            },
            primary: {
                color: '{neutral.950}',
                inverseColor: '#ffffff',
                hoverColor: '{neutral.900}',
                activeColor: '{neutral.800}'
            },
            highlight: {
                background: '{neutral.950}',
                focusBackground: '{neutral.700}',
                color: '#ffffff',
                focusColor: '#ffffff'
            },
        },
    }
});

app.use(PrimeVue, {
    theme: {
        preset: Nora,
        options: {
            darkModeSelector: '.my-app-dark',
        }
    }
})

app.mount('#app')
