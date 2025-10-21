import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'
import { getGitCommit } from './src/lib/buildInfo'

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  define: {
    'import.meta.env.VITE_GIT_HASH': JSON.stringify(getGitCommit()),
  },
  build: {
    rollupOptions: {
      external: ['roslib'],
    },
  },
  server: {
    port: 5173,
    proxy: {
      '/api':    { target: 'http://localhost:8080', changeOrigin: true },
      '/stream': { target: 'http://localhost:8080', changeOrigin: true },
      '/ws': {
        target: 'ws://localhost:8080',
        ws: true,
        changeOrigin: true,
      },
    },
  },
})
