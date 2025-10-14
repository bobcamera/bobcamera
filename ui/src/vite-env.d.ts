/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_API_BASE_URL?: string
  readonly VITE_MOCK_MODE?: string
  readonly VITE_ENABLE_RECORDINGS?: string
  readonly VITE_ENABLE_SYSTEM_METRICS?: string
  readonly VITE_ENABLE_LOGS?: string
  readonly VITE_GIT_HASH?: string
}

interface ImportMeta {
  readonly env: ImportMetaEnv
}
