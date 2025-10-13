import { type StateCreator } from 'zustand'
import { type SystemHealth, type Metrics } from '../services/schema'
import { apiClient } from '../services/api'

export interface SystemSlice {
  // State
  health: SystemHealth | null
  systemHealth: SystemHealth | null // Alias for compatibility
  metrics: Metrics[]
  versions: { ui: string; backend: string } | null
  version: string // Computed from versions
  isRunning: boolean
  wsStatus: 'connecting' | 'connected' | 'disconnected' | 'error'
  backendStatus: 'unknown' | 'online' | 'offline' | 'connecting' | 'disconnected'
  mockMode: boolean
  uiPreferences: {
    darkMode: boolean
    sidebarCollapsed: boolean
  }
  featureFlags: {
    recordings: boolean
    systemMetrics: boolean
    logs: boolean
  }

  // Actions
  setHealth: (health: SystemHealth | null) => void
  addMetrics: (metrics: Metrics) => void
  setVersions: (versions: { ui: string; backend: string }) => void
  setRunning: (running: boolean) => void
  setWSStatus: (status: 'connecting' | 'connected' | 'disconnected' | 'error') => void
  setBackendStatus: (status: 'unknown' | 'online' | 'offline' | 'connecting' | 'disconnected') => void
  setMockMode: (enabled: boolean) => void
  setFeatureFlag: (flag: keyof SystemSlice['featureFlags'], enabled: boolean) => void
  toggleDarkMode: () => void
  toggleSidebar: () => void
  fetchSystemHealth: () => Promise<void>
  fetchMetrics: () => Promise<void>
}

export const createSystemSlice: StateCreator<SystemSlice> = (set, get) => ({
  // Initial state
  health: null,
  systemHealth: null,
  metrics: [],
  versions: null,
  version: 'v1.0.0',
  isRunning: false,
  wsStatus: 'disconnected',
  backendStatus: 'unknown',
  mockMode: import.meta.env.VITE_MOCK_MODE === 'true',
  uiPreferences: {
    darkMode: false,
    sidebarCollapsed: false,
  },
  featureFlags: {
    recordings: import.meta.env.VITE_ENABLE_RECORDINGS !== 'false',
    systemMetrics: import.meta.env.VITE_ENABLE_SYSTEM_METRICS !== 'false',
    logs: import.meta.env.VITE_ENABLE_LOGS !== 'false',
  },

  // Actions
  setHealth: (health) => set({ health, systemHealth: health }),

  addMetrics: (metrics) => {
    const current = get().metrics
    const updated = [...current, metrics].slice(-100) // Keep last 100 data points
    set({ metrics: updated })
  },

  setVersions: (versions) => set({ 
    versions, 
    version: versions ? `v${versions.ui}` : 'v1.0.0' 
  }),

  setRunning: (running) => set({ isRunning: running }),

  setWSStatus: (status) => set({ wsStatus: status }),

  setBackendStatus: (status) => set({ backendStatus: status }),

  setMockMode: (enabled) => set({ mockMode: enabled }),

  setFeatureFlag: (flag, enabled) =>
    set((state) => ({
      featureFlags: {
        ...state.featureFlags,
        [flag]: enabled,
      },
    })),

  toggleDarkMode: () =>
    set((state) => ({
      uiPreferences: {
        ...state.uiPreferences,
        darkMode: !state.uiPreferences.darkMode,
      },
    })),

  toggleSidebar: () =>
    set((state) => ({
      uiPreferences: {
        ...state.uiPreferences,
        sidebarCollapsed: !state.uiPreferences.sidebarCollapsed,
      },
    })),

  fetchSystemHealth: async () => {
    try {
      const health = await apiClient.getSystemHealth()
      set({ health, systemHealth: health, backendStatus: 'online' })
    } catch (error) {
      console.error('Failed to fetch system health:', error)
      set({ backendStatus: 'offline' })
    }
  },

  fetchMetrics: async () => {
    try {
      const metrics = await apiClient.getMetrics()
      get().addMetrics(metrics[metrics.length - 1])
    } catch (error) {
      console.error('Failed to fetch metrics:', error)
    }
  },
})