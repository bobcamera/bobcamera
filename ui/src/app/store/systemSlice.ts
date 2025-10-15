import { type StateCreator } from 'zustand'
import { type SystemHealth, type Metrics } from '../services/schema'
import { apiClient } from '../services/api'
import { mockDetectionGenerator } from '../../lib/mock/mockDetections'
import type { Detection } from '../services/schema'

// Adapter to convert simple mock detections to the advanced UI format
function adaptMockDetection(mockDetection: any): Detection {
  return {
    id: mockDetection.id,
    cameraId: 'mock-camera-1',
    timestamp: new Date(mockDetection.timestamp).toISOString(),
    bbox: {
      x: mockDetection.x,
      y: mockDetection.y,
      width: mockDetection.width,
      height: mockDetection.height,
    },
    class: mockDetection.class,
    confidence: mockDetection.confidence,
    snapshotUrl: undefined,
    metadata: {
      mock: true,
    },
  }
}

export interface SystemSlice {
  // State
  health: SystemHealth | null
  systemHealth: SystemHealth | null // Alias for compatibility
  metrics: Metrics[]
  versions: { ui: string; backend: string } | null
  version: string // Computed from versions (deprecated, use versions instead)
  gitHash: string | null
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
  setGitHash: (hash: string) => void
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
  // Initial state for SystemSlice
  health: null,
  systemHealth: null,
  metrics: [],
  versions: { ui: '0.9.0', backend: '1.7.5' },
  version: 'v0.9.0',
  gitHash: import.meta.env.VITE_GIT_HASH || null,
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
  
  // Initial state for TracksSlice (required to satisfy SystemSlice & TracksSlice type)
  tracks: [],
  detections: [],
  liveEvents: [],
  selectedTrackId: null,
  filters: {},
  pagination: {
    page: 1,
    pageSize: 50,
    total: 0,
    hasMore: false,
  },
  loading: false,
  error: null,

  // SystemSlice actions
  setHealth: (health) => set({ health, systemHealth: health }),

  addMetrics: (metrics) => {
    const current = get().metrics
    const updated = [...current, metrics].slice(-100)
    set({ metrics: updated })
  },

  setVersions: (versions) => set({
    versions,
    version: versions ? `v${versions.ui}` : 'v0.9.0'
  }),

  setGitHash: (hash) => set({ gitHash: hash }),

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
      set({ health, systemHealth: health, backendStatus: 'online', mockMode: false })
      mockDetectionGenerator.stop()
    } catch (error) {
      console.error('Failed to fetch system health:', error)
      const currentMockMode = get().mockMode
      set({ backendStatus: 'offline', mockMode: true })

      if (!currentMockMode) {
        console.log('[SystemSlice] Starting mock mode...')
        mockDetectionGenerator.start((detections) => {
          console.log('[SystemSlice] Mock detections generated:', detections.length)
          detections.forEach((mockDetection) => {
            const state = get() as any
            if (state.addLiveEvent) {
              const adaptedDetection = adaptMockDetection(mockDetection)
              state.addLiveEvent(adaptedDetection)
            }
          })
        })
      }
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
