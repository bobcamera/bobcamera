import { type StateCreator } from 'zustand'
import { type Config } from '../services/schema'
import { apiClient } from '../services/api'

export interface SettingsSlice {
  // State
  config: Config | null
  draftConfig: Config | null
  saveStatus: 'idle' | 'saving' | 'success' | 'error'
  error: string | null

  // Actions
  setConfig: (config: Config) => void
  setDraftConfig: (config: Config | null) => void
  updateDraftConfig: (updates: Partial<Config>) => void
  setSaveStatus: (status: SettingsSlice['saveStatus']) => void
  setError: (error: string | null) => void
  resetDraft: () => void
  fetchConfig: () => Promise<void>
  saveConfig: (config: Config) => Promise<void>
  resetDraftConfig: () => void
  hasPendingChanges: () => boolean

  // Computed
  hasChanges: () => boolean
  getDiff: () => any
}

export const createSettingsSlice: StateCreator<SettingsSlice> = (set, get) => ({
  // Initial state
  config: null,
  draftConfig: null,
  saveStatus: 'idle',
  error: null,

  // Actions
  setConfig: (config) => set({ config, draftConfig: null, error: null }),

  setDraftConfig: (config) => set({ draftConfig: config }),

  updateDraftConfig: (updates) =>
    set((state) => ({
      draftConfig: state.draftConfig ? { ...state.draftConfig, ...updates } : null,
    })),

  setSaveStatus: (status) => set({ saveStatus: status }),

  setError: (error) => set({ error }),

  resetDraft: () => set({ draftConfig: null }),

  fetchConfig: async () => {
    try {
      const config = await apiClient.getConfig()
      set({ config, draftConfig: config })
    } catch (error) {
      set({ error: 'Failed to fetch config' })
    }
  },

  saveConfig: async (config) => {
    set({ saveStatus: 'saving' })
    try {
      await apiClient.updateConfig(config)
      set({ config, draftConfig: null, saveStatus: 'success' })
      setTimeout(() => set({ saveStatus: 'idle' }), 2000)
    } catch (error) {
      set({ saveStatus: 'error', error: 'Failed to save config' })
    }
  },

  resetDraftConfig: () => {
    const { config } = get()
    set({ draftConfig: config })
  },

  hasPendingChanges: () => {
    const { config, draftConfig } = get()
    return config !== null && draftConfig !== null && JSON.stringify(config) !== JSON.stringify(draftConfig)
  },

  // Computed
  hasChanges: () => {
    const { config, draftConfig } = get()
    return config !== null && draftConfig !== null && config !== draftConfig
  },

  getDiff: () => {
    const { config, draftConfig } = get()
    if (!config || !draftConfig) return null

    const diff: any = {}
    const keys = new Set([...Object.keys(config), ...Object.keys(draftConfig)])

    keys.forEach((key) => {
      const configValue = (config as any)[key]
      const draftValue = (draftConfig as any)[key]

      if (JSON.stringify(configValue) !== JSON.stringify(draftValue)) {
        diff[key] = {
          old: configValue,
          new: draftValue,
        }
      }
    })

    return diff
  },
})