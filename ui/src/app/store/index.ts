import { create } from 'zustand'
import { devtools } from 'zustand/middleware'
import { type SystemSlice, createSystemSlice } from './systemSlice'
import { type CamerasSlice, createCamerasSlice } from './camerasSlice'
import { type TracksSlice, createTracksSlice } from './tracksSlice'
import { type SettingsSlice, createSettingsSlice } from './settingsSlice'

export type AppStore = SystemSlice & CamerasSlice & TracksSlice & SettingsSlice

export const useAppStore = create<AppStore>()(
  devtools(
    (...args) => ({
      ...createSystemSlice(...args),
      ...createCamerasSlice(...args),
      ...createTracksSlice(...args),
      ...createSettingsSlice(...args),
    }),
    { name: 'BOB Camera Store' }
  )
)

// Export individual slices for testing
export * from './systemSlice'
export * from './camerasSlice'
export * from './tracksSlice'
export * from './settingsSlice'