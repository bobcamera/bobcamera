import { type StateCreator } from 'zustand'
import { type Camera } from '../services/schema'
import { apiClient } from '../services/api'

export interface CamerasSlice {
  // State
  cameras: Camera[]
  selectedCameraId: string | null
  loading: boolean
  error: string | null

  // Actions
  setCameras: (cameras: Camera[]) => void
  addCamera: (camera: Camera) => void
  updateCamera: (id: string, updates: Partial<Camera>) => Promise<void>
  removeCamera: (id: string) => void
  selectCamera: (id: string | null) => void
  setLoading: (loading: boolean) => void
  setError: (error: string | null) => void
  fetchCameras: () => Promise<void>
  createCamera: (camera: Omit<Camera, 'id' | 'status'>) => Promise<void>
  deleteCamera: (id: string) => Promise<void>
  testCamera: (id: string) => Promise<boolean>
  setSelectedCamera: (id: string | null) => void

  // Computed
  getCamera: (id: string) => Camera | undefined
  getSelectedCamera: () => Camera | undefined
}

export const createCamerasSlice: StateCreator<CamerasSlice> = (set, get) => ({
  // Initial state
  cameras: [],
  selectedCameraId: null,
  loading: false,
  error: null,

  // Actions
  setCameras: (cameras) => set({ cameras, error: null }),

  addCamera: (camera) =>
    set((state) => ({
      cameras: [...state.cameras, camera],
    })),

  updateCamera: async (id, updates) => {
    try {
      const updatedCamera = await apiClient.updateCamera(id, updates)
      set((state) => ({
        cameras: state.cameras.map((camera) =>
          camera.id === id ? updatedCamera : camera
        ),
      }))
    } catch (error) {
      throw new Error('Failed to update camera')
    }
  },

  removeCamera: (id) =>
    set((state) => ({
      cameras: state.cameras.filter((camera) => camera.id !== id),
      selectedCameraId: state.selectedCameraId === id ? null : state.selectedCameraId,
    })),

  selectCamera: (id) => set({ selectedCameraId: id }),

  setLoading: (loading) => set({ loading }),

  setError: (error) => set({ error }),

  fetchCameras: async () => {
    set({ loading: true, error: null })
    try {
      const cameras = await apiClient.getCameras()
      set({ cameras, loading: false })
    } catch (error) {
      set({ error: 'Failed to fetch cameras', loading: false })
    }
  },

  createCamera: async (camera) => {
    try {
      const newCamera = await apiClient.createCamera(camera as any)
      get().addCamera(newCamera)
    } catch (error) {
      throw new Error('Failed to create camera')
    }
  },

  deleteCamera: async (id) => {
    try {
      await apiClient.deleteCamera(id)
      get().removeCamera(id)
    } catch (error) {
      throw new Error('Failed to delete camera')
    }
  },

  testCamera: async (id) => {
    try {
      await apiClient.testCamera(id)
      return true
    } catch (error) {
      return false
    }
  },

  setSelectedCamera: (id) => set({ selectedCameraId: id }),

  // Computed
  getCamera: (id) => get().cameras.find((camera) => camera.id === id),

  getSelectedCamera: () => {
    const { cameras, selectedCameraId } = get()
    return cameras.find((camera) => camera.id === selectedCameraId)
  },
})