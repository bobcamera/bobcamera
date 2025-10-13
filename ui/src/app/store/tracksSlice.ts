import { type StateCreator } from 'zustand'
import { type Track, type Detection } from '../services/schema'

export interface TracksSlice {
  // State
  tracks: Track[]
  detections: Detection[]
  liveEvents: Detection[]
  selectedTrackId: string | null
  filters: {
    from?: string
    to?: string
    cameraId?: string
    class?: string
    minConfidence?: number
  }
  pagination: {
    page: number
    pageSize: number
    total: number
    hasMore: boolean
  }
  loading: boolean
  error: string | null

  // Actions
  setTracks: (tracks: Track[], total: number, hasMore: boolean) => void
  setDetections: (detections: Detection[]) => void
  addLiveEvent: (detection: Detection) => void
  clearLiveEvents: () => void
  selectTrack: (id: string | null) => void
  setFilters: (filters: Partial<TracksSlice['filters']>) => void
  setPage: (page: number) => void
  setPageSize: (pageSize: number) => void
  setLoading: (loading: boolean) => void
  setError: (error: string | null) => void

  // Computed
  getTrack: (id: string) => Track | undefined
  getSelectedTrack: () => Track | undefined
}

const MAX_LIVE_EVENTS = 1000

export const createTracksSlice: StateCreator<TracksSlice> = (set, get) => ({
  // Initial state
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

  // Actions
  setTracks: (tracks, total, hasMore) =>
    set((state) => ({
      tracks,
      pagination: { ...state.pagination, total, hasMore },
      error: null,
    })),

  setDetections: (detections) => set({ detections, error: null }),

  addLiveEvent: (detection) =>
    set((state) => {
      const updated = [...state.liveEvents, detection]
      return {
        liveEvents: updated.slice(-MAX_LIVE_EVENTS),
      }
    }),

  clearLiveEvents: () => set({ liveEvents: [] }),

  selectTrack: (id) => set({ selectedTrackId: id }),

  setFilters: (filters) =>
    set((state) => ({
      filters: { ...state.filters, ...filters },
      pagination: { ...state.pagination, page: 1 }, // Reset to first page
    })),

  setPage: (page) =>
    set((state) => ({
      pagination: { ...state.pagination, page },
    })),

  setPageSize: (pageSize) =>
    set((state) => ({
      pagination: { ...state.pagination, pageSize, page: 1 },
    })),

  setLoading: (loading) => set({ loading }),

  setError: (error) => set({ error }),

  // Computed
  getTrack: (id) => get().tracks.find((track) => track.id === id),

  getSelectedTrack: () => {
    const { tracks, selectedTrackId } = get()
    return tracks.find((track) => track.id === selectedTrackId)
  },
})