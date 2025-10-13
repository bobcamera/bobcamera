import axios, { type AxiosInstance, type AxiosError } from 'axios'
import {
  type SystemHealth,
  SystemHealthSchema,
  type Camera,
  CameraSchema,
  type Detection,
  DetectionSchema,
  type Track,
  TrackSchema,
  type Recording,
  RecordingSchema,
  type Config,
  ConfigSchema,
  type LogEntry,
  LogEntrySchema,
  type Metrics,
  MetricsSchema,
  type PaginatedResponse,
  PaginatedResponseSchema,
} from './schema'

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || '/api'

class APIClient {
  private client: AxiosInstance

  constructor() {
    this.client = axios.create({
      baseURL: API_BASE_URL,
      timeout: 10000,
      headers: {
        'Content-Type': 'application/json',
      },
    })

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => {
        console.error('API Error:', error.message)
        return Promise.reject(error)
      }
    )
  }

  // Health Check
  async checkHealth(): Promise<boolean> {
    try {
      const response = await this.client.get('/health', { timeout: 3000 })
      return response.status === 200
    } catch {
      return false
    }
  }

  // System Health
  async getSystemHealth(): Promise<SystemHealth> {
    try {
      const response = await this.client.get('/system/health')
      // Validate that we got an object, not a string (HTML error page)
      if (typeof response.data === 'string') {
        throw new Error('Backend returned HTML instead of JSON - is the backend running?')
      }
      return SystemHealthSchema.parse(response.data)
    } catch (error) {
      // Re-throw with more context
      if (axios.isAxiosError(error)) {
        throw new Error(`Failed to fetch system health: ${error.message}`)
      }
      throw error
    }
  }

  // Version Info
  async getVersion(): Promise<{ ui: string; backend: string }> {
    const response = await this.client.get('/version')
    return response.data
  }

  // Cameras
  async getCameras(): Promise<Camera[]> {
    const response = await this.client.get('/cameras')
    return response.data.map((camera: any) => CameraSchema.parse(camera))
  }

  async getCamera(id: string): Promise<Camera> {
    const response = await this.client.get(`/cameras/${id}`)
    return CameraSchema.parse(response.data)
  }

  async createCamera(data: Omit<Camera, 'id' | 'status'>): Promise<Camera> {
    const response = await this.client.post('/cameras', data)
    return CameraSchema.parse(response.data)
  }

  async updateCamera(id: string, data: Partial<Camera>): Promise<Camera> {
    const response = await this.client.put(`/cameras/${id}`, data)
    return CameraSchema.parse(response.data)
  }

  async deleteCamera(id: string): Promise<void> {
    await this.client.delete(`/cameras/${id}`)
  }

  async testCamera(id: string): Promise<boolean> {
    try {
      const response = await this.client.post(`/cameras/${id}/test`)
      return response.data?.success ?? true
    } catch {
      return false
    }
  }

  // Tracks & Detections
  async getTracks(params?: {
    from?: string
    to?: string
    cameraId?: string
    class?: string
    minConfidence?: number
    page?: number
    pageSize?: number
  }): Promise<PaginatedResponse<Track>> {
    const response = await this.client.get('/tracks', { params })
    return PaginatedResponseSchema(TrackSchema).parse(response.data)
  }

  async getTrack(id: string): Promise<Track> {
    const response = await this.client.get(`/tracks/${id}`)
    return TrackSchema.parse(response.data)
  }

  async getDetections(params?: {
    from?: string
    to?: string
    cameraId?: string
    class?: string
    minConfidence?: number
    page?: number
    pageSize?: number
  }): Promise<PaginatedResponse<Detection>> {
    const response = await this.client.get('/detections', { params })
    return PaginatedResponseSchema(DetectionSchema).parse(response.data)
  }

  // Recordings
  async getRecordings(params?: {
    from?: string
    to?: string
    cameraId?: string
    page?: number
    pageSize?: number
  }): Promise<PaginatedResponse<Recording>> {
    const response = await this.client.get('/recordings', { params })
    return PaginatedResponseSchema(RecordingSchema).parse(response.data)
  }

  async getRecording(id: string): Promise<Recording> {
    const response = await this.client.get(`/recordings/${id}`)
    return RecordingSchema.parse(response.data)
  }

  async bookmarkRecording(id: string, bookmarked: boolean): Promise<void> {
    await this.client.patch(`/recordings/${id}`, { bookmarked })
  }

  async deleteRecording(id: string): Promise<void> {
    await this.client.delete(`/recordings/${id}`)
  }

  // Configuration
  async getConfig(): Promise<Config> {
    const response = await this.client.get('/config')
    return ConfigSchema.parse(response.data)
  }

  async updateConfig(config: Partial<Config>): Promise<Config> {
    const response = await this.client.put('/config', config)
    return ConfigSchema.parse(response.data)
  }

  // Metrics
  async getMetrics(params?: { from?: string; to?: string; interval?: string }): Promise<Metrics[]> {
    const response = await this.client.get('/metrics', { params })
    return response.data.map((metric: any) => MetricsSchema.parse(metric))
  }

  // Logs
  async getLogs(params?: {
    tail?: number
    level?: 'debug' | 'info' | 'warn' | 'error'
  }): Promise<LogEntry[]> {
    const response = await this.client.get('/logs', { params })
    return response.data.map((log: any) => LogEntrySchema.parse(log))
  }

  // Control
  async startDetection(): Promise<void> {
    await this.client.post('/control/start')
  }

  async stopDetection(): Promise<void> {
    await this.client.post('/control/stop')
  }

  async getStatus(): Promise<{ running: boolean; uptime: number }> {
    const response = await this.client.get('/control/status')
    return response.data
  }
}

// Singleton instance
export const apiClient = new APIClient()

// Export as 'api' for backward compatibility
export const api = apiClient

// Export for testing
export { APIClient }