import { ReactElement } from 'react'
import { render, RenderOptions } from '@testing-library/react'
import { BrowserRouter } from 'react-router-dom'
import { MantineProvider } from '@mantine/core'
import { ModalsProvider } from '@mantine/modals'
import { Notifications } from '@mantine/notifications'
import { theme } from '@/app/theme'

interface AllTheProvidersProps {
  children: React.ReactNode
}

function AllTheProviders({ children }: AllTheProvidersProps) {
  return (
    <BrowserRouter>
      <MantineProvider theme={theme} defaultColorScheme="dark">
        <ModalsProvider>
          <Notifications />
          {children}
        </ModalsProvider>
      </MantineProvider>
    </BrowserRouter>
  )
}

function customRender(
  ui: ReactElement,
  options?: Omit<RenderOptions, 'wrapper'>
) {
  return render(ui, { wrapper: AllTheProviders, ...options })
}

// Re-export everything
export * from '@testing-library/react'
export { customRender as render }

// Mock API responses
export const mockHealthResponse = {
  status: 'ok',
  cpuLoad: 45.2,
  gpuLoad: 32.1,
  memory: {
    used: 4096,
    total: 16384,
    percent: 25,
  },
  disk: {
    used: 102400,
    total: 512000,
    percent: 20,
  },
  uptime: 86400,
  versions: {
    backend: '1.0.0',
    ros2: 'humble',
  },
}

export const mockCamerasResponse = [
  {
    id: 'cam1',
    name: 'Front Camera',
    enabled: true,
    url: 'rtsp://192.168.1.100:554/stream',
    protocol: 'rtsp',
    status: 'online',
    lastSeen: new Date().toISOString(),
  },
  {
    id: 'cam2',
    name: 'Back Camera',
    enabled: false,
    url: 'rtsp://192.168.1.101:554/stream',
    protocol: 'rtsp',
    status: 'offline',
    lastSeen: new Date(Date.now() - 3600000).toISOString(),
  },
]

export const mockTracksResponse = {
  data: [
    {
      id: 'track1',
      cameraId: 'cam1',
      class: 'bird',
      confidence: 0.95,
      timestamp: new Date().toISOString(),
      duration: 5.2,
      bbox: { x: 100, y: 100, width: 50, height: 50 },
    },
    {
      id: 'track2',
      cameraId: 'cam1',
      class: 'insect',
      confidence: 0.87,
      timestamp: new Date(Date.now() - 60000).toISOString(),
      duration: 2.1,
      bbox: { x: 200, y: 150, width: 30, height: 30 },
    },
  ],
  total: 2,
  page: 1,
  pageSize: 10,
}

export const mockConfigResponse = {
  detection: {
    enabled: true,
    confidence_threshold: 0.5,
    classes: ['bird', 'insect', 'bat'],
  },
  tracking: {
    enabled: true,
    max_age: 30,
    min_hits: 3,
  },
  storage: {
    enabled: true,
    path: '/data/recordings',
    max_size_gb: 100,
  },
}

// Mock WebSocket events
export const mockWebSocketEvent = {
  type: 'detection',
  data: {
    cameraId: 'cam1',
    class: 'bird',
    confidence: 0.92,
    bbox: { x: 150, y: 120, width: 40, height: 40 },
    timestamp: new Date().toISOString(),
  },
}

// Helper to wait for async updates
export const waitForAsync = () =>
  new Promise((resolve) => setTimeout(resolve, 0))

// Helper to create mock store
export const createMockStore = (initialState = {}) => {
  return {
    getState: () => initialState,
    setState: () => {},
    subscribe: () => () => {},
    destroy: () => {},
  }
}