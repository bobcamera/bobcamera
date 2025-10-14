import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen, waitFor } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { Cameras } from './index'
import { renderWithProviders } from '@/test/utils'
import type { Camera } from '@/app/services/schema'

// Mock data
const mockCameras: Camera[] = [
  {
    id: '1',
    name: 'Front Door',
    enabled: true,
    url: 'rtsp://192.168.1.100:554/stream',
    protocol: 'rtsp',
    status: 'online',
    lastSeen: '2024-01-15T10:30:00Z',
    resolution: { width: 1920, height: 1080 },
    fps: 30,
  },
  {
    id: '2',
    name: 'Back Yard',
    enabled: false,
    url: 'rtsp://192.168.1.101:554/stream',
    protocol: 'rtsp',
    status: 'offline',
  },
]

describe('Cameras Page', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('renders loading state initially', () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: [],
        loading: true,
        error: null,
      },
    })

    expect(screen.getByText(/loading cameras/i)).toBeInTheDocument()
  })

  it('renders empty state when no cameras exist', async () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: [],
        loading: false,
        error: null,
      },
    })

    await waitFor(() => {
      expect(screen.getByText(/no cameras configured/i)).toBeInTheDocument()
    })
  })

  it('renders camera list when cameras exist', async () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    await waitFor(() => {
      expect(screen.getByText('Front Door')).toBeInTheDocument()
      expect(screen.getByText('Back Yard')).toBeInTheDocument()
    })
  })

  it('displays camera status badges correctly', async () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    await waitFor(() => {
      expect(screen.getByText('online')).toBeInTheDocument()
      expect(screen.getByText('disabled')).toBeInTheDocument()
    })
  })

  it('shows camera details', async () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    await waitFor(() => {
      expect(screen.getByText('rtsp://192.168.1.100:554/stream')).toBeInTheDocument()
      expect(screen.getByText('1920 × 1080')).toBeInTheDocument()
      expect(screen.getByText('30')).toBeInTheDocument()
    })
  })

  it('opens drawer when Add Camera button is clicked', async () => {
    const user = userEvent.setup()
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: [],
        loading: false,
        error: null,
      },
    })

    const addButton = screen.getByRole('button', { name: /add camera/i })
    await user.click(addButton)

    await waitFor(() => {
      expect(screen.getByText(/camera name/i)).toBeInTheDocument()
    })
  })

  it('opens drawer when Edit button is clicked', async () => {
    const user = userEvent.setup()
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    const editButtons = screen.getAllByTitle(/edit camera/i)
    await user.click(editButtons[0])

    await waitFor(() => {
      expect(screen.getByDisplayValue('Front Door')).toBeInTheDocument()
    })
  })

  it('shows confirmation modal when Delete button is clicked', async () => {
    const user = userEvent.setup()
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    const deleteButtons = screen.getAllByTitle(/delete camera/i)
    await user.click(deleteButtons[0])

    await waitFor(() => {
      expect(screen.getByText(/are you sure/i)).toBeInTheDocument()
    })
  })

  it('displays correct protocol badge', async () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    await waitFor(() => {
      const rtspBadges = screen.getAllByText('RTSP')
      expect(rtspBadges.length).toBeGreaterThan(0)
    })
  })

  it('shows refresh button', async () => {
    renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    const refreshButton = screen.getByTitle(/refresh cameras/i)
    expect(refreshButton).toBeInTheDocument()
  })

  it('renders responsive grid layout', async () => {
    const { container } = renderWithProviders(<Cameras />, {
      preloadedState: {
        cameras: mockCameras,
        loading: false,
        error: null,
      },
    })

    await waitFor(() => {
      const grid = container.querySelector('[class*="mantine-Grid"]')
      expect(grid).toBeInTheDocument()
    })
  })
})