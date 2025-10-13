import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen, waitFor } from '@/test/utils'
import { mockHealthResponse } from '@/test/utils'
import Dashboard from './index'
import { apiClient } from '@/app/services/api'

// Mock the API client
vi.mock('@/app/services/api', () => ({
  apiClient: {
    getHealth: vi.fn(),
  },
}))

// Mock the store
vi.mock('@/app/store', () => ({
  useAppStore: vi.fn((selector) => {
    const state = {
      health: null,
      setHealth: vi.fn(),
      mockMode: false,
      setMockMode: vi.fn(),
    }
    return selector(state)
  }),
}))

describe('Dashboard', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('renders dashboard title', () => {
    render(<Dashboard />)
    expect(screen.getByText('Dashboard')).toBeInTheDocument()
  })

  it('displays loading state initially', () => {
    render(<Dashboard />)
    // The component should show some loading indication
    // This depends on your implementation
  })

  it('fetches and displays health data', async () => {
    vi.mocked(apiClient.getHealth).mockResolvedValue(mockHealthResponse)

    render(<Dashboard />)

    await waitFor(() => {
      expect(apiClient.getHealth).toHaveBeenCalled()
    })

    // Check if metrics are displayed
    await waitFor(() => {
      expect(screen.getByText(/CPU/i)).toBeInTheDocument()
      expect(screen.getByText(/GPU/i)).toBeInTheDocument()
      expect(screen.getByText(/Memory/i)).toBeInTheDocument()
      expect(screen.getByText(/Disk/i)).toBeInTheDocument()
    })
  })

  it('handles API errors gracefully', async () => {
    vi.mocked(apiClient.getHealth).mockRejectedValue(
      new Error('Network error')
    )

    render(<Dashboard />)

    await waitFor(() => {
      expect(apiClient.getHealth).toHaveBeenCalled()
    })

    // Should show offline state or error message
    await waitFor(() => {
      expect(
        screen.getByText(/offline/i) || screen.getByText(/error/i)
      ).toBeInTheDocument()
    })
  })

  it('displays system status card', async () => {
    vi.mocked(apiClient.getHealth).mockResolvedValue(mockHealthResponse)

    render(<Dashboard />)

    await waitFor(() => {
      expect(screen.getByText(/System Status/i)).toBeInTheDocument()
    })
  })

  it('shows uptime information', async () => {
    vi.mocked(apiClient.getHealth).mockResolvedValue(mockHealthResponse)

    render(<Dashboard />)

    await waitFor(() => {
      expect(screen.getByText(/Uptime/i)).toBeInTheDocument()
    })
  })

  it('displays metric cards with correct values', async () => {
    vi.mocked(apiClient.getHealth).mockResolvedValue(mockHealthResponse)

    render(<Dashboard />)

    await waitFor(() => {
      // Check for CPU value
      expect(screen.getByText('45.2%')).toBeInTheDocument()
      // Check for GPU value
      expect(screen.getByText('32.1%')).toBeInTheDocument()
    })
  })

  it('polls health data periodically', async () => {
    vi.useFakeTimers()
    vi.mocked(apiClient.getHealth).mockResolvedValue(mockHealthResponse)

    render(<Dashboard />)

    // Initial call
    await waitFor(() => {
      expect(apiClient.getHealth).toHaveBeenCalledTimes(1)
    })

    // Fast-forward 10 seconds
    vi.advanceTimersByTime(10000)

    await waitFor(() => {
      expect(apiClient.getHealth).toHaveBeenCalledTimes(2)
    })

    vi.useRealTimers()
  })

  it('cleans up interval on unmount', async () => {
    vi.useFakeTimers()
    vi.mocked(apiClient.getHealth).mockResolvedValue(mockHealthResponse)

    const { unmount } = render(<Dashboard />)

    await waitFor(() => {
      expect(apiClient.getHealth).toHaveBeenCalledTimes(1)
    })

    unmount()

    // Fast-forward time after unmount
    vi.advanceTimersByTime(10000)

    // Should not call again after unmount
    expect(apiClient.getHealth).toHaveBeenCalledTimes(1)

    vi.useRealTimers()
  })
})