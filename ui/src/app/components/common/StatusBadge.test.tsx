import { describe, it, expect } from 'vitest'
import { render, screen } from '@/test/utils'
import StatusBadge from './StatusBadge'

describe('StatusBadge', () => {
  it('renders status badge with text', () => {
    render(<StatusBadge status="ok" />)
    expect(screen.getByText('ok')).toBeInTheDocument()
  })

  it('renders custom label when provided', () => {
    render(<StatusBadge status="ok" label="All Good" />)
    expect(screen.getByText('All Good')).toBeInTheDocument()
  })

  it('applies correct color for ok status', () => {
    render(<StatusBadge status="ok" />)
    const badge = screen.getByText('ok')
    expect(badge).toHaveClass('bg-green-500')
  })

  it('applies correct color for error status', () => {
    render(<StatusBadge status="error" />)
    const badge = screen.getByText('error')
    expect(badge).toHaveClass('bg-red-500')
  })

  it('applies correct color for warning status', () => {
    render(<StatusBadge status="warning" />)
    const badge = screen.getByText('warning')
    expect(badge).toHaveClass('bg-yellow-500')
  })

  it('applies correct color for degraded status', () => {
    render(<StatusBadge status="degraded" />)
    const badge = screen.getByText('degraded')
    expect(badge).toHaveClass('bg-orange-500')
  })

  it('applies correct color for offline status', () => {
    render(<StatusBadge status="offline" />)
    const badge = screen.getByText('offline')
    expect(badge).toHaveClass('bg-gray-500')
  })

  it('applies correct color for online status', () => {
    render(<StatusBadge status="online" />)
    const badge = screen.getByText('online')
    expect(badge).toHaveClass('bg-green-500')
  })

  it('applies correct color for unknown status', () => {
    render(<StatusBadge status="unknown" />)
    const badge = screen.getByText('unknown')
    expect(badge).toHaveClass('bg-gray-400')
  })

  it('renders with small size', () => {
    render(<StatusBadge status="ok" size="sm" />)
    const badge = screen.getByText('ok')
    expect(badge).toHaveClass('text-xs')
  })

  it('renders with medium size', () => {
    render(<StatusBadge status="ok" size="md" />)
    const badge = screen.getByText('ok')
    expect(badge).toHaveClass('text-sm')
  })

  it('renders with large size', () => {
    render(<StatusBadge status="ok" size="lg" />)
    const badge = screen.getByText('ok')
    expect(badge).toHaveClass('text-base')
  })
})