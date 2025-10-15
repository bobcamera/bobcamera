import { describe, it, expect } from 'vitest'
import { render, screen } from '@/test/utils'
import { StatusBadge } from './StatusBadge'

describe('StatusBadge', () => {
  it('renders status badge with text', () => {
    render(<StatusBadge status="ok" />)
    expect(screen.getByText('OK')).toBeInTheDocument()
  })

  it('renders custom label when provided', () => {
    render(<StatusBadge status="ok" label="All Good" />)
    expect(screen.getByText('All Good')).toBeInTheDocument()
  })

  it('applies correct color for ok status', () => {
    render(<StatusBadge status="ok" />)
    const badge = screen.getByText('OK')
    expect(badge).toBeInTheDocument()
  })

  it('applies correct color for error status', () => {
    render(<StatusBadge status="error" />)
    const badge = screen.getByText('Error')
    expect(badge).toBeInTheDocument()
  })

  it('applies correct color for warning status', () => {
    render(<StatusBadge status="warning" />)
    const badge = screen.getByText('Warning')
    expect(badge).toBeInTheDocument()
  })

  it('applies correct color for degraded status', () => {
    render(<StatusBadge status="degraded" />)
    const badge = screen.getByText('Degraded')
    expect(badge).toBeInTheDocument()
  })

  it('applies correct color for offline status', () => {
    render(<StatusBadge status="offline" />)
    const badge = screen.getByText('Offline')
    expect(badge).toBeInTheDocument()
  })

  it('applies correct color for online status', () => {
    render(<StatusBadge status="online" />)
    const badge = screen.getByText('Online')
    expect(badge).toBeInTheDocument()
  })

  it('applies correct color for unknown status', () => {
    render(<StatusBadge status="unknown" />)
    const badge = screen.getByText('Unknown')
    expect(badge).toBeInTheDocument()
  })

})