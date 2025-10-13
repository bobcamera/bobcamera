import { describe, it, expect } from 'vitest'
import { render, screen } from '@/test/utils'
import { IconCpu } from '@tabler/icons-react'
import MetricCard from './MetricCard'

describe('MetricCard', () => {
  it('renders metric card with title and value', () => {
    render(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        icon={IconCpu}
      />
    )

    expect(screen.getByText('CPU Usage')).toBeInTheDocument()
    expect(screen.getByText('45.2%')).toBeInTheDocument()
  })

  it('renders subtitle when provided', () => {
    render(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        subtitle="4 cores"
        icon={IconCpu}
      />
    )

    expect(screen.getByText('4 cores')).toBeInTheDocument()
  })

  it('renders trend indicator when provided', () => {
    render(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        trend={{ value: 5.2, direction: 'up' }}
        icon={IconCpu}
      />
    )

    expect(screen.getByText('+5.2%')).toBeInTheDocument()
  })

  it('applies correct color for trend direction', () => {
    const { rerender } = render(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        trend={{ value: 5.2, direction: 'up' }}
        icon={IconCpu}
      />
    )

    let trendElement = screen.getByText('+5.2%')
    expect(trendElement).toHaveClass('text-red-500')

    rerender(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        trend={{ value: 5.2, direction: 'down' }}
        icon={IconCpu}
      />
    )

    trendElement = screen.getByText('-5.2%')
    expect(trendElement).toHaveClass('text-green-500')
  })

  it('renders icon when provided', () => {
    render(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        icon={IconCpu}
      />
    )

    // Icon should be rendered (check for svg element)
    const icon = document.querySelector('svg')
    expect(icon).toBeInTheDocument()
  })

  it('applies custom color', () => {
    render(
      <MetricCard
        title="CPU Usage"
        value="45.2%"
        icon={IconCpu}
        color="blue"
      />
    )

    // Check if color is applied to icon wrapper
    const iconWrapper = document.querySelector('[class*="bg-blue"]')
    expect(iconWrapper).toBeInTheDocument()
  })
})