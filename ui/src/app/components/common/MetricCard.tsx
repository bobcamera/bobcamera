import { Card, Group, Text, Stack, ThemeIcon } from '@mantine/core'
import { type ReactNode } from 'react'

interface MetricCardProps {
  title: string
  value: string | number
  icon?: ReactNode
  color?: string
  subtitle?: string
  trend?: {
    value: number
    label: string
  }
}

export function MetricCard({ title, value, icon, color = 'blue', subtitle, trend }: MetricCardProps) {
  return (
    <Card shadow="sm" padding="lg" radius="md" withBorder>
      <Stack gap="xs">
        <Group justify="space-between">
          <Text size="sm" c="dimmed" fw={500}>
            {title}
          </Text>
          {icon && (
            <ThemeIcon color={color} variant="light" size="lg" radius="md">
              {icon}
            </ThemeIcon>
          )}
        </Group>

        <Text size="xl" fw={700}>
          {value}
        </Text>

        {subtitle && (
          <Text size="xs" c="dimmed">
            {subtitle}
          </Text>
        )}

        {trend && (
          <Text
            size="xs"
            c={trend.value > 0 ? 'green' : trend.value < 0 ? 'red' : 'dimmed'}
            fw={500}
          >
            {trend.value > 0 ? '↑' : trend.value < 0 ? '↓' : '→'} {Math.abs(trend.value)}% {trend.label}
          </Text>
        )}
      </Stack>
    </Card>
  )
}