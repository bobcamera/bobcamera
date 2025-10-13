import { Stack, Text, ThemeIcon } from '@mantine/core'
import { type ReactNode } from 'react'

interface EmptyStateProps {
  icon?: ReactNode
  title: string
  description?: string
  action?: ReactNode
}

export function EmptyState({ icon, title, description, action }: EmptyStateProps) {
  return (
    <Stack align="center" justify="center" py="xl" gap="md">
      {icon && (
        <ThemeIcon size={64} radius="xl" variant="light" color="gray">
          {icon}
        </ThemeIcon>
      )}
      <Stack gap="xs" align="center">
        <Text size="lg" fw={600}>
          {title}
        </Text>
        {description && (
          <Text size="sm" c="dimmed" ta="center" maw={400}>
            {description}
          </Text>
        )}
      </Stack>
      {action && <div style={{ marginTop: 16 }}>{action}</div>}
    </Stack>
  )
}