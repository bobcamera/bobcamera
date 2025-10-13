import { Group, Text, Stack } from '@mantine/core'
import { type ReactNode } from 'react'

interface FieldRowProps {
  label: string
  children: ReactNode
  hint?: string
  error?: string
  required?: boolean
}

export function FieldRow({ label, children, hint, error, required }: FieldRowProps) {
  return (
    <Stack gap="xs">
      <Group gap="xs">
        <Text size="sm" fw={500}>
          {label}
          {required && <Text component="span" c="red">*</Text>}
        </Text>
      </Group>
      {children}
      {hint && !error && (
        <Text size="xs" c="dimmed">
          {hint}
        </Text>
      )}
      {error && (
        <Text size="xs" c="red">
          {error}
        </Text>
      )}
    </Stack>
  )
}