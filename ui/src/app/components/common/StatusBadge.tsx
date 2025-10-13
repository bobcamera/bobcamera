import { Badge } from '@mantine/core'

type Status = 'ok' | 'success' | 'warning' | 'error' | 'degraded' | 'offline' | 'online' | 'unknown'

interface StatusBadgeProps {
  status: Status
  label?: string
}

const statusConfig: Record<Status, { color: string; label: string }> = {
  ok: { color: 'green', label: 'OK' },
  success: { color: 'green', label: 'Success' },
  warning: { color: 'yellow', label: 'Warning' },
  error: { color: 'red', label: 'Error' },
  degraded: { color: 'orange', label: 'Degraded' },
  offline: { color: 'red', label: 'Offline' },
  online: { color: 'green', label: 'Online' },
  unknown: { color: 'gray', label: 'Unknown' },
}

export function StatusBadge({ status, label }: StatusBadgeProps) {
  const config = statusConfig[status] || statusConfig.unknown
  
  return (
    <Badge color={config.color} variant="dot">
      {label || config.label}
    </Badge>
  )
}