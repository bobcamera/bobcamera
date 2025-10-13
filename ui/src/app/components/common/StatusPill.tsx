import { cn } from '@/lib/utils'
import { CheckCircle2, AlertCircle, XCircle, Loader2 } from 'lucide-react'

type Status = 'ok' | 'success' | 'warning' | 'error' | 'loading' | 'offline' | 'online'

interface StatusPillProps {
  status: Status
  label?: string
  className?: string
  showIcon?: boolean
}

const statusConfig = {
  ok: {
    icon: CheckCircle2,
    className: 'bg-green-100 text-green-800 border-green-200',
    label: 'OK',
  },
  success: {
    icon: CheckCircle2,
    className: 'bg-green-100 text-green-800 border-green-200',
    label: 'Success',
  },
  warning: {
    icon: AlertCircle,
    className: 'bg-yellow-100 text-yellow-800 border-yellow-200',
    label: 'Warning',
  },
  error: {
    icon: XCircle,
    className: 'bg-red-100 text-red-800 border-red-200',
    label: 'Error',
  },
  loading: {
    icon: Loader2,
    className: 'bg-blue-100 text-blue-800 border-blue-200',
    label: 'Loading',
  },
  offline: {
    icon: XCircle,
    className: 'bg-gray-100 text-gray-800 border-gray-200',
    label: 'Offline',
  },
  online: {
    icon: CheckCircle2,
    className: 'bg-green-100 text-green-800 border-green-200',
    label: 'Online',
  },
}

export function StatusPill({ status, label, className, showIcon = true }: StatusPillProps) {
  const config = statusConfig[status]
  const Icon = config.icon
  const displayLabel = label || config.label

  return (
    <span
      className={cn(
        'inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium border',
        config.className,
        className
      )}
    >
      {showIcon && (
        <Icon className={cn('w-3.5 h-3.5', status === 'loading' && 'animate-spin')} />
      )}
      {displayLabel}
    </span>
  )
}