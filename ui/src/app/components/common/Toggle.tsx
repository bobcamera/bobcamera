import * as Switch from '@radix-ui/react-switch'
import { cn } from '@/lib/utils'

interface ToggleProps {
  checked: boolean
  onCheckedChange: (checked: boolean) => void
  label?: string
  description?: string
  disabled?: boolean
  className?: string
}

export function Toggle({
  checked,
  onCheckedChange,
  label,
  description,
  disabled,
  className,
}: ToggleProps) {
  return (
    <div className={cn('flex items-center justify-between', className)}>
      <div className="flex-1">
        {label && <label className="text-sm font-medium text-gray-900">{label}</label>}
        {description && <p className="text-sm text-gray-500 mt-0.5">{description}</p>}
      </div>
      <Switch.Root
        checked={checked}
        onCheckedChange={onCheckedChange}
        disabled={disabled}
        className={cn(
          'relative w-11 h-6 rounded-full transition-colors',
          'data-[state=checked]:bg-blue-600 data-[state=unchecked]:bg-gray-200',
          'disabled:opacity-50 disabled:cursor-not-allowed'
        )}
      >
        <Switch.Thumb
          className={cn(
            'block w-5 h-5 bg-white rounded-full transition-transform',
            'data-[state=checked]:translate-x-5 data-[state=unchecked]:translate-x-0.5',
            'translate-y-0.5'
          )}
        />
      </Switch.Root>
    </div>
  )
}