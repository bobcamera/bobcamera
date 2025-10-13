import { modals } from '@mantine/modals'
import { Text } from '@mantine/core'

interface ConfirmDialogOptions {
  title: string
  message: string
  confirmLabel?: string
  cancelLabel?: string
  onConfirm: () => void | Promise<void>
  onCancel?: () => void
  color?: string
}

export function openConfirmDialog({
  title,
  message,
  confirmLabel = 'Confirm',
  cancelLabel = 'Cancel',
  onConfirm,
  onCancel,
  color = 'red',
}: ConfirmDialogOptions) {
  modals.openConfirmModal({
    title,
    children: <Text size="sm">{message}</Text>,
    labels: { confirm: confirmLabel, cancel: cancelLabel },
    confirmProps: { color },
    onConfirm: async () => {
      await onConfirm()
    },
    onCancel,
  })
}