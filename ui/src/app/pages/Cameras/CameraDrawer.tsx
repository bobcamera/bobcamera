import { useState, useEffect } from 'react'
import {
  Drawer,
  TextInput,
  Select,
  PasswordInput,
  Switch,
  Button,
  Stack,
  Group,
} from '@mantine/core'
import { notifications } from '@mantine/notifications'
import { useAppStore } from '@/app/store'
import type { Camera } from '@/app/services/schema'

interface CameraDrawerProps {
  camera: Camera | null
  isOpen: boolean
  isCreating: boolean
  onClose: () => void
}

interface CameraFormData {
  name: string
  url: string
  protocol: 'rtsp' | 'onvif' | 'usb' | 'file'
  enabled: boolean
  username?: string
  password?: string
}

export function CameraDrawer({ camera, isOpen, isCreating, onClose }: CameraDrawerProps) {
  const createCamera = useAppStore((state) => state.createCamera)
  const updateCamera = useAppStore((state) => state.updateCamera)

  const [formData, setFormData] = useState<CameraFormData>({
    name: '',
    url: '',
    protocol: 'rtsp',
    enabled: true,
    username: '',
    password: '',
  })

  const [isSubmitting, setIsSubmitting] = useState(false)

  useEffect(() => {
    if (camera) {
      setFormData({
        name: camera.name,
        url: camera.url || '',
        protocol: camera.protocol,
        enabled: camera.enabled,
        username: camera.username || '',
        password: '',
      })
    } else {
      setFormData({
        name: '',
        url: '',
        protocol: 'rtsp',
        enabled: true,
        username: '',
        password: '',
      })
    }
  }, [camera, isOpen])

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setIsSubmitting(true)

    try {
      if (isCreating) {
        await createCamera(formData as any)
        notifications.show({
          title: 'Camera created',
          message: `${formData.name} has been added`,
          color: 'green',
        })
      } else if (camera) {
        await updateCamera(camera.id, formData as any)
        notifications.show({
          title: 'Camera updated',
          message: `${formData.name} has been updated`,
          color: 'blue',
        })
      }
      onClose()
    } catch (err) {
      notifications.show({
        title: isCreating ? 'Failed to create camera' : 'Failed to update camera',
        message: (err as Error).message,
        color: 'red',
      })
    } finally {
      setIsSubmitting(false)
    }
  }

  return (
    <Drawer
      opened={isOpen}
      onClose={onClose}
      title={isCreating ? 'Add Camera' : 'Edit Camera'}
      position="right"
      size="md"
      padding="lg"
    >
      <form onSubmit={handleSubmit}>
        <Stack gap="md">
          {/* Name */}
          <TextInput
            label="Camera Name"
            placeholder="Front Door Camera"
            required
            value={formData.name}
            onChange={(e) => setFormData({ ...formData, name: e.target.value })}
          />

          {/* Protocol */}
          <Select
            label="Protocol"
            required
            value={formData.protocol}
            onChange={(value) =>
              setFormData({ ...formData, protocol: value as CameraFormData['protocol'] })
            }
            data={[
              { value: 'rtsp', label: 'RTSP' },
              { value: 'onvif', label: 'ONVIF' },
              { value: 'usb', label: 'USB' },
              { value: 'file', label: 'File' },
            ]}
          />

          {/* URL */}
          <TextInput
            label="Stream URL"
            placeholder="rtsp://192.168.1.100:554/stream"
            required
            value={formData.url}
            onChange={(e) => setFormData({ ...formData, url: e.target.value })}
            description="Full URL to the camera stream"
            styles={{
              input: {
                fontFamily: 'monospace',
              },
            }}
          />

          {/* Username */}
          <TextInput
            label="Username"
            placeholder="admin"
            value={formData.username}
            onChange={(e) => setFormData({ ...formData, username: e.target.value })}
            autoComplete="username"
          />

          {/* Password */}
          <PasswordInput
            label="Password"
            placeholder={isCreating ? '' : '••••••••'}
            value={formData.password}
            onChange={(e) => setFormData({ ...formData, password: e.target.value })}
            description={!isCreating ? 'Leave blank to keep existing password' : undefined}
            autoComplete="current-password"
          />

          {/* Enabled */}
          <Switch
            label="Enable Camera"
            description="Start streaming from this camera immediately"
            checked={formData.enabled}
            onChange={(e) => setFormData({ ...formData, enabled: e.currentTarget.checked })}
          />

          {/* Actions */}
          <Group justify="flex-end" mt="md">
            <Button variant="default" onClick={onClose}>
              Cancel
            </Button>
            <Button type="submit" loading={isSubmitting}>
              {isCreating ? 'Create' : 'Update'}
            </Button>
          </Group>
        </Stack>
      </form>
    </Drawer>
  )
}