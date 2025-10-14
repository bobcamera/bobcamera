import { useEffect, useState } from 'react'
import {
  Button,
  Card,
  Group,
  Stack,
  Text,
  Title,
  Badge,
  ActionIcon,
  Tooltip,
  Grid,
  Loader,
} from '@mantine/core'
import { notifications } from '@mantine/notifications'
import { modals } from '@mantine/modals'
import {
  IconCamera,
  IconPlus,
  IconEdit,
  IconTrash,
  IconTestPipe,
  IconPower,
  IconPowerOff,
  IconRefresh,
} from '@tabler/icons-react'
import { useAppStore } from '@/app/store'
import { EmptyState } from '@/app/components/common/EmptyState'
import { CameraDrawer } from './CameraDrawer'
import type { Camera as CameraType } from '@/app/services/schema'

export function Cameras() {
  const cameras = useAppStore((state) => state.cameras)
  const loading = useAppStore((state) => state.loading)
  const fetchCameras = useAppStore((state) => state.fetchCameras)
  const updateCamera = useAppStore((state) => state.updateCamera)
  const deleteCamera = useAppStore((state) => state.deleteCamera)
  const testCamera = useAppStore((state) => state.testCamera)

  const [selectedCamera, setSelectedCamera] = useState<CameraType | null>(null)
  const [isDrawerOpen, setIsDrawerOpen] = useState(false)
  const [isCreating, setIsCreating] = useState(false)
  const [testingCameraId, setTestingCameraId] = useState<string | null>(null)

  useEffect(() => {
    fetchCameras()

    // Poll every 10 seconds
    const interval = setInterval(() => {
      fetchCameras()
    }, 10000)

    return () => clearInterval(interval)
  }, [fetchCameras])

  const handleEdit = (camera: CameraType) => {
    setSelectedCamera(camera)
    setIsCreating(false)
    setIsDrawerOpen(true)
  }

  const handleCreate = () => {
    setSelectedCamera(null)
    setIsCreating(true)
    setIsDrawerOpen(true)
  }

  const handleToggleEnabled = async (camera: CameraType) => {
    try {
      await updateCamera(camera.id, { enabled: !camera.enabled })
      notifications.show({
        title: camera.enabled ? 'Camera disabled' : 'Camera enabled',
        message: `${camera.name} has been ${camera.enabled ? 'disabled' : 'enabled'}`,
        color: 'blue',
      })
    } catch (err) {
      notifications.show({
        title: 'Failed to update camera',
        message: (err as Error).message,
        color: 'red',
      })
    }
  }

  const handleTest = async (camera: CameraType) => {
    setTestingCameraId(camera.id)
    try {
      const result = await testCamera(camera.id)
      if (result) {
        notifications.show({
          title: 'Connection successful',
          message: `${camera.name} is reachable`,
          color: 'green',
        })
      } else {
        notifications.show({
          title: 'Connection failed',
          message: 'Unable to connect to camera',
          color: 'red',
        })
      }
    } catch (err) {
      notifications.show({
        title: 'Test failed',
        message: (err as Error).message,
        color: 'red',
      })
    } finally {
      setTestingCameraId(null)
    }
  }

  const handleDelete = (camera: CameraType) => {
    modals.openConfirmModal({
      title: 'Delete camera',
      children: (
        <Text size="sm">
          Are you sure you want to delete <strong>{camera.name}</strong>? This action cannot be
          undone.
        </Text>
      ),
      labels: { confirm: 'Delete', cancel: 'Cancel' },
      confirmProps: { color: 'red' },
      onConfirm: async () => {
        try {
          await deleteCamera(camera.id)
          notifications.show({
            title: 'Camera deleted',
            message: `${camera.name} has been removed`,
            color: 'blue',
          })
        } catch (err) {
          notifications.show({
            title: 'Failed to delete camera',
            message: (err as Error).message,
            color: 'red',
          })
        }
      },
    })
  }

  const getStatusColor = (camera: CameraType) => {
    if (!camera.enabled) return 'gray'
    switch (camera.status) {
      case 'online':
        return 'green'
      case 'offline':
        return 'gray'
      case 'error':
        return 'red'
      case 'connecting':
        return 'yellow'
      default:
        return 'gray'
    }
  }

  if (loading && cameras.length === 0) {
    return (
      <Stack align="center" justify="center" h="50vh">
        <Loader size="lg" />
        <Text c="dimmed">Loading cameras...</Text>
      </Stack>
    )
  }

  return (
    <Stack gap="lg">
      {/* Header */}
      <Group justify="space-between">
        <div>
          <Title order={2}>Cameras</Title>
          <Text size="sm" c="dimmed">
            Manage camera sources and configurations
          </Text>
        </div>
        <Group>
          <Tooltip label="Refresh cameras">
            <ActionIcon
              variant="light"
              size="lg"
              onClick={() => fetchCameras()}
              loading={loading}
            >
              <IconRefresh size={18} />
            </ActionIcon>
          </Tooltip>
          <Button leftSection={<IconPlus size={18} />} onClick={handleCreate}>
            Add Camera
          </Button>
        </Group>
      </Group>

      {/* Camera List */}
      {cameras.length === 0 ? (
        <EmptyState
          icon={IconCamera}
          title="No cameras configured"
          description="Get started by adding your first camera source"
          action={
            <Button leftSection={<IconPlus size={18} />} onClick={handleCreate}>
              Add Camera
            </Button>
          }
        />
      ) : (
        <Grid>
          {cameras.map((camera) => (
            <Grid.Col key={camera.id} span={{ base: 12, sm: 6, lg: 4 }}>
              <Card shadow="sm" padding="lg" radius="md" withBorder>
                <Stack gap="md">
                  {/* Header */}
                  <Group justify="space-between" wrap="nowrap">
                    <Group gap="sm" wrap="nowrap">
                      <IconCamera size={24} style={{ flexShrink: 0 }} />
                      <div style={{ minWidth: 0 }}>
                        <Text fw={600} truncate>
                          {camera.name}
                        </Text>
                        <Text size="xs" c="dimmed" tt="uppercase">
                          {camera.protocol}
                        </Text>
                      </div>
                    </Group>
                    <Badge color={getStatusColor(camera)} variant="light" size="sm">
                      {camera.enabled ? camera.status : 'disabled'}
                    </Badge>
                  </Group>

                  {/* Details */}
                  <Stack gap="xs">
                    {camera.url && (
                      <div>
                        <Text size="xs" c="dimmed">
                          URL
                        </Text>
                        <Text size="xs" ff="monospace" truncate>
                          {camera.url}
                        </Text>
                      </div>
                    )}
                    {camera.lastSeen && (
                      <div>
                        <Text size="xs" c="dimmed">
                          Last seen
                        </Text>
                        <Text size="xs">{new Date(camera.lastSeen).toLocaleString()}</Text>
                      </div>
                    )}
                    {camera.resolution && (
                      <Group gap="md">
                        <div>
                          <Text size="xs" c="dimmed">
                            Resolution
                          </Text>
                          <Text size="xs">
                            {camera.resolution.width} × {camera.resolution.height}
                          </Text>
                        </div>
                        {camera.fps && (
                          <div>
                            <Text size="xs" c="dimmed">
                              FPS
                            </Text>
                            <Text size="xs">{camera.fps}</Text>
                          </div>
                        )}
                      </Group>
                    )}
                  </Stack>

                  {/* Actions */}
                  <Group gap="xs" pt="xs" style={{ borderTop: '1px solid var(--mantine-color-default-border)' }}>
                    <Tooltip label={camera.enabled ? 'Disable camera' : 'Enable camera'}>
                      <ActionIcon
                        variant="light"
                        color={camera.enabled ? 'red' : 'green'}
                        onClick={() => handleToggleEnabled(camera)}
                      >
                        {camera.enabled ? <IconPowerOff size={18} /> : <IconPower size={18} />}
                      </ActionIcon>
                    </Tooltip>
                    <Tooltip label="Test connection">
                      <ActionIcon
                        variant="light"
                        color="blue"
                        onClick={() => handleTest(camera)}
                        loading={testingCameraId === camera.id}
                      >
                        <IconTestPipe size={18} />
                      </ActionIcon>
                    </Tooltip>
                    <Tooltip label="Edit camera">
                      <ActionIcon variant="light" color="blue" onClick={() => handleEdit(camera)}>
                        <IconEdit size={18} />
                      </ActionIcon>
                    </Tooltip>
                    <Tooltip label="Delete camera">
                      <ActionIcon variant="light" color="red" onClick={() => handleDelete(camera)}>
                        <IconTrash size={18} />
                      </ActionIcon>
                    </Tooltip>
                  </Group>
                </Stack>
              </Card>
            </Grid.Col>
          ))}
        </Grid>
      )}

      {/* Camera Drawer */}
      <CameraDrawer
        camera={selectedCamera}
        isOpen={isDrawerOpen}
        isCreating={isCreating}
        onClose={() => {
          setIsDrawerOpen(false)
          setSelectedCamera(null)
        }}
      />
    </Stack>
  )
}