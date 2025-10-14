import { useEffect, useState } from 'react'
import {
  Container,
  Title,
  Text,
  Tabs,
  Card,
  Button,
  Group,
  Stack,
  NumberInput,
  TextInput,
  Switch,
  Slider,
  Alert,
  Loader,
  Center,
  Badge,
  Divider,
  MultiSelect,
} from '@mantine/core'
import { notifications } from '@mantine/notifications'
import { modals } from '@mantine/modals'
import {
  IconSettings,
  IconDeviceFloppy,
  IconRestore,
  IconAlertTriangle,
  IconEye,
  IconRoute,
  IconDatabase,
  IconNetwork,
  IconCheck,
  IconX,
} from '@tabler/icons-react'
import { useAppStore } from '@/app/store'
import { EmptyState } from '@/app/components/common/EmptyState'

// Available object classes for detection
const AVAILABLE_CLASSES = [
  'person',
  'bicycle',
  'car',
  'motorcycle',
  'airplane',
  'bus',
  'train',
  'truck',
  'boat',
  'bird',
  'cat',
  'dog',
  'horse',
  'sheep',
  'cow',
  'elephant',
  'bear',
  'zebra',
  'giraffe',
]

export function Settings() {
  const config = useAppStore((state) => state.config)
  const draftConfig = useAppStore((state) => state.draftConfig)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const fetchConfig = useAppStore((state) => state.fetchConfig)
  const updateDraftConfig = useAppStore((state) => state.updateDraftConfig)
  const saveConfig = useAppStore((state) => state.saveConfig)
  const resetDraftConfig = useAppStore((state) => state.resetDraftConfig)
  const hasPendingChanges = useAppStore((state) => state.hasPendingChanges)

  const [activeTab, setActiveTab] = useState<string | null>('detection')
  const [isSaving, setIsSaving] = useState(false)

  useEffect(() => {
    fetchConfig()
  }, [fetchConfig])

  const handleSave = async () => {
    if (!hasPendingChanges() || !draftConfig) {
      return
    }

    setIsSaving(true)
    try {
      await saveConfig(draftConfig)
      notifications.show({
        title: 'Settings saved',
        message: 'Configuration has been updated successfully',
        color: 'green',
        icon: <IconCheck size={18} />,
      })
    } catch (err) {
      notifications.show({
        title: 'Failed to save settings',
        message: (err as Error).message,
        color: 'red',
        icon: <IconX size={18} />,
      })
    } finally {
      setIsSaving(false)
    }
  }

  const handleReset = () => {
    modals.openConfirmModal({
      title: 'Discard changes',
      children: (
        <Text size="sm">
          Are you sure you want to discard all unsaved changes? This action cannot be undone.
        </Text>
      ),
      labels: { confirm: 'Discard', cancel: 'Cancel' },
      confirmProps: { color: 'red' },
      onConfirm: () => {
        resetDraftConfig()
        notifications.show({
          title: 'Changes discarded',
          message: 'Configuration has been reset to saved values',
          color: 'blue',
        })
      },
    })
  }

  // Loading state
  if (backendStatus === 'connecting' || !config) {
    return (
      <Center h={400}>
        <Stack align="center" gap="md">
          <Loader size="lg" />
          <Text c="dimmed">Loading settings...</Text>
        </Stack>
      </Center>
    )
  }

  // Disconnected state
  if (backendStatus === 'disconnected') {
    return (
      <EmptyState
        icon={<IconSettings size={48} />}
        title="Backend Disconnected"
        description="Unable to load settings. Please check your connection and try again."
        action={
          <Button onClick={() => fetchConfig()} leftSection={<IconRestore size={16} />}>
            Retry
          </Button>
        }
      />
    )
  }

  const draft = draftConfig || config
  const hasChanges = hasPendingChanges()

  return (
    <Container size="lg" py="xl">
      <Stack gap="lg">
        {/* Header */}
        <div>
          <Group justify="space-between" align="flex-start">
            <div>
              <Title order={1}>Settings</Title>
              <Text c="dimmed" mt={4}>
                Configure detection, tracking, storage, and network parameters
              </Text>
            </div>
            <Group gap="sm">
              <Button
                variant="default"
                leftSection={<IconRestore size={16} />}
                onClick={handleReset}
                disabled={!hasChanges}
              >
                Reset
              </Button>
              <Button
                leftSection={<IconDeviceFloppy size={16} />}
                onClick={handleSave}
                disabled={!hasChanges}
                loading={isSaving}
              >
                Save Changes
              </Button>
            </Group>
          </Group>
        </div>

        {/* Pending Changes Alert */}
        {hasChanges && (
          <Alert
            icon={<IconAlertTriangle size={20} />}
            title="Unsaved Changes"
            color="yellow"
            variant="light"
          >
            You have unsaved changes. Click "Save Changes" to apply them or "Reset" to discard.
          </Alert>
        )}

        {/* Settings Tabs */}
        <Tabs value={activeTab} onChange={setActiveTab}>
          <Tabs.List>
            <Tabs.Tab value="detection" leftSection={<IconEye size={16} />}>
              Detection
            </Tabs.Tab>
            <Tabs.Tab value="tracking" leftSection={<IconRoute size={16} />}>
              Tracking
            </Tabs.Tab>
            <Tabs.Tab value="storage" leftSection={<IconDatabase size={16} />}>
              Storage
            </Tabs.Tab>
            <Tabs.Tab value="network" leftSection={<IconNetwork size={16} />}>
              Network
            </Tabs.Tab>
          </Tabs.List>

          {/* Detection Settings */}
          <Tabs.Panel value="detection" pt="lg">
            <Card shadow="sm" padding="lg" radius="md" withBorder>
              <Stack gap="lg">
                <div>
                  <Group justify="space-between" mb="xs">
                    <Text fw={600} size="lg">
                      Detection Parameters
                    </Text>
                    <Badge
                      color={draft.detection?.enabled ? 'green' : 'gray'}
                      variant="light"
                    >
                      {draft.detection?.enabled ? 'Enabled' : 'Disabled'}
                    </Badge>
                  </Group>
                  <Text size="sm" c="dimmed">
                    Configure object detection model parameters and behavior
                  </Text>
                </div>

                <Divider />

                <Switch
                  label="Enable Detection"
                  description="Enable or disable object detection globally"
                  checked={draft.detection?.enabled || false}
                  onChange={(e) =>
                    updateDraftConfig({
                      detection: {
                        ...draft.detection,
                        enabled: e.currentTarget.checked,
                      },
                    })
                  }
                  size="md"
                />

                <div>
                  <Text size="sm" fw={500} mb="xs">
                    Confidence Threshold: {(draft.detection?.confidence || 0.5).toFixed(2)}
                  </Text>
                  <Slider
                    value={draft.detection?.confidence || 0.5}
                    onChange={(value) =>
                      updateDraftConfig({
                        detection: {
                          ...draft.detection,
                          confidence: value,
                        },
                      })
                    }
                    min={0}
                    max={1}
                    step={0.05}
                    marks={[
                      { value: 0, label: '0.0' },
                      { value: 0.25, label: '0.25' },
                      { value: 0.5, label: '0.5' },
                      { value: 0.75, label: '0.75' },
                      { value: 1, label: '1.0' },
                    ]}
                    disabled={!draft.detection?.enabled}
                  />
                  <Text size="xs" c="dimmed" mt="xs">
                    Minimum confidence score for detections. Higher values reduce false positives
                    but may miss valid detections.
                  </Text>
                </div>

                <div>
                  <Text size="sm" fw={500} mb="xs">
                    NMS Threshold: {(draft.detection?.nms || 0.45).toFixed(2)}
                  </Text>
                  <Slider
                    value={draft.detection?.nms || 0.45}
                    onChange={(value) =>
                      updateDraftConfig({
                        detection: {
                          ...draft.detection,
                          nms: value,
                        },
                      })
                    }
                    min={0}
                    max={1}
                    step={0.05}
                    marks={[
                      { value: 0, label: '0.0' },
                      { value: 0.25, label: '0.25' },
                      { value: 0.5, label: '0.5' },
                      { value: 0.75, label: '0.75' },
                      { value: 1, label: '1.0' },
                    ]}
                    disabled={!draft.detection?.enabled}
                  />
                  <Text size="xs" c="dimmed" mt="xs">
                    Non-maximum suppression threshold for removing duplicate detections. Lower
                    values are more aggressive.
                  </Text>
                </div>

                <MultiSelect
                  label="Detected Classes"
                  description="Select which object classes to detect"
                  placeholder="Select classes"
                  data={AVAILABLE_CLASSES}
                  value={draft.detection?.classes || []}
                  onChange={(value) =>
                    updateDraftConfig({
                      detection: {
                        ...draft.detection,
                        classes: value,
                      },
                    })
                  }
                  searchable
                  clearable
                  disabled={!draft.detection?.enabled}
                />
              </Stack>
            </Card>
          </Tabs.Panel>

          {/* Tracking Settings */}
          <Tabs.Panel value="tracking" pt="lg">
            <Card shadow="sm" padding="lg" radius="md" withBorder>
              <Stack gap="lg">
                <div>
                  <Group justify="space-between" mb="xs">
                    <Text fw={600} size="lg">
                      Tracking Parameters
                    </Text>
                    <Badge
                      color={draft.tracking?.enabled ? 'green' : 'gray'}
                      variant="light"
                    >
                      {draft.tracking?.enabled ? 'Enabled' : 'Disabled'}
                    </Badge>
                  </Group>
                  <Text size="sm" c="dimmed">
                    Configure object tracking algorithm parameters
                  </Text>
                </div>

                <Divider />

                <Switch
                  label="Enable Tracking"
                  description="Enable or disable object tracking globally"
                  checked={draft.tracking?.enabled || false}
                  onChange={(e) =>
                    updateDraftConfig({
                      tracking: {
                        ...draft.tracking,
                        enabled: e.currentTarget.checked,
                      },
                    })
                  }
                  size="md"
                />

                <NumberInput
                  label="Max Track Age"
                  description="Maximum number of frames to keep a track without detection"
                  placeholder="30"
                  value={draft.tracking?.maxAge || 30}
                  onChange={(value) =>
                    updateDraftConfig({
                      tracking: {
                        ...draft.tracking,
                        maxAge: Number(value),
                      },
                    })
                  }
                  min={1}
                  max={300}
                  disabled={!draft.tracking?.enabled}
                />

                <NumberInput
                  label="Min Hits"
                  description="Minimum number of detections before confirming a track"
                  placeholder="3"
                  value={draft.tracking?.minHits || 3}
                  onChange={(value) =>
                    updateDraftConfig({
                      tracking: {
                        ...draft.tracking,
                        minHits: Number(value),
                      },
                    })
                  }
                  min={1}
                  max={20}
                  disabled={!draft.tracking?.enabled}
                />

                <div>
                  <Text size="sm" fw={500} mb="xs">
                    IOU Threshold: {(draft.tracking?.iouThreshold || 0.3).toFixed(2)}
                  </Text>
                  <Slider
                    value={draft.tracking?.iouThreshold || 0.3}
                    onChange={(value) =>
                      updateDraftConfig({
                        tracking: {
                          ...draft.tracking,
                          iouThreshold: value,
                        },
                      })
                    }
                    min={0}
                    max={1}
                    step={0.05}
                    marks={[
                      { value: 0, label: '0.0' },
                      { value: 0.25, label: '0.25' },
                      { value: 0.5, label: '0.5' },
                      { value: 0.75, label: '0.75' },
                      { value: 1, label: '1.0' },
                    ]}
                    disabled={!draft.tracking?.enabled}
                  />
                  <Text size="xs" c="dimmed" mt="xs">
                    Intersection over Union threshold for matching detections to tracks. Higher
                    values require closer matches.
                  </Text>
                </div>
              </Stack>
            </Card>
          </Tabs.Panel>

          {/* Storage Settings */}
          <Tabs.Panel value="storage" pt="lg">
            <Card shadow="sm" padding="lg" radius="md" withBorder>
              <Stack gap="lg">
                <div>
                  <Group justify="space-between" mb="xs">
                    <Text fw={600} size="lg">
                      Storage Configuration
                    </Text>
                    <Badge color={draft.storage?.enabled ? 'green' : 'gray'} variant="light">
                      {draft.storage?.enabled ? 'Enabled' : 'Disabled'}
                    </Badge>
                  </Group>
                  <Text size="sm" c="dimmed">
                    Configure recording storage location and retention policies
                  </Text>
                </div>

                <Divider />

                <Switch
                  label="Enable Storage"
                  description="Enable or disable recording storage"
                  checked={draft.storage?.enabled || false}
                  onChange={(e) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        enabled: e.currentTarget.checked,
                      },
                    })
                  }
                  size="md"
                />

                <TextInput
                  label="Storage Path"
                  description="Directory path where recordings will be stored"
                  placeholder="/data/recordings"
                  value={draft.storage?.path || ''}
                  onChange={(e) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        path: e.currentTarget.value,
                      },
                    })
                  }
                  disabled={!draft.storage?.enabled}
                  styles={{
                    input: {
                      fontFamily: 'monospace',
                    },
                  }}
                />

                <NumberInput
                  label="Max Storage Size"
                  description="Maximum storage size in gigabytes (GB)"
                  placeholder="100"
                  value={Math.round((draft.storage?.maxSize || 100000000000) / 1000000000)}
                  onChange={(value) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        maxSize: Number(value) * 1000000000,
                      },
                    })
                  }
                  min={1}
                  max={10000}
                  suffix=" GB"
                  disabled={!draft.storage?.enabled}
                />

                <NumberInput
                  label="Retention Period"
                  description="Number of days to retain recordings before automatic deletion"
                  placeholder="30"
                  value={draft.storage?.retention || 30}
                  onChange={(value) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        retention: Number(value),
                      },
                    })
                  }
                  min={1}
                  max={365}
                  suffix=" days"
                  disabled={!draft.storage?.enabled}
                />
              </Stack>
            </Card>
          </Tabs.Panel>

          {/* Network Settings */}
          <Tabs.Panel value="network" pt="lg">
            <Card shadow="sm" padding="lg" radius="md" withBorder>
              <Stack gap="lg">
                <div>
                  <Text fw={600} size="lg" mb="xs">
                    Network Configuration
                  </Text>
                  <Text size="sm" c="dimmed">
                    Configure network ports and connectivity settings
                  </Text>
                </div>

                <Divider />

                <Alert
                  icon={<IconAlertTriangle size={20} />}
                  title="Warning"
                  color="orange"
                  variant="light"
                >
                  Changing network settings may require restarting the application. Ensure ports
                  are not already in use.
                </Alert>

                <NumberInput
                  label="API Port"
                  description="Port for REST API server"
                  placeholder="8080"
                  value={draft.network?.apiPort || 8080}
                  onChange={(value) =>
                    updateDraftConfig({
                      network: {
                        ...draft.network,
                        apiPort: Number(value),
                      },
                    })
                  }
                  min={1024}
                  max={65535}
                />

                <NumberInput
                  label="WebSocket Port"
                  description="Port for WebSocket event streaming"
                  placeholder="8081"
                  value={draft.network?.wsPort || 8081}
                  onChange={(value) =>
                    updateDraftConfig({
                      network: {
                        ...draft.network,
                        wsPort: Number(value),
                      },
                    })
                  }
                  min={1024}
                  max={65535}
                />

                <NumberInput
                  label="Stream Port"
                  description="Port for video streaming server"
                  placeholder="8082"
                  value={draft.network?.streamPort || 8082}
                  onChange={(value) =>
                    updateDraftConfig({
                      network: {
                        ...draft.network,
                        streamPort: Number(value),
                      },
                    })
                  }
                  min={1024}
                  max={65535}
                />
              </Stack>
            </Card>
          </Tabs.Panel>
        </Tabs>
      </Stack>
    </Container>
  )
}