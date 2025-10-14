import {
  Drawer,
  Stack,
  Group,
  Text,
  Badge,
  Image,
  Card,
  Divider,
  ThemeIcon,
  Button,
  Tooltip,
  CopyButton,
  ActionIcon,
} from '@mantine/core'
import {
  IconTarget,
  IconClock,
  IconCamera,
  IconTrendingUp,
  IconHash,
  IconCheck,
  IconCopy,
  IconCalendar,
} from '@tabler/icons-react'
import { useAppStore } from '@/app/store'
import type { Track } from '@/app/services/schema'

interface TrackDetailDrawerProps {
  track: Track
  opened: boolean
  onClose: () => void
}

export function TrackDetailDrawer({ track, opened, onClose }: TrackDetailDrawerProps) {
  const cameras = useAppStore((state) => state.cameras)
  const camera = cameras.find((c) => c.id === track.cameraId)

  const firstSeen = new Date(track.firstSeen)
  const lastSeen = new Date(track.lastSeen)
  const duration = lastSeen.getTime() - firstSeen.getTime()
  const durationSeconds = Math.floor(duration / 1000)
  const durationMinutes = Math.floor(durationSeconds / 60)
  const durationHours = Math.floor(durationMinutes / 60)

  const formatDuration = () => {
    if (durationHours > 0) {
      return `${durationHours}h ${durationMinutes % 60}m`
    } else if (durationMinutes > 0) {
      return `${durationMinutes}m ${durationSeconds % 60}s`
    } else {
      return `${durationSeconds}s`
    }
  }

  return (
    <Drawer
      opened={opened}
      onClose={onClose}
      position="right"
      size="lg"
      title={
        <Group gap="xs">
          <ThemeIcon variant="light" size="lg">
            <IconTarget size={20} />
          </ThemeIcon>
          <div>
            <Text fw={600}>Track Details</Text>
            <Text size="xs" c="dimmed">
              {track.class} • {track.detectionCount} detections
            </Text>
          </div>
        </Group>
      }
      padding="md"
    >
      <Stack gap="lg">
        {/* Thumbnail */}
        {track.thumbnailUrl && (
          <Card withBorder padding={0}>
            <Image
              src={track.thumbnailUrl}
              alt={`Track ${track.id}`}
              fit="contain"
              h={300}
              fallbackSrc="https://placehold.co/600x400?text=No+Thumbnail"
            />
          </Card>
        )}

        {/* Status Badge */}
        <Group justify="center">
          <Badge
            size="lg"
            variant="light"
            color={
              track.status === 'active'
                ? 'green'
                : track.status === 'lost'
                ? 'yellow'
                : 'gray'
            }
            leftSection={
              <div
                style={{
                  width: 8,
                  height: 8,
                  borderRadius: '50%',
                  backgroundColor:
                    track.status === 'active'
                      ? 'var(--mantine-color-green-6)'
                      : track.status === 'lost'
                      ? 'var(--mantine-color-yellow-6)'
                      : 'var(--mantine-color-gray-6)',
                }}
              />
            }
          >
            {track.status.toUpperCase()}
          </Badge>
        </Group>

        <Divider />

        {/* Basic Information */}
        <div>
          <Text size="sm" fw={600} mb="md" c="dimmed" tt="uppercase">
            Basic Information
          </Text>

          <Stack gap="md">
            {/* Track ID */}
            <Group justify="space-between" wrap="nowrap">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm">
                  <IconHash size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Track ID
                </Text>
              </Group>
              <Group gap="xs">
                <Tooltip label={track.id}>
                  <Text size="sm" ff="monospace" fw={500}>
                    {track.id.slice(0, 12)}...
                  </Text>
                </Tooltip>
                <CopyButton value={track.id}>
                  {({ copied, copy }) => (
                    <ActionIcon
                      size="sm"
                      variant="subtle"
                      color={copied ? 'teal' : 'gray'}
                      onClick={copy}
                    >
                      {copied ? <IconCheck size={14} /> : <IconCopy size={14} />}
                    </ActionIcon>
                  )}
                </CopyButton>
              </Group>
            </Group>

            {/* Camera */}
            <Group justify="space-between">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm">
                  <IconCamera size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Camera
                </Text>
              </Group>
              <Text size="sm" fw={500}>
                {camera?.name || track.cameraId}
              </Text>
            </Group>

            {/* Object Class */}
            <Group justify="space-between">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm">
                  <IconTarget size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Object Class
                </Text>
              </Group>
              <Badge variant="light" color="blue">
                {track.class}
              </Badge>
            </Group>

            {/* Average Confidence */}
            <Group justify="space-between">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm">
                  <IconTrendingUp size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Avg Confidence
                </Text>
              </Group>
              <Tooltip label={`${(track.avgConfidence * 100).toFixed(4)}%`}>
                <Badge
                  variant="light"
                  color={
                    track.avgConfidence >= 0.8
                      ? 'green'
                      : track.avgConfidence >= 0.6
                      ? 'yellow'
                      : 'red'
                  }
                >
                  {(track.avgConfidence * 100).toFixed(1)}%
                </Badge>
              </Tooltip>
            </Group>

            {/* Detection Count */}
            <Group justify="space-between">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm">
                  <IconHash size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Total Detections
                </Text>
              </Group>
              <Text size="sm" fw={500}>
                {track.detectionCount.toLocaleString()}
              </Text>
            </Group>
          </Stack>
        </div>

        <Divider />

        {/* Timeline Information */}
        <div>
          <Text size="sm" fw={600} mb="md" c="dimmed" tt="uppercase">
            Timeline
          </Text>

          <Stack gap="md">
            {/* First Seen */}
            <Group justify="space-between" wrap="nowrap">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm" color="green">
                  <IconClock size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  First Seen
                </Text>
              </Group>
              <div style={{ textAlign: 'right' }}>
                <Text size="sm" fw={500}>
                  {firstSeen.toLocaleDateString()}
                </Text>
                <Text size="xs" c="dimmed">
                  {firstSeen.toLocaleTimeString()}
                </Text>
              </div>
            </Group>

            {/* Last Seen */}
            <Group justify="space-between" wrap="nowrap">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm" color="red">
                  <IconClock size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Last Seen
                </Text>
              </Group>
              <div style={{ textAlign: 'right' }}>
                <Text size="sm" fw={500}>
                  {lastSeen.toLocaleDateString()}
                </Text>
                <Text size="xs" c="dimmed">
                  {lastSeen.toLocaleTimeString()}
                </Text>
              </div>
            </Group>

            {/* Duration */}
            <Group justify="space-between">
              <Group gap="xs">
                <ThemeIcon variant="light" size="sm" color="blue">
                  <IconCalendar size={14} />
                </ThemeIcon>
                <Text size="sm" c="dimmed">
                  Duration
                </Text>
              </Group>
              <Text size="sm" fw={500}>
                {formatDuration()}
              </Text>
            </Group>
          </Stack>
        </div>

        <Divider />

        {/* Statistics */}
        <div>
          <Text size="sm" fw={600} mb="md" c="dimmed" tt="uppercase">
            Statistics
          </Text>

          <Group grow>
            <Card withBorder padding="sm">
              <Text size="xs" c="dimmed" ta="center">
                Detections/Min
              </Text>
              <Text size="lg" fw={700} ta="center" mt={4}>
                {durationMinutes > 0
                  ? (track.detectionCount / durationMinutes).toFixed(1)
                  : track.detectionCount}
              </Text>
            </Card>

            <Card withBorder padding="sm">
              <Text size="xs" c="dimmed" ta="center">
                Avg Interval
              </Text>
              <Text size="lg" fw={700} ta="center" mt={4}>
                {track.detectionCount > 1
                  ? `${(durationSeconds / (track.detectionCount - 1)).toFixed(1)}s`
                  : 'N/A'}
              </Text>
            </Card>
          </Group>
        </div>

        {/* Additional Info */}
        {track.status === 'active' && (
          <Card withBorder bg="green.0">
            <Group gap="xs">
              <ThemeIcon color="green" variant="light" size="sm">
                <IconTarget size={14} />
              </ThemeIcon>
              <div>
                <Text size="sm" fw={500}>
                  Track is currently active
                </Text>
                <Text size="xs" c="dimmed">
                  This object is still being tracked by the system
                </Text>
              </div>
            </Group>
          </Card>
        )}

        {track.status === 'lost' && (
          <Card withBorder bg="yellow.0">
            <Group gap="xs">
              <ThemeIcon color="yellow" variant="light" size="sm">
                <IconTarget size={14} />
              </ThemeIcon>
              <div>
                <Text size="sm" fw={500}>
                  Track was lost
                </Text>
                <Text size="xs" c="dimmed">
                  The object is no longer visible or trackable
                </Text>
              </div>
            </Group>
          </Card>
        )}

        {track.status === 'completed' && (
          <Card withBorder bg="gray.0">
            <Group gap="xs">
              <ThemeIcon color="gray" variant="light" size="sm">
                <IconTarget size={14} />
              </ThemeIcon>
              <div>
                <Text size="sm" fw={500}>
                  Track completed
                </Text>
                <Text size="xs" c="dimmed">
                  This track has been finalized and archived
                </Text>
              </div>
            </Group>
          </Card>
        )}

        {/* Actions */}
        <Group grow>
          <Button variant="default" onClick={onClose}>
            Close
          </Button>
        </Group>
      </Stack>
    </Drawer>
  )
}