import { useEffect } from 'react'
import { Grid, Card, Title, Text, Stack, Group, Badge, SimpleGrid, Alert } from '@mantine/core'
import {
  IconCpu,
  IconDeviceDesktop,
  IconDatabase,
  IconActivity,
  IconCamera,
  IconTarget,
} from '@tabler/icons-react'
import { MetricCard } from '@/app/components/common/MetricCard'
import { StatusBadge } from '@/app/components/common/StatusBadge'
import { EmptyState } from '@/app/components/common/EmptyState'
import { useAppStore } from '@/app/store'

export function Dashboard() {
  const { health, backendStatus, fetchSystemHealth } = useAppStore()
  const cameras = useAppStore((state) => state.cameras)
  const tracks = useAppStore((state) => state.tracks)

  useEffect(() => {
    fetchSystemHealth()
    const interval = setInterval(fetchSystemHealth, 10000)
    return () => clearInterval(interval)
  }, [fetchSystemHealth])

  const activeCameras = cameras.filter((c) => c.enabled).length
  const activeTracks = tracks.length
  const isOffline = backendStatus === 'offline'

  return (
    <Stack gap="lg">
      <Group justify="space-between">
        <Title order={2}>Dashboard</Title>
        <StatusBadge
          status={backendStatus === 'online' ? 'online' : 'offline'}
        />
      </Group>

      {/* Offline Alert */}
      {isOffline && (
        <Alert
          icon={<IconActivity size={20} />}
          title="Backend Offline"
          color="yellow"
          variant="light"
        >
          The BOB Camera backend is currently offline. Displaying UI structure with placeholder data.
        </Alert>
      )}

      {/* Metrics Grid */}
      <SimpleGrid cols={{ base: 1, sm: 2, lg: 4 }} spacing="lg">
        <MetricCard
          title="CPU Load"
          value={health?.cpuLoad ? `${health.cpuLoad.toFixed(1)}%` : 'N/A'}
          icon={<IconCpu size={20} />}
          color="blue"
        />
        <MetricCard
          title="GPU Load"
          value={health?.gpuLoad ? `${health.gpuLoad.toFixed(1)}%` : 'N/A'}
          icon={<IconDeviceDesktop size={20} />}
          color="violet"
        />
        <MetricCard
          title="Memory"
          value={health?.memory ? `${health.memory.percent.toFixed(1)}%` : 'N/A'}
          icon={<IconDatabase size={20} />}
          color="cyan"
        />
        <MetricCard
          title="Disk"
          value={health?.disk ? `${health.disk.percent.toFixed(1)}%` : 'N/A'}
          icon={<IconDatabase size={20} />}
          color="teal"
        />
      </SimpleGrid>

      {/* System Status */}
      <Grid>
        <Grid.Col span={{ base: 12, md: 6 }}>
          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Stack gap="md">
              <Group justify="space-between">
                <Text size="lg" fw={600}>
                  System Status
                </Text>
                <Badge color={health?.status === 'ok' ? 'green' : 'red'} variant="filled">
                  {health?.status || 'Unknown'}
                </Badge>
              </Group>

              <Stack gap="xs">
                <Group justify="space-between">
                  <Text size="sm" c="dimmed">
                    Uptime
                  </Text>
                  <Text size="sm" fw={500}>
                    {health?.uptime ? formatUptime(health.uptime) : 'N/A'}
                  </Text>
                </Group>

                <Group justify="space-between">
                  <Text size="sm" c="dimmed">
                    Active Cameras
                  </Text>
                  <Badge size="lg" variant="light" color="blue">
                    {activeCameras}
                  </Badge>
                </Group>

                <Group justify="space-between">
                  <Text size="sm" c="dimmed">
                    Active Tracks
                  </Text>
                  <Badge size="lg" variant="light" color="green">
                    {activeTracks}
                  </Badge>
                </Group>
              </Stack>
            </Stack>
          </Card>
        </Grid.Col>

        <Grid.Col span={{ base: 12, md: 6 }}>
          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Stack gap="md">
              <Text size="lg" fw={600}>
                Quick Stats
              </Text>

              <SimpleGrid cols={2} spacing="md">
                <Stack gap="xs" align="center">
                  <IconCamera size={32} color="var(--mantine-color-blue-6)" />
                  <Text size="xl" fw={700}>
                    {cameras.length}
                  </Text>
                  <Text size="sm" c="dimmed">
                    Total Cameras
                  </Text>
                </Stack>

                <Stack gap="xs" align="center">
                  <IconTarget size={32} color="var(--mantine-color-green-6)" />
                  <Text size="xl" fw={700}>
                    {tracks.length}
                  </Text>
                  <Text size="sm" c="dimmed">
                    Total Tracks
                  </Text>
                </Stack>
              </SimpleGrid>
            </Stack>
          </Card>
        </Grid.Col>
      </Grid>

      {/* Recent Events */}
      <Card shadow="sm" padding="lg" radius="md" withBorder>
        <Stack gap="md">
          <Text size="lg" fw={600}>
            Recent Events
          </Text>
          {tracks.length === 0 ? (
            <EmptyState
              icon={<IconActivity size={24} />}
              title="No recent events"
              description="Events will appear here as they are detected."
            />
          ) : (
            <Stack gap="xs">
              {tracks.slice(0, 5).map((track) => (
                <Group key={track.id} justify="space-between" p="sm" style={{ borderRadius: 8, backgroundColor: 'var(--mantine-color-gray-0)' }}>
                  <Group>
                    <Badge color="blue">{track.class}</Badge>
                    <Text size="sm">Track #{track.id}</Text>
                  </Group>
                  <Text size="xs" c="dimmed">
                    {new Date(track.lastSeen).toLocaleTimeString()}
                  </Text>
                </Group>
              ))}
            </Stack>
          )}
        </Stack>
      </Card>
    </Stack>
  )
}

function formatUptime(seconds: number): string {
  const days = Math.floor(seconds / 86400)
  const hours = Math.floor((seconds % 86400) / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)

  if (days > 0) {
    return `${days}d ${hours}h ${minutes}m`
  } else if (hours > 0) {
    return `${hours}h ${minutes}m`
  } else {
    return `${minutes}m`
  }
}