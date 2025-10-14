import { useEffect, useState } from 'react'
import {
  Container,
  Title,
  Text,
  Stack,
  Card,
  Group,
  Progress,
  Badge,
  SimpleGrid,
  ThemeIcon,
  Alert,
  Loader,
  Center,
  Button,
  Divider,
  Table,
  RingProgress,
} from '@mantine/core'
import {
  IconCpu,
  IconDeviceDesktop,
  IconDatabase,
  IconTemperature,
  IconClock,
  IconRefresh,
  IconAlertCircle,
  IconCheck,
  IconX,
  IconServer,
  IconBrandPython,
  IconCode,
} from '@tabler/icons-react'
import { apiClient } from '@/app/services/api'
import type { SystemHealth } from '@/app/services/schema'
import { formatBytes } from '@/lib/utils'

export default function System() {
  const [health, setHealth] = useState<SystemHealth | null>(null)
  const [isLoading, setIsLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date())

  useEffect(() => {
    fetchSystemHealth()
    // Auto-refresh every 5 seconds
    const interval = setInterval(fetchSystemHealth, 5000)
    return () => clearInterval(interval)
  }, [])

  const fetchSystemHealth = async () => {
    try {
      const data = await apiClient.getSystemHealth()
      setHealth(data)
      setError(null)
      setLastUpdate(new Date())
    } catch (err) {
      setError((err as Error).message)
    } finally {
      setIsLoading(false)
    }
  }

  const formatUptime = (seconds: number) => {
    const days = Math.floor(seconds / 86400)
    const hours = Math.floor((seconds % 86400) / 3600)
    const minutes = Math.floor((seconds % 3600) / 60)
    
    if (days > 0) return `${days}d ${hours}h ${minutes}m`
    if (hours > 0) return `${hours}h ${minutes}m`
    return `${minutes}m`
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'ok':
        return 'green'
      case 'degraded':
        return 'yellow'
      case 'error':
        return 'red'
      default:
        return 'gray'
    }
  }

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'ok':
        return <IconCheck size={20} />
      case 'degraded':
        return <IconAlertCircle size={20} />
      case 'error':
        return <IconX size={20} />
      default:
        return <IconAlertCircle size={20} />
    }
  }

  const getProgressColor = (percent: number) => {
    if (percent < 60) return 'green'
    if (percent < 80) return 'yellow'
    return 'red'
  }

  if (isLoading && !health) {
    return (
      <Center h={400}>
        <Stack align="center" gap="md">
          <Loader size="lg" />
          <Text c="dimmed">Loading system information...</Text>
        </Stack>
      </Center>
    )
  }

  return (
    <Container size="xl" py="xl">
      <Stack gap="lg">
        {/* Header */}
        <Group justify="space-between" align="flex-start">
          <div>
            <Title order={2}>System Status</Title>
            <Text c="dimmed" size="sm">
              Monitor system health and performance metrics
            </Text>
          </div>
          <Group>
            <Text size="xs" c="dimmed">
              Last updated: {lastUpdate.toLocaleTimeString()}
            </Text>
            <Button
              leftSection={<IconRefresh size={16} />}
              variant="light"
              onClick={fetchSystemHealth}
              loading={isLoading}
            >
              Refresh
            </Button>
          </Group>
        </Group>

        {/* Error Alert */}
        {error && (
          <Alert
            icon={<IconAlertCircle size={16} />}
            title="Connection Error"
            color="red"
            variant="light"
          >
            {error}
          </Alert>
        )}

        {health && (
          <>
            {/* Overall Status Card */}
            <Card withBorder padding="lg">
              <Group justify="space-between">
                <Group>
                  <ThemeIcon size="xl" variant="light" color={getStatusColor(health.status)}>
                    {getStatusIcon(health.status)}
                  </ThemeIcon>
                  <div>
                    <Text size="sm" c="dimmed">
                      System Status
                    </Text>
                    <Text size="xl" fw={700} tt="uppercase">
                      {health.status}
                    </Text>
                  </div>
                </Group>
                <Group>
                  <div style={{ textAlign: 'right' }}>
                    <Text size="sm" c="dimmed">
                      Uptime
                    </Text>
                    <Group gap="xs">
                      <IconClock size={16} />
                      <Text size="lg" fw={600}>
                        {formatUptime(health.uptime)}
                      </Text>
                    </Group>
                  </div>
                </Group>
              </Group>
            </Card>

            {/* Resource Usage Cards */}
            <SimpleGrid cols={{ base: 1, sm: 2, lg: 4 }}>
              {/* CPU */}
              <Card withBorder padding="lg">
                <Stack gap="md">
                  <Group justify="space-between">
                    <Group gap="xs">
                      <ThemeIcon size="lg" variant="light" color="blue">
                        <IconCpu size={20} />
                      </ThemeIcon>
                      <Text fw={600}>CPU</Text>
                    </Group>
                    <Text size="xl" fw={700}>
                      {health.cpuLoad.toFixed(1)}%
                    </Text>
                  </Group>
                  <Progress
                    value={health.cpuLoad}
                    color={getProgressColor(health.cpuLoad)}
                    size="lg"
                    radius="md"
                  />
                </Stack>
              </Card>

              {/* GPU */}
              {health.gpuLoad !== undefined && (
                <Card withBorder padding="lg">
                  <Stack gap="md">
                    <Group justify="space-between">
                      <Group gap="xs">
                        <ThemeIcon size="lg" variant="light" color="grape">
                          <IconDeviceDesktop size={20} />
                        </ThemeIcon>
                        <Text fw={600}>GPU</Text>
                      </Group>
                      <Text size="xl" fw={700}>
                        {health.gpuLoad.toFixed(1)}%
                      </Text>
                    </Group>
                    <Progress
                      value={health.gpuLoad}
                      color={getProgressColor(health.gpuLoad)}
                      size="lg"
                      radius="md"
                    />
                  </Stack>
                </Card>
              )}

              {/* Memory */}
              <Card withBorder padding="lg">
                <Stack gap="md">
                  <Group justify="space-between">
                    <Group gap="xs">
                      <ThemeIcon size="lg" variant="light" color="cyan">
                        <IconServer size={20} />
                      </ThemeIcon>
                      <Text fw={600}>Memory</Text>
                    </Group>
                    <Text size="xl" fw={700}>
                      {health.memory.percent.toFixed(1)}%
                    </Text>
                  </Group>
                  <Progress
                    value={health.memory.percent}
                    color={getProgressColor(health.memory.percent)}
                    size="lg"
                    radius="md"
                  />
                  <Text size="xs" c="dimmed" ta="center">
                    {formatBytes(health.memory.used)} / {formatBytes(health.memory.total)}
                  </Text>
                </Stack>
              </Card>

              {/* Disk */}
              <Card withBorder padding="lg">
                <Stack gap="md">
                  <Group justify="space-between">
                    <Group gap="xs">
                      <ThemeIcon size="lg" variant="light" color="orange">
                        <IconDatabase size={20} />
                      </ThemeIcon>
                      <Text fw={600}>Disk</Text>
                    </Group>
                    <Text size="xl" fw={700}>
                      {health.disk.percent.toFixed(1)}%
                    </Text>
                  </Group>
                  <Progress
                    value={health.disk.percent}
                    color={getProgressColor(health.disk.percent)}
                    size="lg"
                    radius="md"
                  />
                  <Text size="xs" c="dimmed" ta="center">
                    {formatBytes(health.disk.used)} / {formatBytes(health.disk.total)}
                  </Text>
                </Stack>
              </Card>
            </SimpleGrid>

            {/* Temperature Card (if available) */}
            {health.temperature !== undefined && (
              <Card withBorder padding="lg">
                <Group justify="space-between">
                  <Group>
                    <ThemeIcon size="xl" variant="light" color="red">
                      <IconTemperature size={24} />
                    </ThemeIcon>
                    <div>
                      <Text size="sm" c="dimmed">
                        System Temperature
                      </Text>
                      <Text size="xl" fw={700}>
                        {health.temperature.toFixed(1)}°C
                      </Text>
                    </div>
                  </Group>
                  <RingProgress
                    size={120}
                    thickness={12}
                    sections={[
                      {
                        value: (health.temperature / 100) * 100,
                        color: health.temperature > 80 ? 'red' : health.temperature > 60 ? 'yellow' : 'green',
                      },
                    ]}
                    label={
                      <Text size="lg" fw={700} ta="center">
                        {health.temperature.toFixed(0)}°C
                      </Text>
                    }
                  />
                </Group>
              </Card>
            )}

            {/* Version Information */}
            <Card withBorder padding="lg">
              <Stack gap="md">
                <Group>
                  <ThemeIcon size="lg" variant="light" color="indigo">
                    <IconCode size={20} />
                  </ThemeIcon>
                  <Text size="lg" fw={600}>
                    Version Information
                  </Text>
                </Group>
                <Divider />
                <Table>
                  <Table.Tbody>
                    <Table.Tr>
                      <Table.Td>
                        <Group gap="xs">
                          <IconDeviceDesktop size={16} />
                          <Text fw={500}>UI Version</Text>
                        </Group>
                      </Table.Td>
                      <Table.Td>
                        <Badge variant="light" color="blue">
                          {health.versions.ui}
                        </Badge>
                      </Table.Td>
                    </Table.Tr>
                    <Table.Tr>
                      <Table.Td>
                        <Group gap="xs">
                          <IconServer size={16} />
                          <Text fw={500}>Backend Version</Text>
                        </Group>
                      </Table.Td>
                      <Table.Td>
                        <Badge variant="light" color="green">
                          {health.versions.backend}
                        </Badge>
                      </Table.Td>
                    </Table.Tr>
                    {health.versions.ros2 && (
                      <Table.Tr>
                        <Table.Td>
                          <Group gap="xs">
                            <IconBrandPython size={16} />
                            <Text fw={500}>ROS2 Version</Text>
                          </Group>
                        </Table.Td>
                        <Table.Td>
                          <Badge variant="light" color="grape">
                            {health.versions.ros2}
                          </Badge>
                        </Table.Td>
                      </Table.Tr>
                    )}
                  </Table.Tbody>
                </Table>
              </Stack>
            </Card>
          </>
        )}
      </Stack>
    </Container>
  )
}