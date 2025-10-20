import { useEffect, useState } from 'react'
import {
  Container,
  Title,
  Stack,
  Card,
  Group,
  Button,
  Badge,
  Text,
  Select,
  Input,
  Loader,
  Center,
  Table,
  ActionIcon,
  CopyButton,
  Tooltip,
} from '@mantine/core'
import {
  IconRefresh,
  IconClock,
  IconCopy,
  IconCheck,
  IconFilter,
} from '@tabler/icons-react'
import { apiClient } from '@/app/services/api'
import type { LogEntry } from '@/app/services/schema'
import { logsClient } from '@/app/services/ws'

export default function Logs() {
  const [logs, setLogs] = useState<LogEntry[]>([])
  const [isLoading, setIsLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date())
  const [levelFilter, setLevelFilter] = useState<string | null>(null)
  const [searchText, setSearchText] = useState('')

  // Fetch logs on mount
  useEffect(() => {
    fetchLogs()
  }, [])

  // Subscribe to real-time logs via WebSocket
  useEffect(() => {
    try {
      logsClient.connect('/logs')
      const unsubscribe = logsClient.onEvent((event) => {
        if (event.type === 'log') {
          setLogs((prevLogs) => [event.data, ...prevLogs.slice(0, 99)])
          setLastUpdate(new Date())
        }
      })
      return () => {
        unsubscribe()
        logsClient.disconnect()
      }
    } catch (err) {
      console.warn('WebSocket connection failed, using polling instead')
    }
  }, [])

  const fetchLogs = async () => {
    try {
      setIsLoading(true)
      const data = await apiClient.getLogs({ tail: 100 })
      setLogs(data)
      setError(null)
      setLastUpdate(new Date())
    } catch (err) {
      setError((err as Error).message)
      // Show placeholder data when backend is offline
      setLogs([
        {
          timestamp: new Date().toISOString(),
          level: 'info',
          message: 'Backend is offline - showing placeholder logs',
          source: 'system',
        },
      ])
    } finally {
      setIsLoading(false)
    }
  }

  // Filter logs based on level and search text
  const filteredLogs = logs.filter((log) => {
    if (levelFilter && log.level !== levelFilter) return false
    if (searchText && !log.message.toLowerCase().includes(searchText.toLowerCase())) {
      return false
    }
    return true
  })

  const getLevelColor = (level: string) => {
    switch (level) {
      case 'debug':
        return 'gray'
      case 'info':
        return 'blue'
      case 'warn':
        return 'yellow'
      case 'error':
        return 'red'
      default:
        return 'gray'
    }
  }

  const formatTimestamp = (timestamp: string) => {
    try {
      const date = new Date(timestamp)
      return date.toLocaleString()
    } catch {
      return timestamp
    }
  }

  return (
    <Container size="xl" py="xl">
      <Stack gap="lg">
        {/* Header */}
        <Group justify="space-between" align="flex-start">
          <Stack gap="xs" flex={1}>
            <Title order={2}>System Logs</Title>
            <Text size="sm" c="dimmed">
              Last updated: {lastUpdate.toLocaleTimeString()}
            </Text>
          </Stack>
          <Button
            leftSection={<IconRefresh size={16} />}
            onClick={fetchLogs}
            loading={isLoading}
            variant="default"
          >
            Refresh
          </Button>
        </Group>

        {/* Error Alert */}
        {error && (
          <Card bg="rgba(255, 0, 0, 0.05)" p="md" radius="md">
            <Group>
              <Text c="red" size="sm">
                ⚠️ {error}
              </Text>
            </Group>
          </Card>
        )}

        {/* Filters */}
        <Card withBorder p="md" radius="md">
          <Stack gap="sm">
            <Text fw={500} size="sm">
              <IconFilter size={16} style={{ display: 'inline', marginRight: 8 }} />
              Filters
            </Text>
            <Group grow>
              <Select
                placeholder="Filter by level"
                data={[
                  { value: 'debug', label: 'Debug' },
                  { value: 'info', label: 'Info' },
                  { value: 'warn', label: 'Warning' },
                  { value: 'error', label: 'Error' },
                ]}
                value={levelFilter}
                onChange={setLevelFilter}
                clearable
              />
              <Input
                placeholder="Search logs..."
                value={searchText}
                onChange={(e) => setSearchText(e.currentTarget.value)}
              />
            </Group>
          </Stack>
        </Card>

        {/* Logs Table */}
        <Card withBorder radius="md" p="md">
          <Stack gap="md">
            <Group justify="space-between">
              <Text fw={500}>
                Showing {filteredLogs.length} of {logs.length} logs
              </Text>
            </Group>

            {isLoading ? (
              <Center py="xl">
                <Loader />
              </Center>
            ) : filteredLogs.length === 0 ? (
              <Center py="xl">
                <Text c="dimmed">No logs found</Text>
              </Center>
            ) : (
              <div style={{ overflowX: 'auto' }}>
                <Table striped highlightOnHover>
                  <Table.Thead>
                    <Table.Tr>
                      <Table.Th>Timestamp</Table.Th>
                      <Table.Th>Level</Table.Th>
                      <Table.Th>Source</Table.Th>
                      <Table.Th>Message</Table.Th>
                      <Table.Th align="right">Action</Table.Th>
                    </Table.Tr>
                  </Table.Thead>
                  <Table.Tbody>
                    {filteredLogs.map((log, idx) => (
                      <Table.Tr key={idx}>
                        <Table.Td>
                          <Group gap="xs">
                            <IconClock size={14} color="gray" />
                            <Text size="sm">{formatTimestamp(log.timestamp)}</Text>
                          </Group>
                        </Table.Td>
                        <Table.Td>
                          <Badge color={getLevelColor(log.level)} variant="light" size="sm">
                            {log.level.toUpperCase()}
                          </Badge>
                        </Table.Td>
                        <Table.Td>
                          <Text size="sm" c="dimmed">
                            {log.source || '-'}
                          </Text>
                        </Table.Td>
                        <Table.Td>
                          <Text size="sm" style={{ maxWidth: 400, wordWrap: 'break-word' }}>
                            {log.message}
                          </Text>
                        </Table.Td>
                        <Table.Td align="right">
                          <CopyButton value={log.message} timeout={2000}>
                            {({ copied }) => (
                              <Tooltip
                                label={copied ? 'Copied' : 'Copy message'}
                                withArrow
                                position="left"
                              >
                                <ActionIcon
                                  color={copied ? 'teal' : 'gray'}
                                  variant="subtle"
                                  onClick={() => {}}
                                >
                                  {copied ? (
                                    <IconCheck size={16} />
                                  ) : (
                                    <IconCopy size={16} />
                                  )}
                                </ActionIcon>
                              </Tooltip>
                            )}
                          </CopyButton>
                        </Table.Td>
                      </Table.Tr>
                    ))}
                  </Table.Tbody>
                </Table>
              </div>
            )}
          </Stack>
        </Card>
      </Stack>
    </Container>
  )
}