import { useEffect, useState, useMemo } from 'react'
import {
  Container,
  Title,
  Text,
  Button,
  Group,
  Stack,
  Card,
  Badge,
  Select,
  NumberInput,
  Collapse,
  Alert,
  Loader,
  Center,
  ActionIcon,
  Tooltip,
  Table as MantineTable,
  Pagination,
  Menu,
  TextInput,
} from '@mantine/core'
import {
  IconTarget,
  IconDownload,
  IconFilter,
  IconX,
  IconChevronDown,
  IconFileTypeCsv,
  IconJson,
  IconRefresh,
  IconSearch,
} from '@tabler/icons-react'
import { DatePickerInput } from '@mantine/dates'
import { useAppStore } from '@/app/store'
import { apiClient } from '@/app/services/api'
import { EmptyState } from '@/app/components/common/EmptyState'
import { TrackDetailDrawer } from './TrackDetailDrawer'
import type { Track } from '@/app/services/schema'

// Object classes for filtering
const OBJECT_CLASSES = [
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

export default function Tracks() {
  const {
    tracks,
    filters,
    pagination,
    loading,
    error,
    backendStatus,
    cameras,
    setTracks,
    setFilters,
    setPage,
    setPageSize,
    setLoading,
    setError,
  } = useAppStore()

  const [selectedTrack, setSelectedTrack] = useState<Track | null>(null)
  const [showFilters, setShowFilters] = useState(false)
  const [searchQuery, setSearchQuery] = useState('')

  // Fetch tracks when filters or pagination changes
  useEffect(() => {
    if (backendStatus !== 'connected') return

    const fetchTracks = async () => {
      setLoading(true)
      setError(null)

      try {
        const response = await apiClient.getTracks({
          from: filters.from,
          to: filters.to,
          cameraId: filters.cameraId,
          class: filters.class,
          minConfidence: filters.minConfidence,
          page: pagination.page,
          pageSize: pagination.pageSize,
        })

        setTracks(response.data, response.total, response.hasMore)
      } catch (err) {
        console.error('Failed to fetch tracks:', err)
        setError(err instanceof Error ? err.message : 'Failed to fetch tracks')
      } finally {
        setLoading(false)
      }
    }

    fetchTracks()
  }, [
    filters,
    pagination.page,
    pagination.pageSize,
    backendStatus,
    setTracks,
    setLoading,
    setError,
  ])

  // Filter tracks by search query (client-side)
  const filteredTracks = useMemo(() => {
    if (!searchQuery.trim()) return tracks

    const query = searchQuery.toLowerCase()
    return tracks.filter(
      (track) =>
        track.id.toLowerCase().includes(query) ||
        track.class.toLowerCase().includes(query) ||
        cameras.find((c) => c.id === track.cameraId)?.name.toLowerCase().includes(query)
    )
  }, [tracks, searchQuery, cameras])

  // Export to CSV
  const handleExportCSV = () => {
    const headers = ['Track ID', 'First Seen', 'Last Seen', 'Camera', 'Class', 'Confidence', 'Detections', 'Status']
    const rows = tracks.map((track) => [
      track.id,
      new Date(track.firstSeen).toISOString(),
      new Date(track.lastSeen).toISOString(),
      cameras.find((c) => c.id === track.cameraId)?.name || track.cameraId,
      track.class,
      (track.avgConfidence * 100).toFixed(1) + '%',
      track.detectionCount.toString(),
      track.status,
    ])

    const csv = [headers, ...rows].map((row) => row.join(',')).join('\n')
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = `tracks-${new Date().toISOString().split('T')[0]}.csv`
    link.click()
    URL.revokeObjectURL(url)
  }

  // Export to JSON
  const handleExportJSON = () => {
    const data = tracks.map((track) => ({
      ...track,
      cameraName: cameras.find((c) => c.id === track.cameraId)?.name || track.cameraId,
    }))

    const json = JSON.stringify(data, null, 2)
    const blob = new Blob([json], { type: 'application/json;charset=utf-8;' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = `tracks-${new Date().toISOString().split('T')[0]}.json`
    link.click()
    URL.revokeObjectURL(url)
  }

  // Clear all filters
  const clearFilters = () => {
    setFilters({
      cameraId: undefined,
      class: undefined,
      minConfidence: undefined,
      from: undefined,
      to: undefined,
    })
    setSearchQuery('')
  }

  // Check if any filters are active
  const hasActiveFilters =
    filters.cameraId ||
    filters.class ||
    filters.minConfidence !== undefined ||
    filters.from ||
    filters.to ||
    searchQuery.trim()

  // Handle retry
  const handleRetry = () => {
    window.location.reload()
  }

  // Loading state
  if (backendStatus === 'connecting') {
    return (
      <Center h={400}>
        <Stack align="center" gap="md">
          <Loader size="lg" />
          <Text c="dimmed">Connecting to backend...</Text>
        </Stack>
      </Center>
    )
  }

  // Disconnected state
  if (backendStatus === 'disconnected') {
    return (
      <Container size="sm" py="xl">
        <EmptyState
          icon={<IconTarget size={48} stroke={1.5} />}
          title="Backend Disconnected"
          description="Unable to load tracks. Please check your connection and try again."
          action={
            <Button leftSection={<IconRefresh size={16} />} onClick={handleRetry}>
              Retry Connection
            </Button>
          }
        />
      </Container>
    )
  }

  return (
    <Container size="xl" py="md">
      <Stack gap="lg">
        {/* Header */}
        <Group justify="space-between" align="flex-start">
          <div>
            <Title order={2}>Tracks & Detections</Title>
            <Text size="sm" c="dimmed" mt={4}>
              Browse and analyze detected objects across all cameras
            </Text>
          </div>

          <Group gap="xs">
            <Button
              variant={showFilters ? 'filled' : 'default'}
              leftSection={<IconFilter size={16} />}
              onClick={() => setShowFilters(!showFilters)}
              rightSection={
                hasActiveFilters && (
                  <Badge size="xs" circle color="blue">
                    !
                  </Badge>
                )
              }
            >
              Filters
            </Button>

            <Menu position="bottom-end" shadow="md">
              <Menu.Target>
                <Button
                  variant="default"
                  leftSection={<IconDownload size={16} />}
                  rightSection={<IconChevronDown size={14} />}
                  disabled={tracks.length === 0}
                >
                  Export
                </Button>
              </Menu.Target>
              <Menu.Dropdown>
                <Menu.Item
                  leftSection={<IconFileTypeCsv size={16} />}
                  onClick={handleExportCSV}
                >
                  Export as CSV
                </Menu.Item>
                <Menu.Item
                  leftSection={<IconJson size={16} />}
                  onClick={handleExportJSON}
                >
                  Export as JSON
                </Menu.Item>
              </Menu.Dropdown>
            </Menu>
          </Group>
        </Group>

        {/* Filters Panel */}
        <Collapse in={showFilters}>
          <Card withBorder>
            <Stack gap="md">
              <Group justify="space-between">
                <Text fw={600} size="sm">
                  Filter Tracks
                </Text>
                {hasActiveFilters && (
                  <Button
                    variant="subtle"
                    size="xs"
                    leftSection={<IconX size={14} />}
                    onClick={clearFilters}
                  >
                    Clear all
                  </Button>
                )}
              </Group>

              <Group grow align="flex-start">
                {/* Search */}
                <TextInput
                  label="Search"
                  placeholder="Search by ID, class, or camera..."
                  leftSection={<IconSearch size={16} />}
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  rightSection={
                    searchQuery && (
                      <ActionIcon
                        variant="subtle"
                        size="sm"
                        onClick={() => setSearchQuery('')}
                      >
                        <IconX size={14} />
                      </ActionIcon>
                    )
                  }
                />

                {/* Camera Filter */}
                <Select
                  label="Camera"
                  placeholder="All cameras"
                  data={[
                    { value: '', label: 'All cameras' },
                    ...cameras.map((camera) => ({
                      value: camera.id,
                      label: camera.name,
                    })),
                  ]}
                  value={filters.cameraId || ''}
                  onChange={(value) =>
                    setFilters({ ...filters, cameraId: value || undefined })
                  }
                  clearable
                />

                {/* Class Filter */}
                <Select
                  label="Object Class"
                  placeholder="All classes"
                  data={[
                    { value: '', label: 'All classes' },
                    ...OBJECT_CLASSES.map((cls) => ({
                      value: cls,
                      label: cls.charAt(0).toUpperCase() + cls.slice(1),
                    })),
                  ]}
                  value={filters.class || ''}
                  onChange={(value) =>
                    setFilters({ ...filters, class: value || undefined })
                  }
                  clearable
                  searchable
                />

                {/* Min Confidence */}
                <NumberInput
                  label="Min Confidence (%)"
                  placeholder="0-100"
                  min={0}
                  max={100}
                  step={5}
                  value={
                    filters.minConfidence !== undefined
                      ? filters.minConfidence * 100
                      : undefined
                  }
                  onChange={(value) =>
                    setFilters({
                      ...filters,
                      minConfidence:
                        typeof value === 'number' ? value / 100 : undefined,
                    })
                  }
                  suffix="%"
                  allowDecimal={false}
                />
              </Group>

              <Group grow align="flex-start">
                {/* Date Range */}
                <DatePickerInput
                  label="From Date"
                  placeholder="Start date"
                  value={filters.from ? new Date(filters.from) : null}
                  onChange={(date) =>
                    setFilters({
                      ...filters,
                      from: date ? date.toISOString() : undefined,
                    })
                  }
                  clearable
                  maxDate={filters.to ? new Date(filters.to) : new Date()}
                />

                <DatePickerInput
                  label="To Date"
                  placeholder="End date"
                  value={filters.to ? new Date(filters.to) : null}
                  onChange={(date) =>
                    setFilters({
                      ...filters,
                      to: date ? date.toISOString() : undefined,
                    })
                  }
                  clearable
                  minDate={filters.from ? new Date(filters.from) : undefined}
                  maxDate={new Date()}
                />
              </Group>
            </Stack>
          </Card>
        </Collapse>

        {/* Error Alert */}
        {error && (
          <Alert color="red" title="Error loading tracks" withCloseButton onClose={() => setError(null)}>
            {error}
          </Alert>
        )}

        {/* Active Filters Summary */}
        {hasActiveFilters && (
          <Group gap="xs">
            <Text size="sm" c="dimmed">
              Active filters:
            </Text>
            {searchQuery && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setSearchQuery('')}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Search: {searchQuery}
              </Badge>
            )}
            {filters.cameraId && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, cameraId: undefined })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Camera: {cameras.find((c) => c.id === filters.cameraId)?.name}
              </Badge>
            )}
            {filters.class && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, class: undefined })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Class: {filters.class}
              </Badge>
            )}
            {filters.minConfidence !== undefined && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() =>
                      setFilters({ ...filters, minConfidence: undefined })
                    }
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Min Confidence: {(filters.minConfidence * 100).toFixed(0)}%
              </Badge>
            )}
            {(filters.from || filters.to) && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() =>
                      setFilters({ ...filters, from: undefined, to: undefined })
                    }
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Date Range
              </Badge>
            )}
          </Group>
        )}

        {/* Tracks Table */}
        <Card withBorder padding={0}>
          {loading ? (
            <Center p="xl">
              <Stack align="center" gap="md">
                <Loader size="md" />
                <Text size="sm" c="dimmed">
                  Loading tracks...
                </Text>
              </Stack>
            </Center>
          ) : filteredTracks.length === 0 ? (
            <EmptyState
              icon={<IconTarget size={48} stroke={1.5} />}
              title={hasActiveFilters ? 'No tracks found' : 'No tracks yet'}
              description={
                hasActiveFilters
                  ? 'Try adjusting your filters to see more results.'
                  : 'Tracks will appear here when objects are detected by your cameras.'
              }
              action={
                hasActiveFilters ? (
                  <Button variant="light" onClick={clearFilters}>
                    Clear Filters
                  </Button>
                ) : undefined
              }
            />
          ) : (
            <>
              <MantineTable.ScrollContainer minWidth={800}>
                <MantineTable highlightOnHover>
                  <MantineTable.Thead>
                    <MantineTable.Tr>
                      <MantineTable.Th>Track ID</MantineTable.Th>
                      <MantineTable.Th>First Seen</MantineTable.Th>
                      <MantineTable.Th>Last Seen</MantineTable.Th>
                      <MantineTable.Th>Camera</MantineTable.Th>
                      <MantineTable.Th>Class</MantineTable.Th>
                      <MantineTable.Th>Confidence</MantineTable.Th>
                      <MantineTable.Th>Detections</MantineTable.Th>
                      <MantineTable.Th>Status</MantineTable.Th>
                    </MantineTable.Tr>
                  </MantineTable.Thead>
                  <MantineTable.Tbody>
                    {filteredTracks.map((track) => {
                      const camera = cameras.find((c) => c.id === track.cameraId)
                      const firstSeen = new Date(track.firstSeen)
                      const lastSeen = new Date(track.lastSeen)

                      return (
                        <MantineTable.Tr
                          key={track.id}
                          style={{ cursor: 'pointer' }}
                          onClick={() => setSelectedTrack(track)}
                        >
                          <MantineTable.Td>
                            <Text size="sm" ff="monospace">
                              {track.id.slice(0, 8)}...
                            </Text>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <div>
                              <Text size="sm">{firstSeen.toLocaleDateString()}</Text>
                              <Text size="xs" c="dimmed">
                                {firstSeen.toLocaleTimeString()}
                              </Text>
                            </div>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <div>
                              <Text size="sm">{lastSeen.toLocaleDateString()}</Text>
                              <Text size="xs" c="dimmed">
                                {lastSeen.toLocaleTimeString()}
                              </Text>
                            </div>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <Text size="sm">{camera?.name || track.cameraId}</Text>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <Badge variant="light" color="blue">
                              {track.class}
                            </Badge>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <Tooltip
                              label={`${(track.avgConfidence * 100).toFixed(2)}%`}
                            >
                              <Text size="sm" fw={500}>
                                {(track.avgConfidence * 100).toFixed(1)}%
                              </Text>
                            </Tooltip>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <Text size="sm">{track.detectionCount}</Text>
                          </MantineTable.Td>
                          <MantineTable.Td>
                            <Badge
                              color={
                                track.status === 'active'
                                  ? 'green'
                                  : track.status === 'lost'
                                  ? 'yellow'
                                  : 'gray'
                              }
                              variant="light"
                            >
                              {track.status}
                            </Badge>
                          </MantineTable.Td>
                        </MantineTable.Tr>
                      )
                    })}
                  </MantineTable.Tbody>
                </MantineTable>
              </MantineTable.ScrollContainer>

              {/* Pagination */}
              {pagination.total > pagination.pageSize && (
                <Group justify="space-between" p="md" style={{ borderTop: '1px solid var(--mantine-color-gray-3)' }}>
                  <Text size="sm" c="dimmed">
                    Showing {(pagination.page - 1) * pagination.pageSize + 1} to{' '}
                    {Math.min(pagination.page * pagination.pageSize, pagination.total)} of{' '}
                    {pagination.total} tracks
                  </Text>

                  <Group gap="xs">
                    <Select
                      size="xs"
                      value={pagination.pageSize.toString()}
                      onChange={(value) => setPageSize(Number(value))}
                      data={[
                        { value: '25', label: '25 per page' },
                        { value: '50', label: '50 per page' },
                        { value: '100', label: '100 per page' },
                      ]}
                      w={130}
                    />

                    <Pagination
                      size="sm"
                      value={pagination.page}
                      onChange={setPage}
                      total={Math.ceil(pagination.total / pagination.pageSize)}
                    />
                  </Group>
                </Group>
              )}
            </>
          )}
        </Card>

        {/* Statistics Summary */}
        {!loading && filteredTracks.length > 0 && (
          <Group grow>
            <Card withBorder>
              <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                Total Tracks
              </Text>
              <Text size="xl" fw={700} mt="xs">
                {pagination.total.toLocaleString()}
              </Text>
            </Card>

            <Card withBorder>
              <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                Active Tracks
              </Text>
              <Text size="xl" fw={700} mt="xs" c="green">
                {filteredTracks.filter((t) => t.status === 'active').length}
              </Text>
            </Card>

            <Card withBorder>
              <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                Avg Confidence
              </Text>
              <Text size="xl" fw={700} mt="xs">
                {filteredTracks.length > 0
                  ? (
                      (filteredTracks.reduce((sum, t) => sum + t.avgConfidence, 0) /
                        filteredTracks.length) *
                      100
                    ).toFixed(1)
                  : 0}
                %
              </Text>
            </Card>

            <Card withBorder>
              <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                Total Detections
              </Text>
              <Text size="xl" fw={700} mt="xs">
                {filteredTracks
                  .reduce((sum, t) => sum + t.detectionCount, 0)
                  .toLocaleString()}
              </Text>
            </Card>
          </Group>
        )}
      </Stack>

      {/* Track Detail Drawer */}
      {selectedTrack && (
        <TrackDetailDrawer
          track={selectedTrack}
          opened={!!selectedTrack}
          onClose={() => setSelectedTrack(null)}
        />
      )}
    </Container>
  )
}