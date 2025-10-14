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
  Collapse,
  Alert,
  Loader,
  Center,
  ActionIcon,
  Tooltip,
  Pagination,
  Menu,
  TextInput,
  Modal,
  SegmentedControl,
  Progress,
  Grid,
  Table,
  AspectRatio,
  ThemeIcon,
  SimpleGrid,
} from '@mantine/core'
import {
  IconVideo,
  IconDownload,
  IconFilter,
  IconX,
  IconChevronDown,
  IconRefresh,
  IconSearch,
  IconTrash,
  IconPlayerPlay,
  IconStar,
  IconStarFilled,
  IconCalendar,
  IconCamera,
  IconClock,
  IconFile,
  IconGridDots,
  IconList,
  IconAlertCircle,
  IconDatabase,
} from '@tabler/icons-react'
import { DatePickerInput } from '@mantine/dates'
import { useAppStore } from '@/app/store'
import { apiClient } from '@/app/services/api'
import { EmptyState } from '@/app/components/common/EmptyState'
import { formatBytes, formatDuration } from '@/lib/utils'
import type { Recording } from '@/app/services/schema'

type ViewMode = 'grid' | 'list'
type SortField = 'startTime' | 'duration' | 'fileSize' | 'detectionCount'
type SortOrder = 'asc' | 'desc'

export default function Recordings() {
  const [recordings, setRecordings] = useState<Recording[]>([])
  const [isLoading, setIsLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [showFilters, setShowFilters] = useState(false)
  const [viewMode, setViewMode] = useState<ViewMode>('grid')
  const [selectedRecording, setSelectedRecording] = useState<Recording | null>(null)
  const [videoModalOpen, setVideoModalOpen] = useState(false)

  // Pagination
  const [page, setPage] = useState(1)
  const [pageSize, setPageSize] = useState(12)
  const [totalRecordings, setTotalRecordings] = useState(0)

  // Filters
  const [filters, setFilters] = useState({
    cameraId: '',
    startDate: null as Date | null,
    endDate: null as Date | null,
    bookmarked: '',
    search: '',
  })

  // Sorting
  const [sortField, setSortField] = useState<SortField>('startTime')
  const [sortOrder, setSortOrder] = useState<SortOrder>('desc')

  const cameras = useAppStore((state) => state.cameras)

  useEffect(() => {
    fetchRecordings()
  }, [page, pageSize, sortField, sortOrder])

  useEffect(() => {
    // Reset to page 1 when filters change
    if (page !== 1) {
      setPage(1)
    } else {
      fetchRecordings()
    }
  }, [filters])

  const fetchRecordings = async () => {
    setIsLoading(true)
    setError(null)
    try {
      const params: any = {
        page,
        pageSize,
      }

      if (filters.cameraId) params.cameraId = filters.cameraId
      if (filters.startDate) params.from = filters.startDate.toISOString()
      if (filters.endDate) params.to = filters.endDate.toISOString()

      const response = await apiClient.getRecordings(params)
      setRecordings(response.items)
      setTotalRecordings(response.total)
    } catch (err) {
      setError((err as Error).message)
      setRecordings([])
    } finally {
      setIsLoading(false)
    }
  }

  // Client-side filtering for search and bookmarked
  const filteredRecordings = useMemo(() => {
    let filtered = [...recordings]

    // Search filter
    if (filters.search) {
      const searchLower = filters.search.toLowerCase()
      filtered = filtered.filter(
        (r) =>
          r.filename?.toLowerCase().includes(searchLower) ||
          r.id.toLowerCase().includes(searchLower) ||
          cameras.find((c) => c.id === r.cameraId)?.name.toLowerCase().includes(searchLower)
      )
    }

    // Bookmarked filter
    if (filters.bookmarked === 'true') {
      filtered = filtered.filter((r) => r.bookmarked)
    } else if (filters.bookmarked === 'false') {
      filtered = filtered.filter((r) => !r.bookmarked)
    }

    return filtered
  }, [recordings, filters.search, filters.bookmarked, cameras])

  // Statistics
  const stats = useMemo(() => {
    const totalSize = recordings.reduce((sum, r) => sum + r.fileSize, 0)
    const totalDuration = recordings.reduce((sum, r) => sum + r.duration, 0)
    const bookmarkedCount = recordings.filter((r) => r.bookmarked).length

    return {
      total: totalRecordings,
      totalSize,
      totalDuration,
      bookmarkedCount,
    }
  }, [recordings, totalRecordings])

  const handleBookmark = async (recording: Recording) => {
    try {
      await apiClient.bookmarkRecording(recording.id, !recording.bookmarked)
      setRecordings((prev) =>
        prev.map((r) => (r.id === recording.id ? { ...r, bookmarked: !r.bookmarked } : r))
      )
    } catch (err) {
      setError(`Failed to ${recording.bookmarked ? 'unbookmark' : 'bookmark'} recording`)
    }
  }

  const handleDownload = (recording: Recording) => {
    const a = document.createElement('a')
    a.href = recording.url
    a.download = recording.filename || `recording-${recording.id}.mp4`
    a.click()
  }

  const handleDelete = async (recording: Recording) => {
    if (!confirm(`Are you sure you want to delete "${recording.filename || 'this recording'}"?`)) {
      return
    }

    try {
      await apiClient.deleteRecording(recording.id)
      fetchRecordings()
    } catch (err) {
      setError('Failed to delete recording')
    }
  }

  const handlePlayVideo = (recording: Recording) => {
    setSelectedRecording(recording)
    setVideoModalOpen(true)
  }

  const clearFilters = () => {
    setFilters({
      cameraId: '',
      startDate: null,
      endDate: null,
      bookmarked: '',
      search: '',
    })
  }

  const activeFilterCount = [
    filters.cameraId,
    filters.startDate,
    filters.endDate,
    filters.bookmarked,
    filters.search,
  ].filter(Boolean).length

  const totalPages = Math.ceil(totalRecordings / pageSize)

  if (isLoading && recordings.length === 0) {
    return (
      <Center h={400}>
        <Stack align="center" gap="md">
          <Loader size="lg" />
          <Text c="dimmed">Loading recordings...</Text>
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
            <Title order={2}>Recordings</Title>
            <Text c="dimmed" size="sm">
              Browse and manage recorded video clips
            </Text>
          </div>
          <Group>
            <SegmentedControl
              value={viewMode}
              onChange={(value) => setViewMode(value as ViewMode)}
              data={[
                { label: <IconGridDots size={16} />, value: 'grid' },
                { label: <IconList size={16} />, value: 'list' },
              ]}
            />
            <Button
              leftSection={<IconRefresh size={16} />}
              variant="light"
              onClick={fetchRecordings}
              loading={isLoading}
            >
              Refresh
            </Button>
          </Group>
        </Group>

        {/* Statistics Cards */}
        <SimpleGrid cols={{ base: 1, sm: 2, md: 4 }}>
          <Card withBorder>
            <Group justify="space-between">
              <div>
                <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                  Total Recordings
                </Text>
                <Text size="xl" fw={700}>
                  {stats.total}
                </Text>
              </div>
              <ThemeIcon size="xl" variant="light" color="blue">
                <IconVideo size={24} />
              </ThemeIcon>
            </Group>
          </Card>

          <Card withBorder>
            <Group justify="space-between">
              <div>
                <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                  Total Size
                </Text>
                <Text size="xl" fw={700}>
                  {formatBytes(stats.totalSize)}
                </Text>
              </div>
              <ThemeIcon size="xl" variant="light" color="cyan">
                <IconDatabase size={24} />
              </ThemeIcon>
            </Group>
          </Card>

          <Card withBorder>
            <Group justify="space-between">
              <div>
                <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                  Total Duration
                </Text>
                <Text size="xl" fw={700}>
                  {formatDuration(stats.totalDuration)}
                </Text>
              </div>
              <ThemeIcon size="xl" variant="light" color="grape">
                <IconClock size={24} />
              </ThemeIcon>
            </Group>
          </Card>

          <Card withBorder>
            <Group justify="space-between">
              <div>
                <Text size="xs" c="dimmed" tt="uppercase" fw={700}>
                  Bookmarked
                </Text>
                <Text size="xl" fw={700}>
                  {stats.bookmarkedCount}
                </Text>
              </div>
              <ThemeIcon size="xl" variant="light" color="yellow">
                <IconStarFilled size={24} />
              </ThemeIcon>
            </Group>
          </Card>
        </SimpleGrid>

        {/* Filters */}
        <Card withBorder>
          <Stack gap="md">
            <Group justify="space-between">
              <Group>
                <Button
                  leftSection={<IconFilter size={16} />}
                  variant={showFilters ? 'filled' : 'light'}
                  onClick={() => setShowFilters(!showFilters)}
                  rightSection={
                    activeFilterCount > 0 ? (
                      <Badge size="sm" circle>
                        {activeFilterCount}
                      </Badge>
                    ) : (
                      <IconChevronDown
                        size={16}
                        style={{
                          transform: showFilters ? 'rotate(180deg)' : 'rotate(0deg)',
                          transition: 'transform 200ms',
                        }}
                      />
                    )
                  }
                >
                  Filters
                </Button>
                {activeFilterCount > 0 && (
                  <Button
                    variant="subtle"
                    color="gray"
                    size="sm"
                    onClick={clearFilters}
                    leftSection={<IconX size={14} />}
                  >
                    Clear all
                  </Button>
                )}
              </Group>

              <Group>
                <Select
                  placeholder="Sort by"
                  value={sortField}
                  onChange={(value) => setSortField(value as SortField)}
                  data={[
                    { value: 'startTime', label: 'Date' },
                    { value: 'duration', label: 'Duration' },
                    { value: 'fileSize', label: 'Size' },
                    { value: 'detectionCount', label: 'Detections' },
                  ]}
                  w={150}
                />
                <ActionIcon
                  variant="light"
                  onClick={() => setSortOrder(sortOrder === 'asc' ? 'desc' : 'asc')}
                >
                  {sortOrder === 'asc' ? '↑' : '↓'}
                </ActionIcon>
              </Group>
            </Group>

            <Collapse in={showFilters}>
              <Grid>
                <Grid.Col span={{ base: 12, sm: 6, md: 3 }}>
                  <TextInput
                    placeholder="Search recordings..."
                    leftSection={<IconSearch size={16} />}
                    value={filters.search}
                    onChange={(e) => setFilters({ ...filters, search: e.target.value })}
                    rightSection={
                      filters.search && (
                        <ActionIcon
                          variant="subtle"
                          onClick={() => setFilters({ ...filters, search: '' })}
                        >
                          <IconX size={14} />
                        </ActionIcon>
                      )
                    }
                  />
                </Grid.Col>

                <Grid.Col span={{ base: 12, sm: 6, md: 3 }}>
                  <Select
                    placeholder="All cameras"
                    leftSection={<IconCamera size={16} />}
                    value={filters.cameraId}
                    onChange={(value) => setFilters({ ...filters, cameraId: value || '' })}
                    data={[
                      { value: '', label: 'All cameras' },
                      ...cameras.map((c) => ({ value: c.id, label: c.name })),
                    ]}
                    clearable
                  />
                </Grid.Col>

                <Grid.Col span={{ base: 12, sm: 6, md: 3 }}>
                  <DatePickerInput
                    placeholder="Start date"
                    leftSection={<IconCalendar size={16} />}
                    value={filters.startDate}
                    onChange={(value) => setFilters({ ...filters, startDate: value })}
                    clearable
                  />
                </Grid.Col>

                <Grid.Col span={{ base: 12, sm: 6, md: 3 }}>
                  <DatePickerInput
                    placeholder="End date"
                    leftSection={<IconCalendar size={16} />}
                    value={filters.endDate}
                    onChange={(value) => setFilters({ ...filters, endDate: value })}
                    clearable
                  />
                </Grid.Col>

                <Grid.Col span={{ base: 12, sm: 6, md: 3 }}>
                  <Select
                    placeholder="Bookmarked"
                    leftSection={<IconStar size={16} />}
                    value={filters.bookmarked}
                    onChange={(value) => setFilters({ ...filters, bookmarked: value || '' })}
                    data={[
                      { value: '', label: 'All recordings' },
                      { value: 'true', label: 'Bookmarked only' },
                      { value: 'false', label: 'Not bookmarked' },
                    ]}
                    clearable
                  />
                </Grid.Col>
              </Grid>
            </Collapse>
          </Stack>
        </Card>

        {/* Error Alert */}
        {error && (
          <Alert
            icon={<IconAlertCircle size={16} />}
            title="Error"
            color="red"
            withCloseButton
            onClose={() => setError(null)}
          >
            {error}
          </Alert>
        )}

        {/* Active Filters */}
        {activeFilterCount > 0 && (
          <Group gap="xs">
            <Text size="sm" c="dimmed">
              Active filters:
            </Text>
            {filters.cameraId && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, cameraId: '' })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Camera: {cameras.find((c) => c.id === filters.cameraId)?.name || filters.cameraId}
              </Badge>
            )}
            {filters.startDate && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, startDate: null })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                From: {filters.startDate.toLocaleDateString()}
              </Badge>
            )}
            {filters.endDate && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, endDate: null })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                To: {filters.endDate.toLocaleDateString()}
              </Badge>
            )}
            {filters.bookmarked && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, bookmarked: '' })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                {filters.bookmarked === 'true' ? 'Bookmarked' : 'Not bookmarked'}
              </Badge>
            )}
            {filters.search && (
              <Badge
                variant="light"
                rightSection={
                  <ActionIcon
                    size="xs"
                    variant="transparent"
                    onClick={() => setFilters({ ...filters, search: '' })}
                  >
                    <IconX size={10} />
                  </ActionIcon>
                }
              >
                Search: {filters.search}
              </Badge>
            )}
          </Group>
        )}

        {/* Recordings Content */}
        {filteredRecordings.length === 0 ? (
          <EmptyState
            icon={<IconVideo size={48} />}
            title="No recordings found"
            description={
              activeFilterCount > 0
                ? 'Try adjusting your filters to see more results'
                : 'Recordings will appear here when video clips are saved'
            }
          />
        ) : viewMode === 'grid' ? (
          <SimpleGrid cols={{ base: 1, sm: 2, md: 3, lg: 4 }}>
            {filteredRecordings.map((recording) => (
              <RecordingCard
                key={recording.id}
                recording={recording}
                cameras={cameras}
                onBookmark={handleBookmark}
                onDownload={handleDownload}
                onDelete={handleDelete}
                onPlay={handlePlayVideo}
              />
            ))}
          </SimpleGrid>
        ) : (
          <RecordingsTable
            recordings={filteredRecordings}
            cameras={cameras}
            onBookmark={handleBookmark}
            onDownload={handleDownload}
            onDelete={handleDelete}
            onPlay={handlePlayVideo}
          />
        )}

        {/* Pagination */}
        {totalPages > 1 && (
          <Group justify="space-between">
            <Group>
              <Text size="sm" c="dimmed">
                Showing {(page - 1) * pageSize + 1}-
                {Math.min(page * pageSize, totalRecordings)} of {totalRecordings}
              </Text>
              <Select
                value={pageSize.toString()}
                onChange={(value) => {
                  setPageSize(Number(value))
                  setPage(1)
                }}
                data={[
                  { value: '12', label: '12 per page' },
                  { value: '24', label: '24 per page' },
                  { value: '48', label: '48 per page' },
                  { value: '96', label: '96 per page' },
                ]}
                w={150}
              />
            </Group>
            <Pagination value={page} onChange={setPage} total={totalPages} />
          </Group>
        )}
      </Stack>

      {/* Video Player Modal */}
      <Modal
        opened={videoModalOpen}
        onClose={() => setVideoModalOpen(false)}
        title={selectedRecording?.filename || 'Recording'}
        size="xl"
        centered
      >
        {selectedRecording && (
          <Stack gap="md">
            <AspectRatio ratio={16 / 9}>
              <video
                src={selectedRecording.url}
                controls
                autoPlay
                style={{ width: '100%', height: '100%', backgroundColor: '#000' }}
              />
            </AspectRatio>
            <Group justify="space-between">
              <div>
                <Text size="sm" c="dimmed">
                  Duration: {formatDuration(selectedRecording.duration)}
                </Text>
                <Text size="sm" c="dimmed">
                  Size: {formatBytes(selectedRecording.fileSize)}
                </Text>
                <Text size="sm" c="dimmed">
                  Detections: {selectedRecording.detectionCount}
                </Text>
              </div>
              <Group>
                <Button
                  leftSection={<IconDownload size={16} />}
                  variant="light"
                  onClick={() => handleDownload(selectedRecording)}
                >
                  Download
                </Button>
              </Group>
            </Group>
          </Stack>
        )}
      </Modal>
    </Container>
  )
}

// Recording Card Component
interface RecordingCardProps {
  recording: Recording
  cameras: any[]
  onBookmark: (recording: Recording) => void
  onDownload: (recording: Recording) => void
  onDelete: (recording: Recording) => void
  onPlay: (recording: Recording) => void
}

function RecordingCard({
  recording,
  cameras,
  onBookmark,
  onDownload,
  onDelete,
  onPlay,
}: RecordingCardProps) {
  const camera = cameras.find((c) => c.id === recording.cameraId)

  return (
    <Card withBorder padding="md" radius="md">
      <Card.Section>
        <AspectRatio ratio={16 / 9}>
          <div
            style={{
              position: 'relative',
              width: '100%',
              height: '100%',
              backgroundColor: '#000',
              cursor: 'pointer',
            }}
            onClick={() => onPlay(recording)}
          >
            {recording.thumbnailUrl ? (
              <img
                src={recording.thumbnailUrl}
                alt={recording.filename || 'Recording'}
                style={{ width: '100%', height: '100%', objectFit: 'cover' }}
              />
            ) : (
              <Center h="100%">
                <IconVideo size={48} color="#666" />
              </Center>
            )}
            <div
              style={{
                position: 'absolute',
                top: 8,
                right: 8,
              }}
            >
              <ActionIcon
                variant="filled"
                color={recording.bookmarked ? 'yellow' : 'gray'}
                onClick={(e) => {
                  e.stopPropagation()
                  onBookmark(recording)
                }}
              >
                {recording.bookmarked ? <IconStarFilled size={16} /> : <IconStar size={16} />}
              </ActionIcon>
            </div>
            <div
              style={{
                position: 'absolute',
                bottom: 8,
                right: 8,
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                padding: '4px 8px',
                borderRadius: 4,
              }}
            >
              <Text size="xs" c="white">
                {formatDuration(recording.duration)}
              </Text>
            </div>
            <div
              style={{
                position: 'absolute',
                top: '50%',
                left: '50%',
                transform: 'translate(-50%, -50%)',
              }}
            >
              <ActionIcon size="xl" variant="filled" color="blue" radius="xl">
                <IconPlayerPlay size={24} />
              </ActionIcon>
            </div>
          </div>
        </AspectRatio>
      </Card.Section>

      <Stack gap="xs" mt="md">
        <Text fw={600} lineClamp={1}>
          {recording.filename || `Recording ${recording.id.slice(0, 8)}`}
        </Text>

        <Group gap="xs">
          <IconCalendar size={14} color="gray" />
          <Text size="xs" c="dimmed">
            {new Date(recording.startTime).toLocaleString()}
          </Text>
        </Group>

        {camera && (
          <Group gap="xs">
            <IconCamera size={14} color="gray" />
            <Text size="xs" c="dimmed">
              {camera.name}
            </Text>
          </Group>
        )}

        <Group justify="space-between">
          <Text size="xs" c="dimmed">
            {formatBytes(recording.fileSize)}
          </Text>
          <Badge size="sm" variant="light">
            {recording.detectionCount} detections
          </Badge>
        </Group>

        <Group gap="xs" mt="xs">
          <Tooltip label="Download">
            <ActionIcon variant="light" onClick={() => onDownload(recording)}>
              <IconDownload size={16} />
            </ActionIcon>
          </Tooltip>
          <Tooltip label="Delete">
            <ActionIcon variant="light" color="red" onClick={() => onDelete(recording)}>
              <IconTrash size={16} />
            </ActionIcon>
          </Tooltip>
        </Group>
      </Stack>
    </Card>
  )
}

// Recordings Table Component
interface RecordingsTableProps {
  recordings: Recording[]
  cameras: any[]
  onBookmark: (recording: Recording) => void
  onDownload: (recording: Recording) => void
  onDelete: (recording: Recording) => void
  onPlay: (recording: Recording) => void
}

function RecordingsTable({
  recordings,
  cameras,
  onBookmark,
  onDownload,
  onDelete,
  onPlay,
}: RecordingsTableProps) {
  return (
    <Card withBorder>
      <Table striped highlightOnHover>
        <Table.Thead>
          <Table.Tr>
            <Table.Th>Filename</Table.Th>
            <Table.Th>Camera</Table.Th>
            <Table.Th>Date</Table.Th>
            <Table.Th>Duration</Table.Th>
            <Table.Th>Size</Table.Th>
            <Table.Th>Detections</Table.Th>
            <Table.Th>Actions</Table.Th>
          </Table.Tr>
        </Table.Thead>
        <Table.Tbody>
          {recordings.map((recording) => {
            const camera = cameras.find((c) => c.id === recording.cameraId)
            return (
              <Table.Tr key={recording.id}>
                <Table.Td>
                  <Group gap="xs">
                    <ActionIcon
                      size="sm"
                      variant="subtle"
                      color={recording.bookmarked ? 'yellow' : 'gray'}
                      onClick={() => onBookmark(recording)}
                    >
                      {recording.bookmarked ? (
                        <IconStarFilled size={14} />
                      ) : (
                        <IconStar size={14} />
                      )}
                    </ActionIcon>
                    <Text size="sm">{recording.filename || `Recording ${recording.id.slice(0, 8)}`}</Text>
                  </Group>
                </Table.Td>
                <Table.Td>
                  <Text size="sm">{camera?.name || recording.cameraId}</Text>
                </Table.Td>
                <Table.Td>
                  <Text size="sm">{new Date(recording.startTime).toLocaleString()}</Text>
                </Table.Td>
                <Table.Td>
                  <Text size="sm">{formatDuration(recording.duration)}</Text>
                </Table.Td>
                <Table.Td>
                  <Text size="sm">{formatBytes(recording.fileSize)}</Text>
                </Table.Td>
                <Table.Td>
                  <Badge size="sm" variant="light">
                    {recording.detectionCount}
                  </Badge>
                </Table.Td>
                <Table.Td>
                  <Group gap="xs">
                    <Tooltip label="Play">
                      <ActionIcon variant="light" onClick={() => onPlay(recording)}>
                        <IconPlayerPlay size={16} />
                      </ActionIcon>
                    </Tooltip>
                    <Tooltip label="Download">
                      <ActionIcon variant="light" onClick={() => onDownload(recording)}>
                        <IconDownload size={16} />
                      </ActionIcon>
                    </Tooltip>
                    <Tooltip label="Delete">
                      <ActionIcon variant="light" color="red" onClick={() => onDelete(recording)}>
                        <IconTrash size={16} />
                      </ActionIcon>
                    </Tooltip>
                  </Group>
                </Table.Td>
              </Table.Tr>
            )
          })}
        </Table.Tbody>
      </Table>
    </Card>
  )
}