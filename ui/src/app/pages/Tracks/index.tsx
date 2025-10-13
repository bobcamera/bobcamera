import { useEffect, useState } from 'react'
import { useAppStore } from '@/app/store'
import { Card } from '@/app/components/common/Card'
import { Table, Pagination } from '@/app/components/common/Table'
import { StatusPill } from '@/app/components/common/StatusPill'
import { PageSpinner } from '@/app/components/common/Spinner'
import { EmptyState } from '@/app/components/common/EmptyState'
import { Target, Download, Filter, X } from 'lucide-react'
import { formatPercent } from '@/lib/utils'
import { TrackDetailPanel } from './TrackDetailPanel'
import type { Track } from '@/app/services/schema'

export function Tracks() {
  const tracks = useAppStore((state) => state.tracks)
  const filters = useAppStore((state) => state.filters)
  const pagination = useAppStore((state) => state.pagination)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const cameras = useAppStore((state) => state.cameras)
  const setFilters = useAppStore((state) => state.setFilters)
  const setPage = useAppStore((state) => state.setPage)
  
  const [selectedTrack, setSelectedTrack] = useState<Track | null>(null)
  const [showFilters, setShowFilters] = useState(false)

  useEffect(() => {
    // Fetch tracks from API when filters or page changes
    // This would be implemented with actual API call
  }, [filters, pagination.page])

  const handleExportCSV = () => {
    const headers = ['ID', 'First Seen', 'Camera', 'Class', 'Confidence', 'Status']
    const rows = tracks.map((track) => [
      track.id,
      new Date(track.firstSeen).toISOString(),
      cameras.find((c) => c.id === track.cameraId)?.name || track.cameraId,
      track.class,
      track.avgConfidence,
      track.status === 'active' ? 'Active' : 'Ended',
    ])

    const csv = [headers, ...rows].map((row) => row.join(',')).join('\n')
    const blob = new Blob([csv], { type: 'text/csv' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `tracks-${new Date().toISOString()}.csv`
    a.click()
    URL.revokeObjectURL(url)
  }

  const clearFilters = () => {
    setFilters({
      cameraId: undefined,
      class: undefined,
      minConfidence: undefined,
      from: undefined,
      to: undefined,
    })
  }

  const hasActiveFilters =
    filters.cameraId ||
    filters.class ||
    filters.minConfidence !== undefined ||
    filters.from ||
    filters.to

  if (backendStatus === 'connecting') {
    return <PageSpinner />
  }

  if (backendStatus === 'disconnected') {
    return (
      <EmptyState
        icon={Target}
        title="Backend Disconnected"
        description="Unable to load tracks. Please check your connection."
        action={
          <button
            onClick={() => window.location.reload()}
            className="rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
          >
            Retry
          </button>
        }
      />
    )
  }

  const columns = [
    {
      key: 'id',
      header: 'Track ID',
      sortable: true,
      render: (track: Track) => (
        <span className="font-mono text-sm">{track.id}</span>
      ),
    },
    {
      key: 'timestamp',
      header: 'Timestamp',
      sortable: true,
      render: (track: Track) => (
        <div className="text-sm">
          <div>{new Date(track.firstSeen).toLocaleDateString()}</div>
          <div className="text-gray-500">
            {new Date(track.firstSeen).toLocaleTimeString()}
          </div>
        </div>
      ),
    },
    {
      key: 'cameraId',
      header: 'Camera',
      sortable: true,
      render: (track: Track) => {
        const camera = cameras.find((c) => c.id === track.cameraId)
        return (
          <span className="text-sm">{camera?.name || track.cameraId}</span>
        )
      },
    },
    {
      key: 'class',
      header: 'Class',
      sortable: true,
      render: (track: Track) => (
        <span className="rounded-full bg-blue-100 px-2 py-1 text-xs font-medium text-blue-700">
          {track.class}
        </span>
      ),
    },
    {
      key: 'confidence',
      header: 'Confidence',
      sortable: true,
      render: (track: Track) => (
        <span className="text-sm font-medium">
          {formatPercent(track.avgConfidence * 100)}
        </span>
      ),
    },
    {
      key: 'status',
      header: 'Status',
      sortable: true,
      render: (track: Track) => (
        <StatusPill
          status={track.status === 'active' ? 'ok' : 'offline'}
          label={track.status === 'active' ? 'Active' : 'Ended'}
        />
      ),
    },
  ]

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Tracks & Detections</h1>
          <p className="mt-1 text-sm text-gray-500">
            Browse and analyze detected objects
          </p>
        </div>
        <div className="flex gap-2">
          <button
            onClick={() => setShowFilters(!showFilters)}
            className="flex items-center gap-2 rounded-lg border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
          >
            <Filter className="h-4 w-4" />
            Filters
            {hasActiveFilters && (
              <span className="flex h-5 w-5 items-center justify-center rounded-full bg-blue-600 text-xs text-white">
                !
              </span>
            )}
          </button>
          <button
            onClick={handleExportCSV}
            disabled={tracks.length === 0}
            className="flex items-center gap-2 rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700 disabled:opacity-50"
          >
            <Download className="h-4 w-4" />
            Export CSV
          </button>
        </div>
      </div>

      {/* Filters */}
      {showFilters && (
        <Card>
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <h3 className="text-sm font-semibold text-gray-900">Filters</h3>
              {hasActiveFilters && (
                <button
                  onClick={clearFilters}
                  className="flex items-center gap-1 text-sm text-blue-600 hover:text-blue-700"
                >
                  <X className="h-4 w-4" />
                  Clear all
                </button>
              )}
            </div>

            <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
              {/* Camera Filter */}
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Camera
                </label>
                <select
                  value={filters.cameraId || ''}
                  onChange={(e) =>
                    setFilters({
                      ...filters,
                      cameraId: e.target.value || undefined,
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-1 focus:ring-blue-500"
                >
                  <option value="">All cameras</option>
                  {cameras.map((camera) => (
                    <option key={camera.id} value={camera.id}>
                      {camera.name}
                    </option>
                  ))}
                </select>
              </div>

              {/* Class Filter */}
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Class
                </label>
                <select
                  value={filters.class || ''}
                  onChange={(e) =>
                    setFilters({
                      ...filters,
                      class: e.target.value || undefined,
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                >
                  <option value="">All classes</option>
                  <option value="bird">Bird</option>
                  <option value="bat">Bat</option>
                  <option value="insect">Insect</option>
                  <option value="uap">UAP</option>
                  <option value="unknown">Unknown</option>
                </select>
              </div>

              {/* Min Confidence */}
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Min Confidence
                </label>
                <input
                  type="number"
                  min="0"
                  max="100"
                  value={filters.minConfidence || ''}
                  onChange={(e) =>
                    setFilters({
                      ...filters,
                      minConfidence: e.target.value
                        ? parseFloat(e.target.value)
                        : undefined,
                    })
                  }
                  placeholder="0-100"
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
              </div>

              {/* Date Range */}
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Date Range
                </label>
                <input
                  type="date"
                  value={filters.from?.split('T')[0] || ''}
                  onChange={(e) =>
                    setFilters({
                      ...filters,
                      from: e.target.value
                        ? new Date(e.target.value).toISOString()
                        : undefined,
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
              </div>
            </div>
          </div>
        </Card>
      )}

      {/* Table */}
      <Card>
        <Table
          columns={columns}
          data={tracks}
          keyExtractor={(track) => track.id}
          onRowClick={(track) => setSelectedTrack(track)}
          emptyMessage={
            hasActiveFilters
              ? 'No tracks found. Try adjusting your filters.'
              : 'Tracks will appear here when objects are detected'
          }
        />
        {tracks.length > 0 && (
          <Pagination
            page={pagination.page}
            pageSize={pagination.pageSize}
            total={pagination.total}
            hasMore={pagination.hasMore}
            onPageChange={setPage}
          />
        )}
      </Card>

      {/* Detail Panel */}
      {selectedTrack && (
        <TrackDetailPanel
          track={selectedTrack}
          onClose={() => setSelectedTrack(null)}
        />
      )}
    </div>
  )
}