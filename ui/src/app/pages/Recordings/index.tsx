import { useEffect, useState } from 'react'
import { Card, CardGrid } from '@/app/components/common/Card'
import { PageSpinner } from '@/app/components/common/Spinner'
import { EmptyState } from '@/app/components/common/EmptyState'
import { Video, Download, Play, Trash2, Calendar } from 'lucide-react'
import { formatBytes, formatDuration } from '@/lib/utils'
import { api } from '@/app/services/api'
import { useToast } from '@/app/components/common/Toast'
import { useAppStore } from '@/app/store'
import type { Recording } from '@/app/services/schema'

export function Recordings() {
  const [recordings, setRecordings] = useState<Recording[]>([])
  const [isLoading, setIsLoading] = useState(true)
  const [filters, setFilters] = useState({
    cameraId: '',
    startDate: '',
    endDate: '',
  })
  const { success, error } = useToast()
  const cameras = useAppStore((state) => state.cameras)

  useEffect(() => {
    fetchRecordings()
  }, [filters])

  const fetchRecordings = async () => {
    setIsLoading(true)
    try {
      const params: Record<string, string> = {}
      if (filters.cameraId) params.cameraId = filters.cameraId
      if (filters.startDate) params.from = new Date(filters.startDate).toISOString()
      if (filters.endDate) params.to = new Date(filters.endDate).toISOString()

      const response = await api.getRecordings(params)
      // Handle paginated response
      setRecordings(response.items || response)
    } catch (err) {
      error('Failed to load recordings', (err as Error).message)
    } finally {
      setIsLoading(false)
    }
  }

  const handleDownload = (recording: Recording) => {
    const a = document.createElement('a')
    a.href = recording.url
    a.download = recording.filename || `recording-${recording.id}.mp4`
    a.click()
    success('Download started', `Downloading ${recording.filename || 'recording'}`)
  }

  const handleDelete = async (recording: Recording) => {
    if (!confirm(`Are you sure you want to delete ${recording.filename || 'this recording'}?`)) {
      return
    }

    try {
      await api.deleteRecording(recording.id)
      success('Recording deleted', `${recording.filename || 'Recording'} has been removed`)
      fetchRecordings()
    } catch (err) {
      error('Failed to delete recording', (err as Error).message)
    }
  }

  if (isLoading) {
    return <PageSpinner />
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Recordings</h1>
          <p className="mt-1 text-sm text-gray-500">
            Browse and download recorded clips
          </p>
        </div>
      </div>

      {/* Filters */}
      <Card>
        <div className="grid gap-4 sm:grid-cols-3">
          <div>
            <label className="block text-sm font-medium text-gray-700">
              Camera
            </label>
            <select
              value={filters.cameraId}
              onChange={(e) => setFilters({ ...filters, cameraId: e.target.value })}
              className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
            >
              <option value="">All cameras</option>
              {/* Camera options would be populated from store */}
            </select>
          </div>

          <div>
            <label className="block text-sm font-medium text-gray-700">
              Start Date
            </label>
            <input
              type="date"
              value={filters.startDate}
              onChange={(e) => setFilters({ ...filters, startDate: e.target.value })}
              className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
            />
          </div>

          <div>
            <label className="block text-sm font-medium text-gray-700">
              End Date
            </label>
            <input
              type="date"
              value={filters.endDate}
              onChange={(e) => setFilters({ ...filters, endDate: e.target.value })}
              className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
            />
          </div>
        </div>
      </Card>

      {/* Recordings Grid */}
      {recordings.length === 0 ? (
        <EmptyState
          icon={<Video />}
          title="No recordings found"
          description="Recordings will appear here when clips are saved"
        />
      ) : (
        <CardGrid>
          {recordings.map((recording) => (
            <Card key={recording.id}>
              <div className="space-y-4">
                {/* Thumbnail */}
                <div className="relative aspect-video overflow-hidden rounded-lg bg-gray-900">
                  {recording.thumbnailUrl ? (
                    <img
                      src={recording.thumbnailUrl}
                      alt={recording.filename || 'Recording'}
                      className="h-full w-full object-cover"
                    />
                  ) : (
                    <div className="flex h-full items-center justify-center">
                      <Video className="h-12 w-12 text-gray-600" />
                    </div>
                  )}
                  <div className="absolute bottom-2 right-2 rounded bg-black/60 px-2 py-1 text-xs text-white backdrop-blur-sm">
                    {formatDuration(recording.duration)}
                  </div>
                </div>

                {/* Info */}
                <div className="space-y-2">
                  <h3 className="truncate font-semibold text-gray-900">
                    {recording.filename || `Recording ${recording.id}`}
                  </h3>
                  <div className="flex items-center gap-2 text-xs text-gray-500">
                    <Calendar className="h-3 w-3" />
                    {new Date(recording.startTime).toLocaleString()}
                  </div>
                  <div className="flex items-center justify-between text-sm">
                    <span className="text-gray-600">Size:</span>
                    <span className="font-medium text-gray-900">
                      {formatBytes(recording.fileSize)}
                    </span>
                  </div>
                  {recording.cameraId && (
                    <div className="flex items-center justify-between text-sm">
                      <span className="text-gray-600">Camera:</span>
                      <span className="font-medium text-gray-900">
                        {cameras.find((c) => c.id === recording.cameraId)?.name || recording.cameraId}
                      </span>
                    </div>
                  )}
                </div>

                {/* Actions */}
                <div className="flex gap-2 border-t border-gray-200 pt-4">
                  <button
                    onClick={() => window.open(recording.url, '_blank')}
                    className="flex flex-1 items-center justify-center gap-2 rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
                  >
                    <Play className="h-4 w-4" />
                    Play
                  </button>
                  <button
                    onClick={() => handleDownload(recording)}
                    className="flex flex-1 items-center justify-center gap-2 rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
                  >
                    <Download className="h-4 w-4" />
                    Download
                  </button>
                  <button
                    onClick={() => handleDelete(recording)}
                    className="flex items-center justify-center gap-2 rounded-lg border border-red-300 bg-white px-3 py-2 text-sm font-medium text-red-700 hover:bg-red-50"
                  >
                    <Trash2 className="h-4 w-4" />
                  </button>
                </div>
              </div>
            </Card>
          ))}
        </CardGrid>
      )}
    </div>
  )
}