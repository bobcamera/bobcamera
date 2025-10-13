import * as Dialog from '@radix-ui/react-dialog'
import { X, Camera, Clock, Target, TrendingUp } from 'lucide-react'
import { useAppStore } from '@/app/store'
import { StatusPill } from '@/app/components/common/StatusPill'
import { formatPercent } from '@/lib/utils'
import type { Track } from '@/app/services/schema'

interface TrackDetailPanelProps {
  track: Track
  onClose: () => void
}

export function TrackDetailPanel({ track, onClose }: TrackDetailPanelProps) {
  const cameras = useAppStore((state) => state.cameras)
  const camera = cameras.find((c) => c.id === track.cameraId)

  return (
    <Dialog.Root open onOpenChange={onClose}>
      <Dialog.Portal>
        <Dialog.Overlay className="fixed inset-0 z-50 bg-black/50 data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:fade-out-0 data-[state=open]:fade-in-0" />
        <Dialog.Content className="fixed right-0 top-0 z-50 h-full w-full max-w-lg border-l border-gray-200 bg-white shadow-xl data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:slide-out-to-right data-[state=open]:slide-in-from-right">
          <div className="flex h-full flex-col">
            {/* Header */}
            <div className="flex items-center justify-between border-b border-gray-200 px-6 py-4">
              <Dialog.Title className="text-lg font-semibold text-gray-900">
                Track Details
              </Dialog.Title>
              <Dialog.Close asChild>
                <button
                  className="rounded-lg p-2 text-gray-400 hover:bg-gray-100 hover:text-gray-900"
                  aria-label="Close"
                >
                  <X className="h-5 w-5" />
                </button>
              </Dialog.Close>
            </div>

            {/* Content */}
            <div className="flex-1 space-y-6 overflow-y-auto px-6 py-6">
              {/* Snapshot */}
              {track.thumbnailUrl && (
                <div>
                  <h3 className="mb-2 text-sm font-semibold text-gray-900">
                    Snapshot
                  </h3>
                  <div className="overflow-hidden rounded-lg border border-gray-200">
                    <img
                      src={track.thumbnailUrl}
                      alt={`Track ${track.id}`}
                      className="h-auto w-full"
                    />
                  </div>
                </div>
              )}

              {/* Basic Info */}
              <div>
                <h3 className="mb-3 text-sm font-semibold text-gray-900">
                  Basic Information
                </h3>
                <dl className="space-y-3">
                  <div className="flex items-center justify-between">
                    <dt className="flex items-center gap-2 text-sm text-gray-600">
                      <Target className="h-4 w-4" />
                      Track ID
                    </dt>
                    <dd className="font-mono text-sm font-medium text-gray-900">
                      {track.id}
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="flex items-center gap-2 text-sm text-gray-600">
                      <Clock className="h-4 w-4" />
                      First Seen
                    </dt>
                    <dd className="text-sm font-medium text-gray-900">
                      <div>{new Date(track.firstSeen).toLocaleDateString()}</div>
                      <div className="text-xs text-gray-500">
                        {new Date(track.firstSeen).toLocaleTimeString()}
                      </div>
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="flex items-center gap-2 text-sm text-gray-600">
                      <Clock className="h-4 w-4" />
                      Last Seen
                    </dt>
                    <dd className="text-sm font-medium text-gray-900">
                      <div>{new Date(track.lastSeen).toLocaleDateString()}</div>
                      <div className="text-xs text-gray-500">
                        {new Date(track.lastSeen).toLocaleTimeString()}
                      </div>
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="flex items-center gap-2 text-sm text-gray-600">
                      <Camera className="h-4 w-4" />
                      Camera
                    </dt>
                    <dd className="text-sm font-medium text-gray-900">
                      {camera?.name || track.cameraId}
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="text-sm text-gray-600">Class</dt>
                    <dd>
                      <span className="rounded-full bg-blue-100 px-2 py-1 text-xs font-medium text-blue-700">
                        {track.class}
                      </span>
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="flex items-center gap-2 text-sm text-gray-600">
                      <TrendingUp className="h-4 w-4" />
                      Avg Confidence
                    </dt>
                    <dd className="text-sm font-medium text-gray-900">
                      {formatPercent(track.avgConfidence * 100)}
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="text-sm text-gray-600">Detections</dt>
                    <dd className="text-sm font-medium text-gray-900">
                      {track.detectionCount}
                    </dd>
                  </div>

                  <div className="flex items-center justify-between">
                    <dt className="text-sm text-gray-600">Status</dt>
                    <dd>
                      <StatusPill
                        status={track.status === 'active' ? 'ok' : 'offline'}
                        label={track.status}
                      />
                    </dd>
                  </div>
                </dl>
              </div>

              {/* Additional Info */}
              <div>
                <h3 className="mb-3 text-sm font-semibold text-gray-900">
                  Additional Information
                </h3>
                <p className="text-sm text-gray-500">
                  Track details and history will be available when the backend provides extended track data.
                </p>
              </div>
            </div>

            {/* Footer */}
            <div className="border-t border-gray-200 px-6 py-4">
              <button
                onClick={onClose}
                className="w-full rounded-lg bg-gray-100 px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-200"
              >
                Close
              </button>
            </div>
          </div>
        </Dialog.Content>
      </Dialog.Portal>
    </Dialog.Root>
  )
}