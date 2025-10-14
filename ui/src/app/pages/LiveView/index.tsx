import { useEffect, useState } from 'react'
import { useAppStore } from '@/app/store'
import { Card } from '@/app/components/common/Card'
import { StatusPill } from '@/app/components/common/StatusPill'
import { EmptyState } from '@/app/components/common/EmptyState'
import { VideoPlayer } from './VideoPlayer'
import { Radio, Camera, Grid3x3, Box, Crosshair } from 'lucide-react'
import * as Slider from '@radix-ui/react-slider'
import { Toggle } from '@/app/components/common/Toggle'

export function LiveView() {
  const cameras = useAppStore((state) => state.cameras)
  const selectedCameraId = useAppStore((state) => state.selectedCameraId)
  const setSelectedCamera = useAppStore((state) => state.setSelectedCamera)
  const fetchCameras = useAppStore((state) => state.fetchCameras)
  
  const [showOverlay, setShowOverlay] = useState(true)
  const [showGrid, setShowGrid] = useState(false)
  const [showBoxes, setShowBoxes] = useState(true)
  const [showCentroids, setShowCentroids] = useState(true)
  const [confidenceThreshold, setConfidenceThreshold] = useState(50)

  useEffect(() => {
    fetchCameras()
  }, [fetchCameras])

  useEffect(() => {
    // Auto-select first enabled camera if none selected
    if (!selectedCameraId && cameras.length > 0) {
      const firstEnabled = cameras.find((c) => c.enabled)
      if (firstEnabled) {
        setSelectedCamera(firstEnabled.id)
      }
    }
  }, [cameras, selectedCameraId, setSelectedCamera])

  const selectedCamera = cameras.find((c) => c.id === selectedCameraId)
  const enabledCameras = cameras.filter((c) => c.enabled)

  if (enabledCameras.length === 0) {
    return (
      <EmptyState
        icon={<Camera />}
        title="No cameras available"
        description="Enable at least one camera to view live streams"
        action={
          <button
            onClick={() => (window.location.href = '/cameras')}
            className="rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
          >
            Go to Cameras
          </button>
        }
      />
    )
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Live View</h1>
          <p className="mt-1 text-sm text-gray-500">
            Real-time camera streams with detection overlays
          </p>
        </div>
        <div className="flex items-center gap-2">
          <Radio className="h-5 w-5 text-red-600" />
          <span className="text-sm font-medium text-gray-900">LIVE</span>
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-[1fr_320px]">
        {/* Video Player */}
        <div className="space-y-4">
          {/* Camera Selector */}
          <div className="flex gap-2 overflow-x-auto pb-2">
            {enabledCameras.map((camera) => (
              <button
                key={camera.id}
                onClick={() => setSelectedCamera(camera.id)}
                className={`flex items-center gap-2 whitespace-nowrap rounded-lg border px-4 py-2 text-sm font-medium transition-colors ${
                  selectedCameraId === camera.id
                    ? 'border-blue-600 bg-blue-50 text-blue-700'
                    : 'border-gray-300 bg-white text-gray-700 hover:bg-gray-50'
                }`}
              >
                <Camera className="h-4 w-4" />
                {camera.name}
                <StatusPill
                  status={camera.status === 'online' ? 'online' : 'offline'}
                  label=""
                />
              </button>
            ))}
          </div>

          {/* Video */}
          {selectedCamera ? (
            <VideoPlayer
              camera={selectedCamera}
              showOverlay={showOverlay}
              showGrid={showGrid}
              showBoxes={showBoxes}
              showCentroids={showCentroids}
              confidenceThreshold={confidenceThreshold}
            />
          ) : (
            <Card>
              <div className="flex aspect-video items-center justify-center">
                <EmptyState
                  icon={<Camera />}
                  title="No camera selected"
                  description="Select a camera from the list above"
                />
              </div>
            </Card>
          )}
        </div>

        {/* Controls */}
        <div className="space-y-4">
          {/* Overlay Controls */}
          <Card title="Overlay Controls">
            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Box className="h-4 w-4 text-gray-600" />
                  <span className="text-sm font-medium text-gray-700">
                    Show Overlay
                  </span>
                </div>
                <Toggle checked={showOverlay} onCheckedChange={setShowOverlay} />
              </div>

              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Grid3x3 className="h-4 w-4 text-gray-600" />
                  <span className="text-sm font-medium text-gray-700">
                    Show Grid
                  </span>
                </div>
                <Toggle checked={showGrid} onCheckedChange={setShowGrid} />
              </div>

              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Box className="h-4 w-4 text-gray-600" />
                  <span className="text-sm font-medium text-gray-700">
                    Bounding Boxes
                  </span>
                </div>
                <Toggle
                  checked={showBoxes}
                  onCheckedChange={setShowBoxes}
                  disabled={!showOverlay}
                />
              </div>

              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Crosshair className="h-4 w-4 text-gray-600" />
                  <span className="text-sm font-medium text-gray-700">
                    Centroids
                  </span>
                </div>
                <Toggle
                  checked={showCentroids}
                  onCheckedChange={setShowCentroids}
                  disabled={!showOverlay}
                />
              </div>
            </div>
          </Card>

          {/* Confidence Threshold */}
          <Card title="Confidence Threshold">
            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-600">Minimum confidence</span>
                <span className="text-sm font-semibold text-gray-900">
                  {confidenceThreshold}%
                </span>
              </div>
              <Slider.Root
                className="relative flex h-5 w-full touch-none select-none items-center"
                value={[confidenceThreshold]}
                onValueChange={(value) => setConfidenceThreshold(value[0])}
                max={100}
                step={5}
                disabled={!showOverlay}
              >
                <Slider.Track className="relative h-2 w-full grow rounded-full bg-gray-200">
                  <Slider.Range className="absolute h-full rounded-full bg-blue-600" />
                </Slider.Track>
                <Slider.Thumb
                  className="block h-5 w-5 rounded-full border-2 border-blue-600 bg-white shadow transition-colors hover:bg-blue-50 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 disabled:pointer-events-none disabled:opacity-50"
                  aria-label="Confidence threshold"
                />
              </Slider.Root>
              <p className="text-xs text-gray-500">
                Only show detections above this confidence level
              </p>
            </div>
          </Card>

          {/* Camera Info */}
          {selectedCamera && (
            <Card title="Camera Info">
              <dl className="space-y-2 text-sm">
                <div>
                  <dt className="text-gray-500">Name</dt>
                  <dd className="font-medium text-gray-900">{selectedCamera.name}</dd>
                </div>
                <div>
                  <dt className="text-gray-500">Protocol</dt>
                  <dd className="font-medium text-gray-900">{selectedCamera.protocol}</dd>
                </div>
                {selectedCamera.resolution && (
                  <div>
                    <dt className="text-gray-500">Resolution</dt>
                    <dd className="font-medium text-gray-900">
                      {selectedCamera.resolution.width} × {selectedCamera.resolution.height}
                    </dd>
                  </div>
                )}
                {selectedCamera.fps && (
                  <div>
                    <dt className="text-gray-500">FPS</dt>
                    <dd className="font-medium text-gray-900">{selectedCamera.fps}</dd>
                  </div>
                )}
                <div>
                  <dt className="text-gray-500">Status</dt>
                  <dd>
                    <StatusPill
                      status={selectedCamera.status === 'online' ? 'online' : 'offline'}
                      label={selectedCamera.status}
                    />
                  </dd>
                </div>
              </dl>
            </Card>
          )}
        </div>
      </div>
    </div>
  )
}