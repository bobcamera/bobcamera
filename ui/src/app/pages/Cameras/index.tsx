import { useEffect, useState } from 'react'
import { useAppStore } from '@/app/store'
import { Card } from '@/app/components/common/Card'
import { StatusPill } from '@/app/components/common/StatusPill'
import { PageSpinner } from '@/app/components/common/Spinner'
import { EmptyState } from '@/app/components/common/EmptyState'
import { useToast } from '@/app/components/common/Toast'
import { Camera, Plus, Edit, Trash2, TestTube, Power, PowerOff } from 'lucide-react'
import { CameraDrawer } from './CameraDrawer'
import type { Camera as CameraType } from '@/app/services/schema'

export function Cameras() {
  const cameras = useAppStore((state) => state.cameras)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const fetchCameras = useAppStore((state) => state.fetchCameras)
  const updateCamera = useAppStore((state) => state.updateCamera)
  const deleteCamera = useAppStore((state) => state.deleteCamera)
  const testCamera = useAppStore((state) => state.testCamera)
  
  const [selectedCamera, setSelectedCamera] = useState<CameraType | null>(null)
  const [isDrawerOpen, setIsDrawerOpen] = useState(false)
  const [isCreating, setIsCreating] = useState(false)
  const [testingCameraId, setTestingCameraId] = useState<string | null>(null)
  
  const { success, error } = useToast()

  useEffect(() => {
    fetchCameras()
    
    // Poll every 10 seconds
    const interval = setInterval(() => {
      fetchCameras()
    }, 10000)
    
    return () => clearInterval(interval)
  }, [fetchCameras])

  const handleEdit = (camera: CameraType) => {
    setSelectedCamera(camera)
    setIsCreating(false)
    setIsDrawerOpen(true)
  }

  const handleCreate = () => {
    setSelectedCamera(null)
    setIsCreating(true)
    setIsDrawerOpen(true)
  }

  const handleToggleEnabled = async (camera: CameraType) => {
    try {
      await updateCamera(camera.id, { enabled: !camera.enabled })
      success(
        camera.enabled ? 'Camera disabled' : 'Camera enabled',
        `${camera.name} has been ${camera.enabled ? 'disabled' : 'enabled'}`
      )
    } catch (err) {
      error('Failed to update camera', (err as Error).message)
    }
  }

  const handleTest = async (camera: CameraType) => {
    setTestingCameraId(camera.id)
    try {
      const result = await testCamera(camera.id)
      if (result) {
        success('Connection successful', `${camera.name} is reachable`)
      } else {
        error('Connection failed', 'Unable to connect to camera')
      }
    } catch (err) {
      error('Test failed', (err as Error).message)
    } finally {
      setTestingCameraId(null)
    }
  }

  const handleDelete = async (camera: CameraType) => {
    if (!confirm(`Are you sure you want to delete ${camera.name}?`)) {
      return
    }
    
    try {
      await deleteCamera(camera.id)
      success('Camera deleted', `${camera.name} has been removed`)
    } catch (err) {
      error('Failed to delete camera', (err as Error).message)
    }
  }

  if (backendStatus === 'connecting') {
    return <PageSpinner />
  }

  if (backendStatus === 'disconnected') {
    return (
      <EmptyState
        icon={Camera}
        title="Backend Disconnected"
        description="Unable to load cameras. Please check your connection."
        action={
          <button
            onClick={() => fetchCameras()}
            className="rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
          >
            Retry
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
          <h1 className="text-3xl font-bold text-gray-900">Cameras</h1>
          <p className="mt-1 text-sm text-gray-500">
            Manage camera sources and configurations
          </p>
        </div>
        <button
          onClick={handleCreate}
          className="flex items-center gap-2 rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
        >
          <Plus className="h-4 w-4" />
          Add Camera
        </button>
      </div>

      {/* Camera List */}
      {cameras.length === 0 ? (
        <EmptyState
          icon={Camera}
          title="No cameras configured"
          description="Get started by adding your first camera source"
          action={
            <button
              onClick={handleCreate}
              className="rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
            >
              Add Camera
            </button>
          }
        />
      ) : (
        <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
          {cameras.map((camera) => (
            <Card key={camera.id}>
              <div className="space-y-4">
                {/* Header */}
                <div className="flex items-start justify-between">
                  <div className="flex items-center gap-3">
                    <div className="rounded-lg bg-blue-100 p-2">
                      <Camera className="h-5 w-5 text-blue-600" />
                    </div>
                    <div>
                      <h3 className="font-semibold text-gray-900">{camera.name}</h3>
                      <p className="text-xs text-gray-500">{camera.protocol}</p>
                    </div>
                  </div>
                  <StatusPill
                    status={
                      camera.enabled
                        ? camera.status === 'online'
                          ? 'online'
                          : camera.status === 'error'
                          ? 'error'
                          : 'warning'
                        : 'offline'
                    }
                    label={camera.enabled ? camera.status : 'disabled'}
                  />
                </div>

                {/* Details */}
                <div className="space-y-2 text-sm">
                  <div>
                    <span className="text-gray-500">URL:</span>
                    <p className="truncate font-mono text-xs text-gray-900">
                      {camera.url}
                    </p>
                  </div>
                  {camera.lastSeen && (
                    <div>
                      <span className="text-gray-500">Last seen:</span>
                      <p className="text-xs text-gray-900">
                        {new Date(camera.lastSeen).toLocaleString()}
                      </p>
                    </div>
                  )}
                  {camera.resolution && (
                    <div>
                      <span className="text-gray-500">Resolution:</span>
                      <p className="text-xs text-gray-900">
                        {camera.resolution.width} × {camera.resolution.height}
                      </p>
                    </div>
                  )}
                  {camera.fps && (
                    <div>
                      <span className="text-gray-500">FPS:</span>
                      <p className="text-xs text-gray-900">{camera.fps}</p>
                    </div>
                  )}
                </div>

                {/* Actions */}
                <div className="flex gap-2 border-t border-gray-200 pt-4">
                  <button
                    onClick={() => handleToggleEnabled(camera)}
                    className="flex flex-1 items-center justify-center gap-2 rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
                    title={camera.enabled ? 'Disable camera' : 'Enable camera'}
                  >
                    {camera.enabled ? (
                      <PowerOff className="h-4 w-4" />
                    ) : (
                      <Power className="h-4 w-4" />
                    )}
                  </button>
                  <button
                    onClick={() => handleTest(camera)}
                    disabled={testingCameraId === camera.id}
                    className="flex flex-1 items-center justify-center gap-2 rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50 disabled:opacity-50"
                    title="Test connection"
                  >
                    <TestTube className="h-4 w-4" />
                    {testingCameraId === camera.id ? 'Testing...' : 'Test'}
                  </button>
                  <button
                    onClick={() => handleEdit(camera)}
                    className="flex flex-1 items-center justify-center gap-2 rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
                    title="Edit camera"
                  >
                    <Edit className="h-4 w-4" />
                  </button>
                  <button
                    onClick={() => handleDelete(camera)}
                    className="flex items-center justify-center gap-2 rounded-lg border border-red-300 bg-white px-3 py-2 text-sm font-medium text-red-700 hover:bg-red-50"
                    title="Delete camera"
                  >
                    <Trash2 className="h-4 w-4" />
                  </button>
                </div>
              </div>
            </Card>
          ))}
        </div>
      )}

      {/* Camera Drawer */}
      <CameraDrawer
        camera={selectedCamera}
        isOpen={isDrawerOpen}
        isCreating={isCreating}
        onClose={() => {
          setIsDrawerOpen(false)
          setSelectedCamera(null)
        }}
      />
    </div>
  )
}