import { useState, useEffect } from 'react'
import { useAppStore } from '@/app/store'
import { useToast } from '@/app/components/common/Toast'
import * as Dialog from '@radix-ui/react-dialog'
import { X } from 'lucide-react'
import type { Camera } from '@/app/services/schema'

interface CameraDrawerProps {
  camera: Camera | null
  isOpen: boolean
  isCreating: boolean
  onClose: () => void
}

interface CameraFormData {
  name: string
  url: string
  protocol: string
  enabled: boolean
  username?: string
  password?: string
}

export function CameraDrawer({ camera, isOpen, isCreating, onClose }: CameraDrawerProps) {
  const createCamera = useAppStore((state) => state.createCamera)
  const updateCamera = useAppStore((state) => state.updateCamera)
  const { success, error } = useToast()

  const [formData, setFormData] = useState<CameraFormData>({
    name: '',
    url: '',
    protocol: 'rtsp',
    enabled: true,
    username: '',
    password: '',
  })

  const [isSubmitting, setIsSubmitting] = useState(false)

  useEffect(() => {
    if (camera) {
      setFormData({
        name: camera.name,
        url: camera.url || '',
        protocol: camera.protocol,
        enabled: camera.enabled,
        username: camera.username || '',
        password: '',
      })
    } else {
      setFormData({
        name: '',
        url: '',
        protocol: 'rtsp',
        enabled: true,
        username: '',
        password: '',
      })
    }
  }, [camera, isOpen])

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setIsSubmitting(true)

    try {
      if (isCreating) {
        await createCamera(formData as any)
        success('Camera created', `${formData.name} has been added`)
      } else if (camera) {
        await updateCamera(camera.id, formData as any)
        success('Camera updated', `${formData.name} has been updated`)
      }
      onClose()
    } catch (err) {
      error(
        isCreating ? 'Failed to create camera' : 'Failed to update camera',
        (err as Error).message
      )
    } finally {
      setIsSubmitting(false)
    }
  }

  return (
    <Dialog.Root open={isOpen} onOpenChange={onClose}>
      <Dialog.Portal>
        <Dialog.Overlay className="fixed inset-0 z-50 bg-black/50 data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:fade-out-0 data-[state=open]:fade-in-0" />
        <Dialog.Content className="fixed right-0 top-0 z-50 h-full w-full max-w-md border-l border-gray-200 bg-white shadow-xl data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:slide-out-to-right data-[state=open]:slide-in-from-right">
          <div className="flex h-full flex-col">
            {/* Header */}
            <div className="flex items-center justify-between border-b border-gray-200 px-6 py-4">
              <Dialog.Title className="text-lg font-semibold text-gray-900">
                {isCreating ? 'Add Camera' : 'Edit Camera'}
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

            {/* Form */}
            <form onSubmit={handleSubmit} className="flex flex-1 flex-col overflow-hidden">
              <div className="flex-1 space-y-6 overflow-y-auto px-6 py-6">
                {/* Name */}
                <div>
                  <label htmlFor="name" className="block text-sm font-medium text-gray-700">
                    Camera Name *
                  </label>
                  <input
                    type="text"
                    id="name"
                    required
                    value={formData.name}
                    onChange={(e) => setFormData({ ...formData, name: e.target.value })}
                    className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                    placeholder="Front Door Camera"
                  />
                </div>

                {/* Protocol */}
                <div>
                  <label htmlFor="protocol" className="block text-sm font-medium text-gray-700">
                    Protocol *
                  </label>
                  <select
                    id="protocol"
                    required
                    value={formData.protocol}
                    onChange={(e) => setFormData({ ...formData, protocol: e.target.value })}
                    className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                  >
                    <option value="rtsp">RTSP</option>
                    <option value="onvif">ONVIF</option>
                    <option value="http">HTTP/MJPEG</option>
                    <option value="usb">USB</option>
                  </select>
                </div>

                {/* URL */}
                <div>
                  <label htmlFor="url" className="block text-sm font-medium text-gray-700">
                    Stream URL *
                  </label>
                  <input
                    type="text"
                    id="url"
                    required
                    value={formData.url}
                    onChange={(e) => setFormData({ ...formData, url: e.target.value })}
                    className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 font-mono text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                    placeholder="rtsp://192.168.1.100:554/stream"
                  />
                  <p className="mt-1 text-xs text-gray-500">
                    Full URL to the camera stream
                  </p>
                </div>

                {/* Username */}
                <div>
                  <label htmlFor="username" className="block text-sm font-medium text-gray-700">
                    Username
                  </label>
                  <input
                    type="text"
                    id="username"
                    value={formData.username}
                    onChange={(e) => setFormData({ ...formData, username: e.target.value })}
                    className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                    placeholder="admin"
                    autoComplete="username"
                  />
                </div>

                {/* Password */}
                <div>
                  <label htmlFor="password" className="block text-sm font-medium text-gray-700">
                    Password
                  </label>
                  <input
                    type="password"
                    id="password"
                    value={formData.password}
                    onChange={(e) => setFormData({ ...formData, password: e.target.value })}
                    className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                    placeholder={isCreating ? '' : '••••••••'}
                    autoComplete="current-password"
                  />
                  {!isCreating && (
                    <p className="mt-1 text-xs text-gray-500">
                      Leave blank to keep existing password
                    </p>
                  )}
                </div>

                {/* Enabled */}
                <div className="flex items-center justify-between">
                  <div>
                    <label htmlFor="enabled" className="text-sm font-medium text-gray-700">
                      Enable Camera
                    </label>
                    <p className="text-xs text-gray-500">
                      Start streaming from this camera immediately
                    </p>
                  </div>
                  <input
                    type="checkbox"
                    id="enabled"
                    checked={formData.enabled}
                    onChange={(e) => setFormData({ ...formData, enabled: e.target.checked })}
                    className="h-4 w-4 rounded border-gray-300 text-blue-600 focus:ring-blue-500"
                  />
                </div>
              </div>

              {/* Footer */}
              <div className="flex gap-3 border-t border-gray-200 px-6 py-4">
                <button
                  type="button"
                  onClick={onClose}
                  className="flex-1 rounded-lg border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={isSubmitting}
                  className="flex-1 rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700 disabled:opacity-50"
                >
                  {isSubmitting ? 'Saving...' : isCreating ? 'Create' : 'Update'}
                </button>
              </div>
            </form>
          </div>
        </Dialog.Content>
      </Dialog.Portal>
    </Dialog.Root>
  )
}