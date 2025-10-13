import { useEffect, useState } from 'react'
import { useAppStore } from '@/app/store'
import { Card } from '@/app/components/common/Card'
import { PageSpinner } from '@/app/components/common/Spinner'
import { EmptyState } from '@/app/components/common/EmptyState'
import { useToast } from '@/app/components/common/Toast'
import { Settings as SettingsIcon, Save, RotateCcw, AlertTriangle } from 'lucide-react'
import * as Tabs from '@radix-ui/react-tabs'

export function Settings() {
  const config = useAppStore((state) => state.config)
  const draftConfig = useAppStore((state) => state.draftConfig)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const fetchConfig = useAppStore((state) => state.fetchConfig)
  const updateDraftConfig = useAppStore((state) => state.updateDraftConfig)
  const saveConfig = useAppStore((state) => state.saveConfig)
  const resetDraftConfig = useAppStore((state) => state.resetDraftConfig)
  const hasPendingChanges = useAppStore((state) => state.hasPendingChanges)
  
  const [activeTab, setActiveTab] = useState('detection')
  const [isSaving, setIsSaving] = useState(false)
  const { success, error } = useToast()

  useEffect(() => {
    fetchConfig()
  }, [fetchConfig])

  const handleSave = async () => {
    if (!hasPendingChanges() || !draftConfig) {
      return
    }

    setIsSaving(true)
    try {
      await saveConfig(draftConfig)
      success('Settings saved', 'Configuration has been updated successfully')
    } catch (err) {
      error('Failed to save settings', (err as Error).message)
    } finally {
      setIsSaving(false)
    }
  }

  const handleReset = () => {
    if (confirm('Are you sure you want to discard all changes?')) {
      resetDraftConfig()
      success('Changes discarded', 'Configuration has been reset')
    }
  }

  if (backendStatus === 'connecting' || !config) {
    return <PageSpinner />
  }

  if (backendStatus === 'disconnected') {
    return (
      <EmptyState
        icon={SettingsIcon}
        title="Backend Disconnected"
        description="Unable to load settings. Please check your connection."
        action={
          <button
            onClick={() => fetchConfig()}
            className="rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
          >
            Retry
          </button>
        }
      />
    )
  }

  const draft = draftConfig || config

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Settings</h1>
          <p className="mt-1 text-sm text-gray-500">
            Configure detection, tracking, and system parameters
          </p>
        </div>
        <div className="flex gap-2">
          <button
            onClick={handleReset}
            disabled={!hasPendingChanges()}
            className="flex items-center gap-2 rounded-lg border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50 disabled:opacity-50"
          >
            <RotateCcw className="h-4 w-4" />
            Reset
          </button>
          <button
            onClick={handleSave}
            disabled={!hasPendingChanges() || isSaving}
            className="flex items-center gap-2 rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700 disabled:opacity-50"
          >
            <Save className="h-4 w-4" />
            {isSaving ? 'Saving...' : 'Save Changes'}
          </button>
        </div>
      </div>

      {/* Pending Changes Warning */}
      {hasPendingChanges() && (
        <Card>
          <div className="flex items-start gap-3">
            <AlertTriangle className="h-5 w-5 flex-shrink-0 text-yellow-600" />
            <div>
              <h3 className="text-sm font-semibold text-gray-900">
                Unsaved Changes
              </h3>
              <p className="mt-1 text-sm text-gray-600">
                You have unsaved changes. Click "Save Changes" to apply them or "Reset" to discard.
              </p>
            </div>
          </div>
        </Card>
      )}

      {/* Settings Tabs */}
      <Tabs.Root value={activeTab} onValueChange={setActiveTab}>
        <Tabs.List className="flex gap-2 border-b border-gray-200">
          <Tabs.Trigger
            value="detection"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Detection
          </Tabs.Trigger>
          <Tabs.Trigger
            value="tracking"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Tracking
          </Tabs.Trigger>
          <Tabs.Trigger
            value="storage"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Storage
          </Tabs.Trigger>
          <Tabs.Trigger
            value="network"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Network
          </Tabs.Trigger>
        </Tabs.List>

        {/* Detection Settings */}
        <Tabs.Content value="detection" className="mt-6">
          <Card title="Detection Parameters">
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Confidence Threshold
                </label>
                <input
                  type="number"
                  min="0"
                  max="1"
                  step="0.05"
                  value={draft.detection?.confidence || 0.5}
                  onChange={(e) =>
                    updateDraftConfig({
                      detection: {
                        ...draft.detection,
                        confidence: parseFloat(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Minimum confidence score for detections (0.0 - 1.0)
                </p>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700">
                  NMS Threshold
                </label>
                <input
                  type="number"
                  min="0"
                  max="1"
                  step="0.05"
                  value={draft.detection?.nms || 0.45}
                  onChange={(e) =>
                    updateDraftConfig({
                      detection: {
                        ...draft.detection,
                        nms: parseFloat(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Non-maximum suppression threshold (0.0 - 1.0)
                </p>
              </div>

              <div className="flex items-center justify-between">
                <div>
                  <label className="text-sm font-medium text-gray-700">
                    Enable Detection
                  </label>
                  <p className="text-xs text-gray-500">
                    Enable object detection
                  </p>
                </div>
                <input
                  type="checkbox"
                  checked={draft.detection?.enabled || false}
                  onChange={(e) =>
                    updateDraftConfig({
                      detection: {
                        ...draft.detection,
                        enabled: e.target.checked,
                      },
                    })
                  }
                  className="h-4 w-4 rounded border-gray-300 text-blue-600 focus:ring-blue-500"
                />
              </div>
            </div>
          </Card>
        </Tabs.Content>

        {/* Tracking Settings */}
        <Tabs.Content value="tracking" className="mt-6">
          <Card title="Tracking Parameters">
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Max Track Age
                </label>
                <input
                  type="number"
                  min="1"
                  value={draft.tracking?.maxAge || 30}
                  onChange={(e) =>
                    updateDraftConfig({
                      tracking: {
                        ...draft.tracking,
                        maxAge: parseInt(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Maximum frames to keep a track without detection
                </p>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Min Hits
                </label>
                <input
                  type="number"
                  min="1"
                  value={draft.tracking?.minHits || 3}
                  onChange={(e) =>
                    updateDraftConfig({
                      tracking: {
                        ...draft.tracking,
                        minHits: parseInt(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Minimum detections before confirming a track
                </p>
              </div>
            </div>
          </Card>
        </Tabs.Content>

        {/* Storage Settings */}
        <Tabs.Content value="storage" className="mt-6">
          <Card title="Storage Configuration">
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Storage Path
                </label>
                <input
                  type="text"
                  value={draft.storage?.path || ''}
                  onChange={(e) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        path: e.target.value,
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 font-mono text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Max Storage Size (bytes)
                </label>
                <input
                  type="number"
                  min="1"
                  value={draft.storage?.maxSize || 100000000000}
                  onChange={(e) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        maxSize: parseInt(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Maximum storage size in bytes
                </p>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700">
                  Retention Period (days)
                </label>
                <input
                  type="number"
                  min="1"
                  value={draft.storage?.retention || 30}
                  onChange={(e) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        retention: parseInt(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Number of days to retain recordings
                </p>
              </div>

              <div className="flex items-center justify-between">
                <div>
                  <label className="text-sm font-medium text-gray-700">
                    Enable Storage
                  </label>
                  <p className="text-xs text-gray-500">
                    Enable recording storage
                  </p>
                </div>
                <input
                  type="checkbox"
                  checked={draft.storage?.enabled || false}
                  onChange={(e) =>
                    updateDraftConfig({
                      storage: {
                        ...draft.storage,
                        enabled: e.target.checked,
                      },
                    })
                  }
                  className="h-4 w-4 rounded border-gray-300 text-blue-600 focus:ring-blue-500"
                />
              </div>
            </div>
          </Card>
        </Tabs.Content>

        {/* Network Settings */}
        <Tabs.Content value="network" className="mt-6">
          <Card title="Network Configuration">
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700">
                  API Port
                </label>
                <input
                  type="number"
                  min="1"
                  max="65535"
                  value={draft.network?.apiPort || 8080}
                  onChange={(e) =>
                    updateDraftConfig({
                      network: {
                        ...draft.network,
                        apiPort: parseInt(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700">
                  WebSocket Port
                </label>
                <input
                  type="number"
                  min="1"
                  max="65535"
                  value={draft.network?.wsPort || 8081}
                  onChange={(e) =>
                    updateDraftConfig({
                      network: {
                        ...draft.network,
                        wsPort: parseInt(e.target.value),
                      },
                    })
                  }
                  className="mt-1 block w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
              </div>
            </div>
          </Card>
        </Tabs.Content>
      </Tabs.Root>
    </div>
  )
}