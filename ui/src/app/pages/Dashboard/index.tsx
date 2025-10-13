import { useEffect } from 'react'
import { useAppStore } from '@/app/store'
import { Card, CardGrid } from '@/app/components/common/Card'
import { StatusPill } from '@/app/components/common/StatusPill'
import { PageSpinner } from '@/app/components/common/Spinner'
import { EmptyState } from '@/app/components/common/EmptyState'
import { 
  Activity, 
  Cpu, 
  HardDrive, 
  Camera, 
  Target, 
  TrendingUp,
  Zap,
  Thermometer
} from 'lucide-react'
import { formatBytes, formatPercent, formatUptime } from '@/lib/utils'

export function Dashboard() {
  const systemHealth = useAppStore((state) => state.systemHealth)
  const metrics = useAppStore((state) => state.metrics)
  const cameras = useAppStore((state) => state.cameras)
  const tracks = useAppStore((state) => state.tracks)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const fetchSystemHealth = useAppStore((state) => state.fetchSystemHealth)
  const fetchMetrics = useAppStore((state) => state.fetchMetrics)
  const fetchCameras = useAppStore((state) => state.fetchCameras)

  useEffect(() => {
    // Initial fetch
    fetchSystemHealth()
    fetchMetrics()
    fetchCameras()

    // Poll every 5 seconds
    const interval = setInterval(() => {
      fetchSystemHealth()
      fetchMetrics()
    }, 5000)

    return () => clearInterval(interval)
  }, [fetchSystemHealth, fetchMetrics, fetchCameras])

  if (backendStatus === 'disconnected') {
    return (
      <EmptyState
        icon={Activity}
        title="Backend Disconnected"
        description="Unable to connect to the BOB Camera backend. Please check that the backend service is running."
        action={
          <button
            onClick={() => {
              fetchSystemHealth()
              fetchMetrics()
            }}
            className="rounded-lg bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
          >
            Retry Connection
          </button>
        }
      />
    )
  }

  if (backendStatus === 'connecting' || !systemHealth) {
    return <PageSpinner />
  }

  const activeCameras = cameras.filter((c) => c.enabled)
  const activeTracks = tracks.filter((t) => t.status === 'active')

  return (
    <div className="space-y-6">
      {/* Header */}
      <div>
        <h1 className="text-3xl font-bold text-gray-900">Dashboard</h1>
        <p className="mt-1 text-sm text-gray-500">
          System overview and real-time monitoring
        </p>
      </div>

      {/* System Status Cards */}
      <CardGrid>
        {/* System Health */}
        <Card>
          <div className="flex items-start justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">System Status</p>
              <div className="mt-2">
                <StatusPill
                  status={
                    systemHealth.status === 'ok'
                      ? 'ok'
                      : systemHealth.status === 'degraded'
                      ? 'warning'
                      : 'error'
                  }
                  label={systemHealth.status}
                />
              </div>
              {systemHealth.uptime && (
                <p className="mt-2 text-xs text-gray-500">
                  Uptime: {formatUptime(systemHealth.uptime)}
                </p>
              )}
            </div>
            <div className="rounded-lg bg-blue-100 p-3">
              <Activity className="h-6 w-6 text-blue-600" />
            </div>
          </div>
        </Card>

        {/* CPU Load */}
        <Card>
          <div className="flex items-start justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">CPU Load</p>
              <p className="mt-2 text-2xl font-bold text-gray-900">
                {systemHealth.cpuLoad !== undefined
                  ? formatPercent(systemHealth.cpuLoad)
                  : 'N/A'}
              </p>
              {metrics && metrics.length > 0 && (
                <p className="mt-1 text-xs text-gray-500">
                  Latest: {formatPercent(metrics[metrics.length - 1].cpu)}
                </p>
              )}
            </div>
            <div className="rounded-lg bg-green-100 p-3">
              <Cpu className="h-6 w-6 text-green-600" />
            </div>
          </div>
        </Card>

        {/* GPU Load */}
        {systemHealth.gpuLoad !== undefined && (
          <Card>
            <div className="flex items-start justify-between">
              <div>
                <p className="text-sm font-medium text-gray-600">GPU Load</p>
                <p className="mt-2 text-2xl font-bold text-gray-900">
                  {formatPercent(systemHealth.gpuLoad)}
                </p>
                {systemHealth.temperature && (
                  <p className="mt-1 text-xs text-gray-500">
                    Temp: {systemHealth.temperature}°C
                  </p>
                )}
              </div>
              <div className="rounded-lg bg-purple-100 p-3">
                <Zap className="h-6 w-6 text-purple-600" />
              </div>
            </div>
          </Card>
        )}

        {/* Memory */}
        <Card>
          <div className="flex items-start justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">Memory</p>
              <p className="mt-2 text-2xl font-bold text-gray-900">
                {systemHealth.memory?.used && systemHealth.memory?.total
                  ? formatPercent(
                      (systemHealth.memory.used / systemHealth.memory.total) * 100
                    )
                  : 'N/A'}
              </p>
              {systemHealth.memory && (
                <p className="mt-1 text-xs text-gray-500">
                  {formatBytes(systemHealth.memory.used)} /{' '}
                  {formatBytes(systemHealth.memory.total)}
                </p>
              )}
            </div>
            <div className="rounded-lg bg-yellow-100 p-3">
              <Thermometer className="h-6 w-6 text-yellow-600" />
            </div>
          </div>
        </Card>

        {/* Disk Space */}
        <Card>
          <div className="flex items-start justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">Disk Space</p>
              <p className="mt-2 text-2xl font-bold text-gray-900">
                {systemHealth.disk?.total && systemHealth.disk?.used
                  ? formatBytes(systemHealth.disk.total - systemHealth.disk.used)
                  : 'N/A'}
              </p>
              {systemHealth.disk && (
                <p className="mt-1 text-xs text-gray-500">
                  {formatBytes(systemHealth.disk.used)} /{' '}
                  {formatBytes(systemHealth.disk.total)} used
                </p>
              )}
            </div>
            <div className="rounded-lg bg-orange-100 p-3">
              <HardDrive className="h-6 w-6 text-orange-600" />
            </div>
          </div>
        </Card>

        {/* Active Cameras */}
        <Card>
          <div className="flex items-start justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">Active Cameras</p>
              <p className="mt-2 text-2xl font-bold text-gray-900">
                {activeCameras.length}
              </p>
              <p className="mt-1 text-xs text-gray-500">
                {cameras.length} total configured
              </p>
            </div>
            <div className="rounded-lg bg-indigo-100 p-3">
              <Camera className="h-6 w-6 text-indigo-600" />
            </div>
          </div>
        </Card>

        {/* Active Tracks */}
        <Card>
          <div className="flex items-start justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">Active Tracks</p>
              <p className="mt-2 text-2xl font-bold text-gray-900">
                {activeTracks.length}
              </p>
              <p className="mt-1 text-xs text-gray-500">
                {tracks.length} total tracks
              </p>
            </div>
            <div className="rounded-lg bg-red-100 p-3">
              <Target className="h-6 w-6 text-red-600" />
            </div>
          </div>
        </Card>

        {/* FPS */}
        {metrics && metrics.length > 0 && metrics[metrics.length - 1].fps && (
          <Card>
            <div className="flex items-start justify-between">
              <div>
                <p className="text-sm font-medium text-gray-600">Processing FPS</p>
                <p className="mt-2 text-2xl font-bold text-gray-900">
                  {metrics[metrics.length - 1].fps!.toFixed(1)}
                </p>
                <p className="mt-1 text-xs text-gray-500">
                  Avg: {(metrics.reduce((sum, m) => sum + (m.fps || 0), 0) / metrics.length).toFixed(1)} fps
                </p>
              </div>
              <div className="rounded-lg bg-teal-100 p-3">
                <TrendingUp className="h-6 w-6 text-teal-600" />
              </div>
            </div>
          </Card>
        )}
      </CardGrid>

      {/* Recent Activity */}
      <Card
        title="Recent Activity"
        subtitle="Latest detections and system events"
      >
        {tracks.length === 0 ? (
          <div className="py-12 text-center">
            <Target className="mx-auto h-12 w-12 text-gray-400" />
            <p className="mt-2 text-sm text-gray-600">No recent activity</p>
            <p className="text-xs text-gray-500">
              Tracks will appear here when objects are detected
            </p>
          </div>
        ) : (
          <div className="space-y-3">
            {tracks.slice(0, 5).map((track) => (
              <div
                key={track.id}
                className="flex items-center justify-between rounded-lg border border-gray-200 p-3 hover:bg-gray-50"
              >
                <div className="flex items-center gap-3">
                  <div className="rounded-lg bg-blue-100 p-2">
                    <Target className="h-4 w-4 text-blue-600" />
                  </div>
                  <div>
                    <p className="text-sm font-medium text-gray-900">
                      Track #{track.id}
                    </p>
                    <p className="text-xs text-gray-500">
                      {track.class} · Confidence: {formatPercent(track.avgConfidence * 100)}
                    </p>
                  </div>
                </div>
                <div className="text-right">
                  <p className="text-xs text-gray-500">
                    {new Date(track.lastSeen).toLocaleTimeString()}
                  </p>
                  <StatusPill
                    status={track.status === 'active' ? 'ok' : 'offline'}
                    label={track.status === 'active' ? 'Active' : 'Ended'}
                  />
                </div>
              </div>
            ))}
          </div>
        )}
      </Card>

      {/* System Info */}
      {systemHealth.versions && (
        <Card title="System Information">
          <dl className="grid grid-cols-2 gap-4 sm:grid-cols-4">
            {Object.entries(systemHealth.versions).map(([key, value]) => (
              <div key={key}>
                <dt className="text-xs font-medium text-gray-500">
                  {key.charAt(0).toUpperCase() + key.slice(1)}
                </dt>
                <dd className="mt-1 text-sm font-semibold text-gray-900">{value}</dd>
              </div>
            ))}
          </dl>
        </Card>
      )}
    </div>
  )
}