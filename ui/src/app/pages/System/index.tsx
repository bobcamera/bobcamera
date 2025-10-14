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
  Thermometer,
  Zap,
  Server,
} from 'lucide-react'
import { formatBytes, formatPercent, formatUptime } from '@/lib/utils'
import * as Tabs from '@radix-ui/react-tabs'

export function System() {
  const systemHealth = useAppStore((state) => state.systemHealth)
  const metrics = useAppStore((state) => state.metrics)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const fetchSystemHealth = useAppStore((state) => state.fetchSystemHealth)
  const fetchMetrics = useAppStore((state) => state.fetchMetrics)

  useEffect(() => {
    fetchSystemHealth()
    fetchMetrics()

    // Poll every 5 seconds
    const interval = setInterval(() => {
      fetchSystemHealth()
      fetchMetrics()
    }, 5000)

    return () => clearInterval(interval)
  }, [fetchSystemHealth, fetchMetrics])

  if (backendStatus === 'connecting' || !systemHealth) {
    return <PageSpinner />
  }

  if (backendStatus === 'disconnected') {
    return (
      <EmptyState
        icon={<Activity />}
        title="Backend Disconnected"
        description="Unable to load system information. Please check your connection."
        action={
          <button
            onClick={() => {
              fetchSystemHealth()
              fetchMetrics()
            }}
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
      <div>
        <h1 className="text-3xl font-bold text-gray-900">System Health</h1>
        <p className="mt-1 text-sm text-gray-500">
          Monitor system resources and service status
        </p>
      </div>

      {/* Overall Status */}
      <Card>
        <div className="flex items-center justify-between">
          <div>
            <h2 className="text-lg font-semibold text-gray-900">System Status</h2>
            <p className="mt-1 text-sm text-gray-500">
              Overall health: {systemHealth.status}
            </p>
          </div>
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
          <div className="mt-4 border-t border-gray-200 pt-4">
            <p className="text-sm text-gray-600">
              Uptime: <span className="font-medium text-gray-900">{formatUptime(systemHealth.uptime)}</span>
            </p>
          </div>
        )}
      </Card>

      {/* Tabs */}
      <Tabs.Root defaultValue="resources">
        <Tabs.List className="flex gap-2 border-b border-gray-200">
          <Tabs.Trigger
            value="resources"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Resources
          </Tabs.Trigger>
          <Tabs.Trigger
            value="services"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Services
          </Tabs.Trigger>
          <Tabs.Trigger
            value="environment"
            className="border-b-2 border-transparent px-4 py-2 text-sm font-medium text-gray-600 hover:text-gray-900 data-[state=active]:border-blue-600 data-[state=active]:text-blue-600"
          >
            Environment
          </Tabs.Trigger>
        </Tabs.List>

        {/* Resources Tab */}
        <Tabs.Content value="resources" className="mt-6">
          <CardGrid>
            {/* CPU */}
            <Card>
              <div className="flex items-start justify-between">
                <div className="flex-1">
                  <div className="flex items-center gap-2">
                    <Cpu className="h-5 w-5 text-green-600" />
                    <h3 className="font-semibold text-gray-900">CPU</h3>
                  </div>
                  <div className="mt-4 space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Load</span>
                      <span className="text-sm font-medium text-gray-900">
                        {systemHealth.cpuLoad !== undefined
                          ? formatPercent(systemHealth.cpuLoad)
                          : 'N/A'}
                      </span>
                    </div>
                    {metrics && metrics.length > 0 && (
                      <>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Average</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatPercent(
                              metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length
                            )}
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Peak</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatPercent(Math.max(...metrics.map((m) => m.cpu)))}
                          </span>
                        </div>
                      </>
                    )}
                  </div>
                </div>
              </div>
            </Card>

            {/* GPU */}
            {systemHealth.gpuLoad !== undefined && (
              <Card>
                <div className="flex items-start justify-between">
                  <div className="flex-1">
                    <div className="flex items-center gap-2">
                      <Zap className="h-5 w-5 text-purple-600" />
                      <h3 className="font-semibold text-gray-900">GPU</h3>
                    </div>
                    <div className="mt-4 space-y-2">
                      <div className="flex items-center justify-between">
                        <span className="text-sm text-gray-600">Load</span>
                        <span className="text-sm font-medium text-gray-900">
                          {formatPercent(systemHealth.gpuLoad)}
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              </Card>
            )}

            {/* Memory */}
            <Card>
              <div className="flex items-start justify-between">
                <div className="flex-1">
                  <div className="flex items-center gap-2">
                    <Thermometer className="h-5 w-5 text-yellow-600" />
                    <h3 className="font-semibold text-gray-900">Memory</h3>
                  </div>
                  <div className="mt-4 space-y-2">
                    {systemHealth.memory && (
                      <>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Used</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatBytes(systemHealth.memory.used)}
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Total</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatBytes(systemHealth.memory.total)}
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Usage</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatPercent(
                              (systemHealth.memory.used / systemHealth.memory.total) * 100
                            )}
                          </span>
                        </div>
                      </>
                    )}
                  </div>
                </div>
              </div>
            </Card>

            {/* Disk */}
            <Card>
              <div className="flex items-start justify-between">
                <div className="flex-1">
                  <div className="flex items-center gap-2">
                    <HardDrive className="h-5 w-5 text-orange-600" />
                    <h3 className="font-semibold text-gray-900">Disk</h3>
                  </div>
                  <div className="mt-4 space-y-2">
                    {systemHealth.disk && (
                      <>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Used</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatBytes(systemHealth.disk.used)}
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Free</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatBytes(systemHealth.disk.total - systemHealth.disk.used)}
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-sm text-gray-600">Total</span>
                          <span className="text-sm font-medium text-gray-900">
                            {formatBytes(systemHealth.disk.total)}
                          </span>
                        </div>
                      </>
                    )}
                  </div>
                </div>
              </div>
            </Card>
          </CardGrid>
        </Tabs.Content>

        {/* Services Tab */}
        <Tabs.Content value="services" className="mt-6">
          <Card>
            <div className="space-y-3">
              <EmptyState
                icon={<Server />}
                title="No service information"
                description="Service status is not available"
              />
            </div>
          </Card>
        </Tabs.Content>

        {/* Environment Tab */}
        <Tabs.Content value="environment" className="mt-6">
          <Card title="Version Information">
            {systemHealth.versions ? (
              <dl className="grid gap-4 sm:grid-cols-2">
                {Object.entries(systemHealth.versions).map(([key, value]) => (
                  <div key={key}>
                    <dt className="text-sm font-medium text-gray-600">
                      {key.charAt(0).toUpperCase() + key.slice(1)}
                    </dt>
                    <dd className="mt-1 font-mono text-sm font-semibold text-gray-900">
                      {value}
                    </dd>
                  </div>
                ))}
              </dl>
            ) : (
              <p className="text-sm text-gray-500">No version information available</p>
            )}
          </Card>
        </Tabs.Content>
      </Tabs.Root>
    </div>
  )
}