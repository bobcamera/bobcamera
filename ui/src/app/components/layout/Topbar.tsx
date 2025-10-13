import { Bell, Settings, Moon, Sun, Activity } from 'lucide-react'
import { useAppStore } from '@/app/store'
import { StatusPill } from '../common/StatusPill'
import * as DropdownMenu from '@radix-ui/react-dropdown-menu'

export function Topbar() {
  const systemHealth = useAppStore((state) => state.systemHealth)
  const backendStatus = useAppStore((state) => state.backendStatus)
  const wsStatus = useAppStore((state) => state.wsStatus)
  const darkMode = useAppStore((state) => state.uiPreferences.darkMode)
  const toggleDarkMode = useAppStore((state) => state.toggleDarkMode)
  const versions = useAppStore((state) => state.versions)

  const getSystemStatus = () => {
    if (backendStatus === 'disconnected') return 'error'
    if (systemHealth?.status === 'ok') return 'ok'
    if (systemHealth?.status === 'degraded') return 'warning'
    return 'error'
  }

  return (
    <header className="sticky top-0 z-50 border-b border-gray-200 bg-white shadow-sm">
      <div className="flex h-16 items-center justify-between px-6">
        {/* Left: Logo & Title */}
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2">
            <Activity className="h-8 w-8 text-blue-600" />
            <div>
              <h1 className="text-xl font-bold text-gray-900">BOB Camera</h1>
              <p className="text-xs text-gray-500">
                Bird · Object · Bat Tracker
              </p>
            </div>
          </div>
        </div>

        {/* Center: System Status */}
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2">
            <span className="text-sm text-gray-600">System:</span>
            <StatusPill status={getSystemStatus()} label={systemHealth?.status || 'Unknown'} />
          </div>
          <div className="flex items-center gap-2">
            <span className="text-sm text-gray-600">WebSocket:</span>
            <StatusPill
              status={wsStatus === 'connected' ? 'online' : wsStatus === 'connecting' ? 'loading' : 'offline'}
              label={wsStatus}
            />
          </div>
        </div>

        {/* Right: Actions */}
        <div className="flex items-center gap-2">
          {/* Version */}
          {versions && (
            <div className="hidden text-xs text-gray-500 lg:block">
              <div>UI: {versions.ui}</div>
              <div>Backend: {versions.backend || 'N/A'}</div>
            </div>
          )}

          {/* Dark Mode Toggle */}
          <button
            onClick={toggleDarkMode}
            className="rounded-lg p-2 text-gray-600 hover:bg-gray-100 hover:text-gray-900"
            aria-label="Toggle dark mode"
          >
            {darkMode ? <Sun className="h-5 w-5" /> : <Moon className="h-5 w-5" />}
          </button>

          {/* Notifications */}
          <DropdownMenu.Root>
            <DropdownMenu.Trigger asChild>
              <button
                className="relative rounded-lg p-2 text-gray-600 hover:bg-gray-100 hover:text-gray-900"
                aria-label="Notifications"
              >
                <Bell className="h-5 w-5" />
                {/* Badge for unread notifications */}
                <span className="absolute right-1 top-1 h-2 w-2 rounded-full bg-red-500" />
              </button>
            </DropdownMenu.Trigger>
            <DropdownMenu.Portal>
              <DropdownMenu.Content
                className="z-50 min-w-[320px] rounded-lg border border-gray-200 bg-white p-2 shadow-lg"
                sideOffset={5}
                align="end"
              >
                <div className="px-3 py-2">
                  <h3 className="text-sm font-semibold text-gray-900">Notifications</h3>
                </div>
                <DropdownMenu.Separator className="my-1 h-px bg-gray-200" />
                <div className="max-h-[400px] overflow-y-auto">
                  <div className="px-3 py-8 text-center text-sm text-gray-500">
                    No new notifications
                  </div>
                </div>
              </DropdownMenu.Content>
            </DropdownMenu.Portal>
          </DropdownMenu.Root>

          {/* Settings Quick Access */}
          <DropdownMenu.Root>
            <DropdownMenu.Trigger asChild>
              <button
                className="rounded-lg p-2 text-gray-600 hover:bg-gray-100 hover:text-gray-900"
                aria-label="Quick settings"
              >
                <Settings className="h-5 w-5" />
              </button>
            </DropdownMenu.Trigger>
            <DropdownMenu.Portal>
              <DropdownMenu.Content
                className="z-50 min-w-[200px] rounded-lg border border-gray-200 bg-white p-2 shadow-lg"
                sideOffset={5}
                align="end"
              >
                <DropdownMenu.Item
                  className="cursor-pointer rounded px-3 py-2 text-sm text-gray-700 outline-none hover:bg-gray-100"
                  onSelect={() => (window.location.href = '/settings')}
                >
                  Settings
                </DropdownMenu.Item>
                <DropdownMenu.Item
                  className="cursor-pointer rounded px-3 py-2 text-sm text-gray-700 outline-none hover:bg-gray-100"
                  onSelect={() => (window.location.href = '/system')}
                >
                  System Health
                </DropdownMenu.Item>
                <DropdownMenu.Item
                  className="cursor-pointer rounded px-3 py-2 text-sm text-gray-700 outline-none hover:bg-gray-100"
                  onSelect={() => (window.location.href = '/logs')}
                >
                  Logs
                </DropdownMenu.Item>
                <DropdownMenu.Separator className="my-1 h-px bg-gray-200" />
                <DropdownMenu.Item
                  className="cursor-pointer rounded px-3 py-2 text-sm text-gray-700 outline-none hover:bg-gray-100"
                  onSelect={() => window.open('https://github.com/twobitshortofabyte/bobcamera', '_blank')}
                >
                  Documentation
                </DropdownMenu.Item>
              </DropdownMenu.Content>
            </DropdownMenu.Portal>
          </DropdownMenu.Root>
        </div>
      </div>
    </header>
  )
}