import { NavLink } from 'react-router-dom'
import {
  LayoutDashboard,
  Camera,
  Radio,
  Target,
  Video,
  Settings,
  Activity,
  FileText,
  ChevronLeft,
  ChevronRight,
} from 'lucide-react'
import { useAppStore } from '@/app/store'
import { cn } from '@/lib/utils'

interface NavItem {
  to: string
  icon: React.ComponentType<{ className?: string }>
  label: string
  badge?: number
}

const navItems: NavItem[] = [
  { to: '/', icon: LayoutDashboard, label: 'Dashboard' },
  { to: '/cameras', icon: Camera, label: 'Cameras' },
  { to: '/live', icon: Radio, label: 'Live View' },
  { to: '/tracks', icon: Target, label: 'Tracks' },
  { to: '/recordings', icon: Video, label: 'Recordings' },
  { to: '/settings', icon: Settings, label: 'Settings' },
  { to: '/system', icon: Activity, label: 'System' },
  { to: '/logs', icon: FileText, label: 'Logs' },
]

export function Sidebar() {
  const sidebarCollapsed = useAppStore((state) => state.uiPreferences.sidebarCollapsed)
  const toggleSidebar = useAppStore((state) => state.toggleSidebar)
  const activeCamerasCount = useAppStore((state) => 
    state.cameras.filter((c) => c.enabled).length
  )
  const activeTracksCount = useAppStore((state) => state.tracks.length)

  // Add dynamic badges
  const itemsWithBadges = navItems.map((item) => {
    if (item.to === '/cameras') {
      return { ...item, badge: activeCamerasCount }
    }
    if (item.to === '/tracks') {
      return { ...item, badge: activeTracksCount }
    }
    return item
  })

  return (
    <aside
      className={cn(
        'fixed left-0 top-16 z-40 h-[calc(100vh-4rem)] border-r border-gray-200 bg-white transition-all duration-300',
        sidebarCollapsed ? 'w-16' : 'w-64'
      )}
    >
      <nav className="flex h-full flex-col">
        {/* Navigation Items */}
        <div className="flex-1 space-y-1 overflow-y-auto p-3">
          {itemsWithBadges.map((item) => (
            <NavLink
              key={item.to}
              to={item.to}
              end={item.to === '/'}
              className={({ isActive }) =>
                cn(
                  'flex items-center gap-3 rounded-lg px-3 py-2 text-sm font-medium transition-colors',
                  'hover:bg-gray-100 hover:text-gray-900',
                  isActive
                    ? 'bg-blue-50 text-blue-700 hover:bg-blue-100'
                    : 'text-gray-700',
                  sidebarCollapsed && 'justify-center'
                )
              }
              title={sidebarCollapsed ? item.label : undefined}
            >
              <item.icon className="h-5 w-5 flex-shrink-0" />
              {!sidebarCollapsed && (
                <>
                  <span className="flex-1">{item.label}</span>
                  {item.badge !== undefined && item.badge > 0 && (
                    <span className="flex h-5 min-w-[20px] items-center justify-center rounded-full bg-blue-100 px-1.5 text-xs font-semibold text-blue-700">
                      {item.badge}
                    </span>
                  )}
                </>
              )}
            </NavLink>
          ))}
        </div>

        {/* Collapse Toggle */}
        <div className="border-t border-gray-200 p-3">
          <button
            onClick={toggleSidebar}
            className={cn(
              'flex w-full items-center gap-3 rounded-lg px-3 py-2 text-sm font-medium text-gray-700 transition-colors hover:bg-gray-100 hover:text-gray-900',
              sidebarCollapsed && 'justify-center'
            )}
            aria-label={sidebarCollapsed ? 'Expand sidebar' : 'Collapse sidebar'}
          >
            {sidebarCollapsed ? (
              <ChevronRight className="h-5 w-5" />
            ) : (
              <>
                <ChevronLeft className="h-5 w-5" />
                <span>Collapse</span>
              </>
            )}
          </button>
        </div>
      </nav>
    </aside>
  )
}