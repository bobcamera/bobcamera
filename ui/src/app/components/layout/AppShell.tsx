import { type ReactNode } from 'react'
import { Outlet } from 'react-router-dom'
import { Topbar } from './Topbar'
import { Sidebar } from './Sidebar'
import { ToastProvider } from '../common/Toast'
import { useAppStore } from '@/app/store'
import { cn } from '@/lib/utils'

interface AppShellProps {
  children?: ReactNode
}

export function AppShell({ children }: AppShellProps) {
  const sidebarCollapsed = useAppStore((state) => state.uiPreferences.sidebarCollapsed)

  return (
    <ToastProvider>
      <div className="min-h-screen bg-gray-50">
        <Topbar />
        <div className="flex">
          <Sidebar />
          <main
            className={cn(
              'flex-1 transition-all duration-300',
              sidebarCollapsed ? 'ml-16' : 'ml-64'
            )}
          >
            <div className="p-6">
              {children || <Outlet />}
            </div>
          </main>
        </div>
      </div>
    </ToastProvider>
  )
}