import { type ReactNode, useEffect } from 'react'
import { Outlet } from 'react-router-dom'
import { AppShell as MantineAppShell, Burger } from '@mantine/core'
import { useDisclosure } from '@mantine/hooks'
import { HeaderBar } from './HeaderBar'
import { Sidebar } from './Sidebar'
import { useAppStore } from '@/app/store'
import { wsClient } from '@/app/services/ws'
import { apiClient } from '@/app/services/api'

interface AppShellProps {
  children?: ReactNode
}

export function AppShell({ children }: AppShellProps) {
  const [mobileOpened, { toggle: toggleMobile }] = useDisclosure()
  const [desktopOpened, { toggle: toggleDesktop }] = useDisclosure(true)
  const { mockMode, setBackendStatus, setMockMode } = useAppStore()

  // Initialize backend connection
  useEffect(() => {
    const initialize = async () => {
      try {
        const isHealthy = await apiClient.checkHealth()
        
        if (isHealthy) {
          setBackendStatus('online')
          setMockMode(false)
          wsClient.connect()
        } else {
          setBackendStatus('offline')
          setMockMode(true)
        }
      } catch (error) {
        console.error('Initialization failed:', error)
        setBackendStatus('offline')
        setMockMode(true)
      }
    }

    initialize()

    return () => {
      wsClient.disconnect()
    }
  }, [setBackendStatus, setMockMode])

  // Periodic health check
  useEffect(() => {
    const healthCheck = setInterval(async () => {
      try {
        const isHealthy = await apiClient.checkHealth()
        
        if (isHealthy && mockMode) {
          setBackendStatus('online')
          setMockMode(false)
          wsClient.connect()
        } else if (!isHealthy && !mockMode) {
          setBackendStatus('offline')
          setMockMode(true)
          wsClient.disconnect()
        }
      } catch (error) {
        // Continue with current mode
      }
    }, 10000)

    return () => clearInterval(healthCheck)
  }, [mockMode, setBackendStatus, setMockMode])

  return (
    <MantineAppShell
      header={{ height: 60 }}
      navbar={{
        width: 260,
        breakpoint: 'sm',
        collapsed: { mobile: !mobileOpened, desktop: !desktopOpened },
      }}
      padding="md"
    >
      <MantineAppShell.Header>
        <HeaderBar
          burger={
            <>
              <Burger
                opened={mobileOpened}
                onClick={toggleMobile}
                hiddenFrom="sm"
                size="sm"
              />
              <Burger
                opened={desktopOpened}
                onClick={toggleDesktop}
                visibleFrom="sm"
                size="sm"
              />
            </>
          }
        />
      </MantineAppShell.Header>

      <MantineAppShell.Navbar p="md">
        <Sidebar />
      </MantineAppShell.Navbar>

      <MantineAppShell.Main>
        {children || <Outlet />}
      </MantineAppShell.Main>
    </MantineAppShell>
  )
}