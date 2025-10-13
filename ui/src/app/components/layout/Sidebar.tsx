import { useNavigate, useLocation } from 'react-router-dom'
import { Stack, NavLink as MantineNavLink, Badge } from '@mantine/core'
import {
  IconDashboard,
  IconCamera,
  IconVideo,
  IconTarget,
  IconDeviceFloppy,
  IconSettings,
  IconServer,
  IconFileText,
} from '@tabler/icons-react'
import { useAppStore } from '@/app/store'

const navigation = [
  { name: 'Dashboard', href: '/', icon: IconDashboard },
  { name: 'Cameras', href: '/cameras', icon: IconCamera },
  { name: 'Live View', href: '/live', icon: IconVideo },
  { name: 'Tracks', href: '/tracks', icon: IconTarget },
  { name: 'Recordings', href: '/recordings', icon: IconDeviceFloppy },
  { name: 'Settings', href: '/settings', icon: IconSettings },
  { name: 'System', href: '/system', icon: IconServer },
  { name: 'Logs', href: '/logs', icon: IconFileText },
]

export function Sidebar() {
  const navigate = useNavigate()
  const location = useLocation()
  
  const activeCamerasCount = useAppStore((state) => 
    state.cameras.filter((c) => c.enabled).length
  )
  const activeTracksCount = useAppStore((state) => state.tracks.length)

  const getBadge = (href: string) => {
    if (href === '/cameras' && activeCamerasCount > 0) {
      return activeCamerasCount
    }
    if (href === '/tracks' && activeTracksCount > 0) {
      return activeTracksCount
    }
    return undefined
  }

  const isActive = (href: string) => {
    if (href === '/') {
      return location.pathname === '/'
    }
    return location.pathname.startsWith(href)
  }

  return (
    <Stack gap="xs">
      {navigation.map((item) => {
        const badge = getBadge(item.href)
        return (
          <MantineNavLink
            key={item.name}
            label={item.name}
            leftSection={<item.icon size={20} />}
            rightSection={
              badge ? (
                <Badge size="sm" variant="filled" color="blue">
                  {badge}
                </Badge>
              ) : undefined
            }
            active={isActive(item.href)}
            variant="filled"
            onClick={() => navigate(item.href)}
          />
        )
      })}
    </Stack>
  )
}