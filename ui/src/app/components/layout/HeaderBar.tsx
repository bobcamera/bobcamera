import { Group, Text, ActionIcon, useMantineColorScheme, Badge } from '@mantine/core'
import { IconSun, IconMoon, IconSearch } from '@tabler/icons-react'
import { useAppStore } from '@/app/store'
import { type ReactNode } from 'react'

interface HeaderBarProps {
  burger: ReactNode
}

export function HeaderBar({ burger }: HeaderBarProps) {
  const { colorScheme, toggleColorScheme } = useMantineColorScheme()
  const { backendStatus, versions, gitHash } = useAppStore()

  return (
    <Group h="100%" px="md" justify="space-between">
      <Group>
        {burger}
        <Text size="xl" fw={700}>
          BOB Camera
        </Text>
        {versions && (
          <Group gap="xs">
            <Badge size="sm" variant="light" color="blue">
              FE v{versions.ui}
            </Badge>
            <Badge size="sm" variant="light" color="gray">
              BE v{versions.backend}
            </Badge>
            {gitHash && (
              <Badge size="sm" variant="light" color="gray" style={{ fontFamily: 'monospace' }}>
                {gitHash}
              </Badge>
            )}
          </Group>
        )}
      </Group>

      <Group>
        <Badge
          size="lg"
          variant="dot"
          color={backendStatus === 'online' ? 'green' : 'red'}
        >
          {backendStatus === 'online' ? 'Online' : 'Offline'}
        </Badge>

        <ActionIcon
          variant="subtle"
          size="lg"
          aria-label="Search"
          color="gray"
        >
          <IconSearch size={20} />
        </ActionIcon>

        <ActionIcon
          variant="subtle"
          size="lg"
          onClick={toggleColorScheme}
          aria-label="Toggle color scheme"
        >
          {colorScheme === 'dark' ? <IconSun size={20} /> : <IconMoon size={20} />}
        </ActionIcon>
      </Group>
    </Group>
  )
}