import { Table, ScrollArea, Text, Group, ActionIcon, Pagination } from '@mantine/core'
import { IconChevronUp, IconChevronDown } from '@tabler/icons-react'
import { useState } from 'react'

export interface Column<T> {
  key: string
  label: string
  sortable?: boolean
  render?: (row: T) => React.ReactNode
  width?: string | number
}

interface DataTableProps<T> {
  columns: Column<T>[]
  data: T[]
  keyExtractor: (row: T) => string | number
  onRowClick?: (row: T) => void
  sortable?: boolean
  pagination?: {
    page: number
    pageSize: number
    total: number
    onPageChange: (page: number) => void
  }
  emptyMessage?: string
}

export function DataTable<T>({
  columns,
  data,
  keyExtractor,
  onRowClick,
  sortable = true,
  pagination,
  emptyMessage = 'No data available',
}: DataTableProps<T>) {
  const [sortBy, setSortBy] = useState<string | null>(null)
  const [sortOrder, setSortOrder] = useState<'asc' | 'desc'>('asc')

  const handleSort = (key: string) => {
    if (!sortable) return
    
    if (sortBy === key) {
      setSortOrder(sortOrder === 'asc' ? 'desc' : 'asc')
    } else {
      setSortBy(key)
      setSortOrder('asc')
    }
  }

  const sortedData = sortBy
    ? [...data].sort((a, b) => {
        const aVal = (a as any)[sortBy]
        const bVal = (b as any)[sortBy]
        
        if (aVal < bVal) return sortOrder === 'asc' ? -1 : 1
        if (aVal > bVal) return sortOrder === 'asc' ? 1 : -1
        return 0
      })
    : data

  return (
    <>
      <ScrollArea>
        <Table striped highlightOnHover>
          <Table.Thead>
            <Table.Tr>
              {columns.map((column) => (
                <Table.Th
                  key={column.key}
                  style={{ width: column.width, cursor: column.sortable !== false && sortable ? 'pointer' : 'default' }}
                  onClick={() => column.sortable !== false && handleSort(column.key)}
                >
                  <Group gap="xs" wrap="nowrap">
                    <Text size="sm" fw={600}>
                      {column.label}
                    </Text>
                    {sortable && column.sortable !== false && sortBy === column.key && (
                      <ActionIcon size="xs" variant="transparent">
                        {sortOrder === 'asc' ? <IconChevronUp size={14} /> : <IconChevronDown size={14} />}
                      </ActionIcon>
                    )}
                  </Group>
                </Table.Th>
              ))}
            </Table.Tr>
          </Table.Thead>
          <Table.Tbody>
            {sortedData.length === 0 ? (
              <Table.Tr>
                <Table.Td colSpan={columns.length}>
                  <Text ta="center" c="dimmed" py="xl">
                    {emptyMessage}
                  </Text>
                </Table.Td>
              </Table.Tr>
            ) : (
              sortedData.map((row) => (
                <Table.Tr
                  key={keyExtractor(row)}
                  onClick={() => onRowClick?.(row)}
                  style={{ cursor: onRowClick ? 'pointer' : 'default' }}
                >
                  {columns.map((column) => (
                    <Table.Td key={column.key}>
                      {column.render ? column.render(row) : String((row as any)[column.key] ?? '')}
                    </Table.Td>
                  ))}
                </Table.Tr>
              ))
            )}
          </Table.Tbody>
        </Table>
      </ScrollArea>

      {pagination && pagination.total > pagination.pageSize && (
        <Group justify="center" mt="md">
          <Pagination
            value={pagination.page}
            onChange={pagination.onPageChange}
            total={Math.ceil(pagination.total / pagination.pageSize)}
          />
        </Group>
      )}
    </>
  )
}