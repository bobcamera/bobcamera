# Tracks Page Documentation

**Status**: ✅ Complete  
**Component Library**: Mantine UI v7  
**Lines of Code**: ~1,050 total  
**Date Completed**: January 2024

## Overview

The **Tracks page** provides a comprehensive interface for browsing, filtering, and analyzing object detection tracks across all cameras in the BOB Camera system. It features advanced filtering capabilities, real-time search, data export functionality, and detailed track information in a drawer panel.

## Features

### 1. Track Browsing
- **Paginated Table**: Displays tracks with sortable columns
- **Responsive Design**: Adapts to different screen sizes with horizontal scrolling
- **Row Click**: Opens detailed drawer for selected track
- **Hover Effects**: Visual feedback on row hover
- **Empty States**: Contextual messages for no data or filtered results

### 2. Advanced Filtering
- **Collapsible Filter Panel**: Toggle visibility to save screen space
- **Real-time Search**: Client-side search by track ID, class, or camera name
- **Camera Filter**: Dropdown to filter by specific camera
- **Class Filter**: Searchable dropdown with all 19 COCO object classes
- **Confidence Filter**: Numeric input for minimum confidence threshold (0-100%)
- **Date Range**: From/To date pickers with validation
- **Active Filter Badges**: Visual indicators showing applied filters
- **Clear All**: One-click removal of all filters
- **Filter Persistence**: Filters maintained during pagination

### 3. Data Export
- **CSV Export**: Download tracks as comma-separated values
- **JSON Export**: Download tracks with full metadata
- **Filename Convention**: Includes date stamp (e.g., `tracks-2024-01-15.csv`)
- **Camera Name Resolution**: Exports camera names instead of IDs
- **Formatted Data**: Proper percentage formatting and timestamps

### 4. Track Detail Drawer
- **Side Panel**: Slides in from right with track details
- **Thumbnail Display**: Shows track snapshot if available
- **Status Badge**: Color-coded status indicator (active/lost/completed)
- **Basic Information**: Track ID, camera, class, confidence, detection count
- **Timeline**: First seen, last seen, duration calculation
- **Statistics**: Detections per minute, average interval between detections
- **Copy to Clipboard**: One-click copy of track ID
- **Status Cards**: Contextual information based on track status

### 5. Statistics Summary
- **Total Tracks**: Count of all tracks matching filters
- **Active Tracks**: Count of currently active tracks
- **Average Confidence**: Mean confidence across all visible tracks
- **Total Detections**: Sum of all detections in visible tracks

### 6. Pagination
- **Page Size Selector**: Choose 25, 50, or 100 tracks per page
- **Page Navigation**: Standard pagination controls
- **Result Counter**: Shows current range (e.g., "Showing 1 to 50 of 234 tracks")
- **Total Count**: Displays total number of tracks

### 7. Loading & Error States
- **Loading Spinner**: Shown while fetching data
- **Connection Status**: Detects backend disconnection
- **Error Alerts**: Dismissible error messages
- **Retry Button**: Reload page on connection failure
- **Empty State**: Helpful message when no tracks exist

## Component Structure

### Main Component: `Tracks` (`index.tsx`)

```
Tracks
├── Header
│   ├── Title & Description
│   └── Action Buttons (Filters, Export)
├── Filter Panel (Collapsible)
│   ├── Search Input
│   ├── Camera Select
│   ├── Class Select
│   ├── Min Confidence Input
│   └── Date Range Pickers
├── Active Filter Badges
├── Error Alert (conditional)
├── Tracks Table Card
│   ├── Table (with 8 columns)
│   └── Pagination Controls
├── Statistics Summary (4 cards)
└── Track Detail Drawer (modal)
```

### Sub-Component: `TrackDetailDrawer` (`TrackDetailDrawer.tsx`)

```
TrackDetailDrawer (Mantine Drawer)
├── Header (Track icon + title)
├── Thumbnail Section
├── Basic Information
│   ├── Track ID (with copy button)
│   ├── Camera Name
│   ├── Object Class Badge
│   ├── Average Confidence
│   └── Detection Count
├── Timeline Section
│   ├── First Seen
│   ├── Last Seen
│   └── Duration
├── Statistics Section
│   ├── Detections per Minute
│   └── Average Interval
└── Status Card (contextual)
```

## Mantine Components Used

### Main Page (20+ components)
- `Container` - Page wrapper
- `Title`, `Text` - Typography
- `Group`, `Stack` - Layout
- `Button`, `ActionIcon` - Actions
- `TextInput` - Search
- `Select` - Dropdowns
- `DatePickerInput` - Date pickers
- `NumberInput` - Confidence input
- `Badge` - Filter badges
- `Card` - Containers
- `Table` - Data table
- `Pagination` - Page navigation
- `Menu` - Export menu
- `Collapse` - Collapsible panel
- `Alert` - Error messages
- `Loader`, `Center` - Loading states
- `ThemeIcon` - Statistic icons

### Track Detail Drawer (15+ components)
- `Drawer` - Side panel
- `Title`, `Text` - Typography
- `Group`, `Stack` - Layout
- `Badge` - Status indicators
- `CopyButton`, `ActionIcon` - Copy action
- `Divider` - Section separators
- `Card` - Status cards
- `Image` - Thumbnail display
- `AspectRatio` - Image aspect ratio

## Table Columns

| Column | Description | Sortable | Width |
|--------|-------------|----------|-------|
| Track ID | Unique identifier | No | 120px |
| Camera | Camera name | Yes | 150px |
| Class | Object class | Yes | 120px |
| Confidence | Average confidence % | Yes | 120px |
| Detections | Number of detections | Yes | 120px |
| Duration | Track duration | Yes | 120px |
| First Seen | Start timestamp | Yes | 180px |
| Last Seen | End timestamp | Yes | 180px |

## Filter Options

### Camera Filter
- Dropdown with all available cameras
- Shows camera name
- "All Cameras" option to clear filter

### Class Filter
- Searchable dropdown
- 19 COCO object classes:
  - person, bicycle, car, motorcycle, airplane
  - bus, train, truck, boat, traffic light
  - fire hydrant, stop sign, parking meter, bench, bird
  - cat, dog, horse, sheep, cow

### Confidence Filter
- Number input (0-100%)
- Filters tracks with average confidence >= threshold
- Default: No filter

### Date Range Filter
- From date picker
- To date picker
- Validates that "from" is before "to"
- Default: Last 7 days

### Search Filter
- Real-time client-side search
- Searches in: Track ID, Class, Camera Name
- Case-insensitive
- Debounced for performance

## Export Functionality

### CSV Export
```csv
Track ID,Camera,Class,Confidence,Detections,Duration,First Seen,Last Seen
track-001,Front Door,person,85.5%,42,00:05:23,2024-01-15 10:30:00,2024-01-15 10:35:23
```

### JSON Export
```json
[
  {
    "id": "track-001",
    "cameraId": "cam-1",
    "cameraName": "Front Door",
    "class": "person",
    "confidence": 0.855,
    "detectionCount": 42,
    "duration": 323,
    "firstSeen": "2024-01-15T10:30:00Z",
    "lastSeen": "2024-01-15T10:35:23Z",
    "status": "completed"
  }
]
```

## Statistics Calculations

### Total Tracks
```typescript
const totalTracks = filteredTracks.length
```

### Active Tracks
```typescript
const activeTracks = filteredTracks.filter(t => t.status === 'active').length
```

### Average Confidence
```typescript
const avgConfidence = filteredTracks.reduce((sum, t) => sum + t.confidence, 0) / totalTracks
```

### Total Detections
```typescript
const totalDetections = filteredTracks.reduce((sum, t) => sum + t.detectionCount, 0)
```

## Track Detail Calculations

### Duration
```typescript
const duration = new Date(track.lastSeen).getTime() - new Date(track.firstSeen).getTime()
const minutes = Math.floor(duration / 60000)
const seconds = Math.floor((duration % 60000) / 1000)
```

### Detections per Minute
```typescript
const durationMinutes = duration / 60000
const detectionsPerMinute = track.detectionCount / durationMinutes
```

### Average Interval
```typescript
const avgInterval = duration / (track.detectionCount - 1)
```

## Status Color Coding

| Status | Color | Badge Variant |
|--------|-------|---------------|
| active | green | filled |
| lost | yellow | light |
| completed | blue | light |

## API Integration

### Fetch Tracks
```typescript
const tracks = await apiClient.getTracks({
  from: filters.dateFrom,
  to: filters.dateTo,
  cameraId: filters.camera,
  class: filters.class,
  minConfidence: filters.minConfidence / 100,
  page: currentPage,
  pageSize: pageSize,
})
```

### Response Schema
```typescript
interface PaginatedResponse<Track> {
  items: Track[]
  total: number
  page: number
  pageSize: number
  totalPages: number
}
```

## State Management

### Local State (11 variables)
- `tracks` - Track list
- `loading` - Loading state
- `error` - Error message
- `selectedTrack` - Currently selected track
- `drawerOpen` - Drawer visibility
- `filtersOpen` - Filter panel visibility
- `searchQuery` - Search input
- `filters` - Filter values
- `currentPage` - Current page number
- `pageSize` - Items per page
- `sortBy` - Sort column and direction

### Zustand Store
- `cameras` - Camera list for filter dropdown
- `backendStatus` - Connection status

## Performance Optimizations

### Client-side Search
- Debounced input (300ms)
- Searches in memory (no API calls)
- Filters applied after search

### Memoized Calculations
- Filtered tracks computed once per filter change
- Statistics computed once per filtered tracks change
- Camera name lookup memoized

### Pagination
- Server-side pagination for scalability
- Only fetches current page data
- Reduces memory usage

## Testing Scenarios

### Manual Testing Checklist
- [ ] Page loads with tracks
- [ ] Pagination works
- [ ] Filters apply correctly
- [ ] Search filters tracks
- [ ] Export to CSV works
- [ ] Export to JSON works
- [ ] Track detail drawer opens
- [ ] Copy track ID works
- [ ] Loading state shows
- [ ] Error state shows
- [ ] Empty state shows
- [ ] Backend disconnected state shows

### Edge Cases
- [ ] No tracks available
- [ ] All tracks filtered out
- [ ] Invalid date range
- [ ] Backend disconnected
- [ ] Very long track IDs
- [ ] Special characters in search
- [ ] Large number of tracks (1000+)

## Known Limitations

1. **Client-side Search**: Search only works on currently loaded page
2. **Export Limit**: Exports only visible/filtered tracks, not all tracks
3. **Thumbnail Loading**: Thumbnails may not be available for all tracks
4. **Date Range**: Limited to date picker capabilities

## Future Enhancements

1. **Advanced Filtering**
   - Multiple class selection
   - Confidence range (min/max)
   - Duration range filter
   - Detection count range

2. **Visualization**
   - Track timeline chart
   - Confidence distribution chart
   - Class distribution pie chart
   - Camera activity heatmap

3. **Bulk Operations**
   - Select multiple tracks
   - Bulk export
   - Bulk delete
   - Bulk tagging

4. **Track Comparison**
   - Compare two tracks side-by-side
   - Highlight differences
   - Merge similar tracks

5. **Real-time Updates**
   - WebSocket integration for live tracks
   - Auto-refresh active tracks
   - Notification for new tracks

## Code Statistics

- **Main Page**: ~750 lines
- **Track Detail Drawer**: ~300 lines
- **Total**: ~1,050 lines
- **Mantine Components**: 25+
- **Tabler Icons**: 15+
- **State Variables**: 11
- **API Calls**: 1 (getTracks)

## Related Documentation

- [Architecture Guide](../guides/architecture.md)
- [Getting Started](../guides/getting-started.md)
- [API Documentation](../guides/api.md)