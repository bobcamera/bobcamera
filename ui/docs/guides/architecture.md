# BOB Camera UI Architecture

## Overview

This document describes the architecture and design decisions for the BOB Camera React frontend.

## Design Principles

### 1. Frontend-Only Changes
- **No backend modifications**: All integration via existing REST/WebSocket APIs
- **Graceful degradation**: UI handles missing/disabled backend features
- **Configurable endpoints**: All API URLs via environment variables

### 2. Type Safety
- **Runtime validation**: Zod schemas for all API responses
- **Compile-time safety**: TypeScript strict mode
- **Schema-first**: Define data contracts before implementation

### 3. Separation of Concerns
- **Services layer**: API and WebSocket clients isolated from UI
- **State management**: Zustand slices by domain
- **Component hierarchy**: Layout → Pages → Common components

### 4. Performance
- **Optimistic updates**: Immediate UI feedback for mutations
- **Debounced inputs**: Reduce unnecessary API calls
- **Memoization**: Prevent unnecessary re-renders
- **Code splitting**: Lazy load routes (future enhancement)

### 5. Accessibility
- **Keyboard navigation**: All interactive elements accessible
- **ARIA labels**: Screen reader support
- **Focus management**: Logical tab order
- **Color contrast**: WCAG 2.1 AA compliance

## Architecture Layers

```
┌─────────────────────────────────────────┐
│           Presentation Layer            │
│  (Pages, Components, Hooks)             │
├─────────────────────────────────────────┤
│          State Management Layer         │
│  (Zustand Slices, Computed Values)      │
├─────────────────────────────────────────┤
│           Services Layer                │
│  (API Client, WebSocket Client)         │
├─────────────────────────────────────────┤
│          Validation Layer               │
│  (Zod Schemas, Type Guards)             │
├─────────────────────────────────────────┤
│           Backend APIs                  │
│  (REST, WebSocket, Streams)             │
└─────────────────────────────────────────┘
```

## State Management

### Zustand Store Architecture

We use a **slice pattern** to organize state by domain:

```typescript
// Combined store
const useAppStore = create<AppState>()(
  devtools(
    (...args) => ({
      ...createSystemSlice(...args),
      ...createCamerasSlice(...args),
      ...createTracksSlice(...args),
      ...createSettingsSlice(...args),
    })
  )
)
```

### Slice Responsibilities

#### systemSlice
- System health status
- Real-time metrics (CPU, GPU, memory, disk)
- Backend connection status
- WebSocket connection status
- Feature flags
- Version information

#### camerasSlice
- Camera list (CRUD operations)
- Selected camera
- Optimistic updates for mutations
- Camera test results

#### tracksSlice
- Track list with pagination
- Live events buffer (max 1000 items)
- Filters (camera, class, confidence, date range)
- Selected track for detail view

#### settingsSlice
- Current configuration
- Draft configuration (for editing)
- Diff computation
- UI preferences (dark mode, sidebar state, table density)
- LocalStorage persistence

### State Update Patterns

#### Optimistic Updates
```typescript
// Immediate UI update
set((state) => ({
  cameras: state.cameras.map((c) =>
    c.id === id ? { ...c, ...updates } : c
  ),
}))

// Then sync with backend
try {
  await api.updateCamera(id, updates)
} catch (err) {
  // Rollback on error
  set((state) => ({ cameras: originalCameras }))
}
```

#### Computed Values
```typescript
// Derived state as getters
const activeCameras = useAppStore((state) =>
  state.cameras.filter((c) => c.enabled)
)
```

## Services Layer

### API Client (api.ts)

**Responsibilities:**
- HTTP request/response handling
- Error interception and transformation
- Response validation with Zod
- Base URL configuration from env

**Key Features:**
- Axios instance with interceptors
- Automatic error handling
- Type-safe methods
- Adapter pattern for backend differences

**Example:**
```typescript
async getSystemHealth(): Promise<SystemHealth> {
  const response = await this.client.get('/system/health')
  return SystemHealthSchema.parse(response.data)
}
```

### WebSocket Client (ws.ts)

**Responsibilities:**
- WebSocket connection management
- Automatic reconnection with exponential backoff
- Event handler registration
- Multiple endpoint support

**Key Features:**
- Reconnection logic (max 10 attempts, 1s-30s backoff)
- Event-based API
- Status tracking (connecting, connected, disconnected, error)
- Graceful disconnect vs auto-reconnect

**Example:**
```typescript
wsClient.connect('telemetry')
wsClient.on('telemetry', 'metrics', (data) => {
  updateMetrics(data)
})
```

## Component Architecture

### Component Hierarchy

```
AppShell
├── Topbar (status, version, quick actions)
├── Sidebar (navigation)
└── Outlet (page content)
    ├── Dashboard
    ├── Cameras
    │   └── CameraDrawer
    ├── LiveView
    │   └── VideoPlayer
    ├── Tracks
    │   └── TrackDetailPanel
    ├── Recordings
    ├── Settings
    ├── System
    └── Logs
```

### Common Components

#### Card
Flexible container with optional header and actions.

```typescript
<Card title="System Status" description="Current health">
  <StatusPill status="ok" label="Healthy" />
</Card>
```

#### Table
Full-featured data table with sorting, pagination, and empty states.

```typescript
<Table
  columns={columns}
  data={items}
  onRowClick={handleRowClick}
  pagination={{ page, pageSize, total, onPageChange }}
/>
```

#### StatusPill
Visual status indicator with icon and label.

```typescript
<StatusPill status="ok" label="Online" />
<StatusPill status="error" label="Offline" />
```

### Page Components

Each page is self-contained with:
- Data fetching in useEffect
- Local state for UI interactions
- Error handling and loading states
- Empty state displays

**Pattern:**
```typescript
export function PageName() {
  const data = useAppStore((state) => state.data)
  const fetchData = useAppStore((state) => state.fetchData)
  
  useEffect(() => {
    fetchData()
  }, [fetchData])
  
  if (loading) return <PageSpinner />
  if (error) return <EmptyState />
  
  return <div>{/* content */}</div>
}
```

## Routing

### Route Configuration

```typescript
const router = createBrowserRouter([
  {
    path: '/',
    element: <AppShell />,
    errorElement: <ErrorBoundary />,
    children: [
      { index: true, element: <Dashboard /> },
      { path: 'cameras', element: <Cameras /> },
      { path: 'live', element: <LiveView /> },
      { path: 'tracks', element: <Tracks /> },
      { path: 'recordings', element: <Recordings /> },
      { path: 'settings', element: <Settings /> },
      { path: 'system', element: <System /> },
      { path: 'logs', element: <Logs /> },
    ],
  },
])
```

### Navigation

- **Sidebar**: Primary navigation with active state
- **Topbar**: Quick actions and status
- **Breadcrumbs**: Future enhancement for nested routes

## Data Flow

### Typical Flow for Data Fetching

```
User Action
    ↓
Component calls store action
    ↓
Store action calls API service
    ↓
API service makes HTTP request
    ↓
Response validated with Zod
    ↓
Store updates state
    ↓
Component re-renders
```

### WebSocket Event Flow

```
Backend emits event
    ↓
WebSocket client receives
    ↓
Event validated with Zod
    ↓
Registered handlers called
    ↓
Store updates state
    ↓
Components re-render
```

## Error Handling

### Levels of Error Handling

1. **API Level**: Axios interceptors catch HTTP errors
2. **Service Level**: Try-catch in service methods
3. **Store Level**: Error state in slices
4. **Component Level**: Error boundaries and empty states
5. **User Level**: Toast notifications

### Error Display Strategy

- **Toast**: Transient errors (save failed, connection lost)
- **Empty State**: Missing data or features
- **Inline**: Form validation errors
- **Error Boundary**: Unhandled exceptions

## Testing Strategy

### Unit Tests
- Utility functions
- Zod schemas
- Store slices (actions and selectors)

### Component Tests
- Render with mock data
- User interactions
- Conditional rendering
- Accessibility

### Integration Tests
- Full user flows
- API mocking with MSW
- WebSocket mocking

### E2E Tests (Future)
- Critical paths
- Cross-browser testing
- Performance monitoring

## Build & Deployment

### Build Process

```
TypeScript → Vite → Optimized Bundle
    ↓
Static Assets (HTML, JS, CSS)
    ↓
Docker Image (Nginx)
    ↓
Container Registry
```

### Docker Multi-Stage Build

1. **Builder Stage**: Install deps, build app
2. **Production Stage**: Nginx serves static files

### Nginx Configuration

- **History fallback**: Serve index.html for all routes
- **API proxy**: Forward /api/* to backend
- **WebSocket proxy**: Forward /ws/* to backend
- **Gzip compression**: Reduce transfer size
- **Cache headers**: Optimize static assets

## Performance Optimizations

### Implemented

- Debounced search inputs
- Throttled scroll handlers
- Memoized computed values
- Optimistic UI updates
- Efficient re-render prevention

### Future Enhancements

- React.lazy() for code splitting
- Virtual scrolling for large lists
- Service worker for offline support
- Image lazy loading
- Request deduplication

## Security Considerations

### Implemented

- XSS protection via React's escaping
- CSRF protection (backend responsibility)
- Secure headers in Nginx
- Input validation with Zod
- No sensitive data in localStorage

### Best Practices

- Sanitize user input
- Validate all API responses
- Use HTTPS in production
- Implement CSP headers
- Regular dependency updates

## Accessibility

### WCAG 2.1 AA Compliance

- ✅ Keyboard navigation
- ✅ ARIA labels and roles
- ✅ Color contrast ratios
- ✅ Focus indicators
- ✅ Screen reader support
- ✅ Semantic HTML

### Testing Tools

- axe DevTools
- Lighthouse accessibility audit
- Manual keyboard testing
- Screen reader testing (NVDA, JAWS)

## Future Enhancements

### Planned Features

1. **Advanced Filtering**: Saved filter presets
2. **Data Visualization**: Charts for metrics over time
3. **Notifications**: Push notifications for events
4. **Multi-language**: i18n support
5. **Themes**: Custom color schemes
6. **Export**: PDF reports, data exports
7. **Collaboration**: Multi-user support
8. **Mobile**: Responsive mobile views

### Technical Debt

- Add E2E tests with Playwright
- Implement service worker
- Add request caching layer
- Optimize bundle size
- Add performance monitoring

## Maintenance

### Regular Tasks

- Update dependencies monthly
- Review and fix security vulnerabilities
- Monitor bundle size
- Check Lighthouse scores
- Update documentation

### Monitoring

- Error tracking (Sentry)
- Performance monitoring (Web Vitals)
- User analytics (privacy-respecting)
- API response times

## Conclusion

This architecture provides a solid foundation for a production-ready React application that integrates with the BOB Camera backend without requiring any backend changes. The modular design, type safety, and comprehensive error handling ensure maintainability and reliability.