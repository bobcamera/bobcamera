# BOB Camera UI - Mantine Integration Guide

## Overview

The BOB Camera UI has been upgraded with **Mantine UI v7** components while maintaining **TailwindCSS** for layout and grid utilities. This provides a modern, accessible, and production-ready interface.

## Tech Stack

- **React 19** + **TypeScript** + **Vite**
- **Mantine UI v7** - Component library
- **TailwindCSS v4** - Layout and utilities
- **React Router v7** - Routing
- **Zustand** - State management
- **Axios** - REST API client
- **Native WebSocket** - Real-time events
- **Vitest** + **React Testing Library** - Testing
- **Tabler Icons** - Icon library

## Project Structure

```
ui/
├── src/
│   ├── app/
│   │   ├── components/
│   │   │   ├── layout/
│   │   │   │   ├── AppShell.tsx       # Main layout with Mantine AppShell
│   │   │   │   ├── HeaderBar.tsx      # Header with theme toggle
│   │   │   │   └── Sidebar.tsx        # Navigation sidebar
│   │   │   └── common/
│   │   │       ├── MetricCard.tsx     # Metric display card
│   │   │       ├── StatusBadge.tsx    # Status indicator
│   │   │       ├── DataTable.tsx      # Sortable/paginated table
│   │   │       ├── ConfirmDialog.tsx  # Confirmation modal
│   │   │       ├── FieldRow.tsx       # Form field wrapper
│   │   │       └── EmptyState.tsx     # Empty state component
│   │   ├── pages/
│   │   │   ├── Dashboard/             # Dashboard page
│   │   │   ├── Cameras/               # Camera management
│   │   │   ├── LiveView/              # Live video view
│   │   │   ├── Tracks/                # Detection tracks
│   │   │   ├── Recordings/            # Recorded clips
│   │   │   ├── Settings/              # Configuration
│   │   │   ├── System/                # System health
│   │   │   └── Logs/                  # Log viewer
│   │   ├── services/
│   │   │   ├── api.ts                 # REST API client
│   │   │   ├── ws.ts                  # WebSocket manager
│   │   │   └── schema.ts              # TypeScript types
│   │   ├── store/
│   │   │   ├── index.ts               # Combined store
│   │   │   ├── systemSlice.ts         # System state
│   │   │   ├── camerasSlice.ts        # Camera state
│   │   │   ├── tracksSlice.ts         # Track state
│   │   │   └── settingsSlice.ts       # Settings state
│   │   ├── theme/
│   │   │   └── index.ts               # Mantine theme config
│   │   └── router.tsx                 # Route definitions
│   ├── main.tsx                       # App entry point
│   └── index.css                      # Global styles
├── .env.example                       # Environment variables
├── vite.config.ts                     # Vite configuration
├── postcss.config.js                  # PostCSS with Mantine
└── package.json                       # Dependencies
```

## Key Features

### 1. Mantine AppShell Layout

The main layout uses Mantine's `AppShell` component with:
- Fixed header (60px height)
- Collapsible sidebar (260px width)
- Responsive mobile/desktop behavior
- Automatic padding and spacing

### 2. Theme Configuration

Located in `src/app/theme/index.ts`:
- Default dark mode
- Blue primary color
- Inter font family
- Consistent border radius (md)
- Component-level defaults

### 3. Color Scheme Toggle

The HeaderBar includes a sun/moon icon to toggle between light and dark modes using Mantine's `useMantineColorScheme` hook.

### 4. Backend Integration

- **Health Check**: Automatic backend connectivity detection
- **Mock Mode**: Graceful degradation when backend is offline
- **WebSocket**: Auto-reconnect with exponential backoff
- **API Adapters**: Thin wrappers for backend endpoints

### 5. Common Components

All components use Mantine primitives:

- **MetricCard**: Display metrics with icons and trends
- **StatusBadge**: Color-coded status indicators
- **DataTable**: Sortable, filterable, paginated tables
- **ConfirmDialog**: Modal confirmations using Mantine modals
- **FieldRow**: Consistent form field layout
- **EmptyState**: Friendly empty states with icons

## Environment Variables

Create a `.env` file based on `.env.example`:

```env
# API Configuration
VITE_API_BASE_URL=/api
VITE_WS_BASE_URL=ws://localhost:8080/ws
VITE_STREAM_PROTOCOL=mjpeg
VITE_STREAM_URL=/stream

# Feature Flags
VITE_ENABLE_RECORDINGS=true
VITE_ENABLE_SYSTEM_METRICS=true
VITE_ENABLE_LOGS=true

# Development
VITE_MOCK_MODE=false
VITE_DEBUG=false
```

## Development

### Install Dependencies

```bash
npm install
```

### Run Development Server

```bash
npm run dev
```

The dev server runs on `http://localhost:5173` with proxy to backend on port 8080.

### Build for Production

```bash
npm run build
```

### Run Tests

```bash
npm test              # Run tests
npm run test:ui       # Run tests with UI
npm run test:coverage # Generate coverage report
```

### Lint and Format

```bash
npm run lint          # ESLint
npm run format        # Prettier
```

## Vite Proxy Configuration

The `vite.config.ts` includes proxies for:
- `/api` → `http://localhost:8080`
- `/stream` → `http://localhost:8080`
- `/ws` → `ws://localhost:8080` (WebSocket)

This allows the dev server to communicate with the backend without CORS issues.

## Mantine + Tailwind Coexistence

### Use Mantine For:
- UI components (buttons, inputs, cards, modals)
- Forms and validation
- Notifications and modals
- Theme and color scheme

### Use Tailwind For:
- Layout (flexbox, grid)
- Spacing (padding, margin)
- Responsive design
- Custom utilities

### Example:

```tsx
import { Card, Text, Button } from '@mantine/core'

function MyComponent() {
  return (
    <div className="grid grid-cols-3 gap-4">
      <Card shadow="sm" padding="lg">
        <Text size="lg" fw={600}>Title</Text>
        <Button mt="md">Click Me</Button>
      </Card>
    </div>
  )
}
```

## State Management

### Zustand Store Structure

```typescript
// Access state
const health = useAppStore((state) => state.health)
const cameras = useAppStore((state) => state.cameras)

// Call actions
const fetchSystemHealth = useAppStore((state) => state.fetchSystemHealth)
fetchSystemHealth()
```

### Store Slices

- **systemSlice**: Health, metrics, versions, backend status
- **camerasSlice**: Camera list, CRUD operations
- **tracksSlice**: Detection tracks, filters, pagination
- **settingsSlice**: Configuration, draft changes, diff

## API Service

### Usage

```typescript
import { apiClient } from '@/app/services/api'

// Health check
const isHealthy = await apiClient.checkHealth()

// Get cameras
const cameras = await apiClient.getCameras()

// Update camera
const updated = await apiClient.updateCamera(id, { enabled: true })
```

### Error Handling

All API methods throw errors that should be caught:

```typescript
try {
  const cameras = await apiClient.getCameras()
} catch (error) {
  console.error('Failed to fetch cameras:', error)
  // Show notification
}
```

## WebSocket Service

### Usage

```typescript
import { eventsClient, telemetryClient } from '@/app/services/ws'

// Connect
eventsClient.connect('/events')

// Listen for events
const unsubscribe = eventsClient.onEvent((event) => {
  console.log('Event:', event)
})

// Cleanup
unsubscribe()
eventsClient.disconnect()
```

### Auto-Reconnect

The WebSocket client automatically reconnects with exponential backoff:
- Initial delay: 1 second
- Max delay: 30 seconds
- Max attempts: 10

## Notifications

Use Mantine's notification system:

```typescript
import { notifications } from '@mantine/notifications'

notifications.show({
  title: 'Success',
  message: 'Camera updated successfully',
  color: 'green',
})

notifications.show({
  title: 'Error',
  message: 'Failed to update camera',
  color: 'red',
})
```

## Modals

Use Mantine's modal system:

```typescript
import { modals } from '@mantine/modals'
import { openConfirmDialog } from '@/app/components/common/ConfirmDialog'

// Confirmation dialog
openConfirmDialog({
  title: 'Delete Camera',
  message: 'Are you sure you want to delete this camera?',
  confirmLabel: 'Delete',
  onConfirm: async () => {
    await apiClient.deleteCamera(id)
  },
  color: 'red',
})
```

## Routing

Routes are defined in `src/app/router.tsx`:

```typescript
const router = createBrowserRouter([
  {
    path: '/',
    element: <AppShell />,
    children: [
      { index: true, element: <Dashboard /> },
      { path: 'cameras', element: <Cameras /> },
      { path: 'live', element: <LiveView /> },
      // ... more routes
    ],
  },
])
```

## Docker Build

The UI is built using a multi-stage Dockerfile:

```dockerfile
# Build stage
FROM node:20-alpine AS builder
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

# Production stage
FROM nginx:stable-alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
```

## Troubleshooting

### Backend Connection Issues

1. Check that backend is running on port 8080
2. Verify `.env` has correct `VITE_API_BASE_URL`
3. Check browser console for CORS errors
4. Ensure Vite proxy is configured correctly

### WebSocket Connection Issues

1. Check `VITE_WS_BASE_URL` in `.env`
2. Verify backend WebSocket endpoint is `/ws/events`
3. Check browser console for WebSocket errors
4. Ensure backend supports WebSocket connections

### Styling Issues

1. Ensure Mantine CSS is imported in `index.css`
2. Check PostCSS config includes `postcss-preset-mantine`
3. Verify Tailwind is not conflicting with Mantine
4. Use browser DevTools to inspect styles

### Build Issues

1. Clear `node_modules` and reinstall: `rm -rf node_modules && npm install`
2. Clear Vite cache: `rm -rf node_modules/.vite`
3. Check TypeScript errors: `npm run build`
4. Verify all imports are correct

## Best Practices

### Component Development

1. Use Mantine components as building blocks
2. Keep components small and focused
3. Extract reusable logic to hooks
4. Use TypeScript for type safety
5. Write tests for critical components

### State Management

1. Keep state close to where it's used
2. Use Zustand for global state
3. Use React state for local UI state
4. Avoid prop drilling with context or store

### API Integration

1. Use the API client service
2. Handle errors gracefully
3. Show loading states
4. Cache responses when appropriate
5. Use optimistic updates for better UX

### Performance

1. Use React.memo for expensive components
2. Lazy load routes with React.lazy
3. Debounce search inputs
4. Virtualize long lists
5. Optimize images and assets

## Next Steps

1. **Complete Page Implementations**: Finish all page components
2. **Add Tests**: Write unit and integration tests
3. **Improve Accessibility**: Add ARIA labels and keyboard navigation
4. **Add Analytics**: Track user interactions
5. **Optimize Performance**: Profile and optimize rendering
6. **Add Documentation**: Document all components and APIs
7. **CI/CD**: Set up automated testing and deployment

## Resources

- [Mantine Documentation](https://mantine.dev/)
- [TailwindCSS Documentation](https://tailwindcss.com/)
- [React Router Documentation](https://reactrouter.com/)
- [Zustand Documentation](https://zustand-demo.pmnd.rs/)
- [Vite Documentation](https://vitejs.dev/)

## Support

For issues or questions:
1. Check this documentation
2. Review the code examples
3. Check the browser console for errors
4. Review backend logs
5. Open an issue on GitHub