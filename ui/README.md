# BOB Camera UI

Modern React-based frontend for the BOB (Bird, Object, Bat) Camera tracking system.

## 📚 Documentation

**→ [Complete Documentation Index](DOCUMENTATION.md)** - Start here for all documentation

### Quick Links
- [Getting Started Guide](docs/guides/getting-started.md) - Setup and development (5 min quick start)
- [Architecture Guide](docs/guides/architecture.md) - System design and patterns
- [Project Status](docs/status/project-status.md) - Current progress (75% complete)
- [Page Documentation](docs/pages/) - Detailed docs for each page

## Features

- 🎯 **Real-time Tracking**: Live video streams with detection overlays via ROS2
- 📊 **System Monitoring**: CPU, GPU, memory, and disk usage metrics
- 📹 **Camera Management**: Configure and manage multiple camera sources
- 🎬 **Recording Playback**: Browse and download recorded clips
- ⚙️ **Configuration**: Adjust detection, tracking, and storage settings
- 📝 **Live Logs**: Real-time system logs with filtering
- 🌙 **Dark Mode**: Toggle between light and dark themes
- ♿ **Accessible**: WCAG 2.1 compliant with keyboard navigation

## Tech Stack

- **React 19** - UI framework
- **TypeScript** - Type safety
- **Vite** - Build tool and dev server
- **React Router 7** - Client-side routing
- **Zustand** - State management
- **Mantine v7** - Component library
- **TailwindCSS 4** - Utility-first styling
- **Axios** - HTTP client
- **Zod** - Runtime schema validation
- **Vitest** - Unit testing
- **React Testing Library** - Component testing

## Getting Started

### Prerequisites

- Node.js 20+ and npm
- Backend service running (see main README)

### Installation

```bash
# Install dependencies
npm install

# Copy environment template
cp .env.example .env

# Edit .env with your backend URLs
```

### Development

```bash
# Start dev server with hot reload
npm run dev

# Open http://localhost:5173
```

The dev server includes:
- Hot module replacement (HMR)
- Proxy to backend API (configured in vite.config.ts)
- Source maps for debugging

### Building

```bash
# Type check
npx tsc --noEmit

# Build for production
npm run build

# Preview production build
npm run preview
```

### Testing

```bash
# Run tests in watch mode
npm run test

# Run tests with UI
npm run test:ui

# Generate coverage report
npm run test:coverage
```

### Linting & Formatting

```bash
# Lint code
npm run lint

# Format code
npm run format

# Check formatting
npm run format -- --check
```

## Environment Variables

Create a `.env` file in the `ui/` directory:

```bash
# API Configuration
VITE_API_BASE_URL=/api
VITE_WS_BASE_URL=/ws

# Stream Configuration
VITE_STREAM_PROTOCOL=mjpeg  # Options: hls, mjpeg, none

# Feature Flags (optional)
VITE_ENABLE_RECORDINGS=true
VITE_ENABLE_SETTINGS=true
```

### Environment Variable Notes

- All Vite env vars must be prefixed with `VITE_`
- Variables are embedded at build time (not runtime)
- For runtime config, use the backend `/api/config` endpoint

## Project Structure

```
ui/
├── src/
│   ├── app/
│   │   ├── components/
│   │   │   ├── common/          # Reusable UI components
│   │   │   └── layout/          # Layout components
│   │   ├── pages/               # Page components
│   │   │   ├── Dashboard/
│   │   │   ├── Cameras/
│   │   │   ├── LiveView/
│   │   │   ├── Tracks/
│   │   │   ├── Recordings/
│   │   │   ├── Settings/
│   │   │   ├── System/
│   │   │   └── Logs/
│   │   ├── services/
│   │   │   ├── api.ts           # REST API client
│   │   │   ├── ws.ts            # WebSocket client
│   │   │   └── schema.ts        # Zod schemas
│   │   ├── store/
│   │   │   ├── index.ts         # Combined store
│   │   │   ├── systemSlice.ts
│   │   │   ├── camerasSlice.ts
│   │   │   ├── tracksSlice.ts
│   │   │   └── settingsSlice.ts
│   │   └── router.tsx           # Route configuration
│   ├── lib/
│   │   └── utils.ts             # Utility functions
│   ├── test/
│   │   └── setup.ts             # Test configuration
│   ├── index.css                # Global styles
│   └── main.tsx                 # App entry point
├── public/                      # Static assets
├── .env.example                 # Environment template
├── Dockerfile                   # Multi-stage Docker build
├── nginx.conf                   # Nginx configuration
├── package.json
├── tsconfig.json
├── vite.config.ts
├── vitest.config.ts
└── tailwind.config.js
```

## Docker Deployment

### Build Image

```bash
docker build -t bobcamera/bob-ui:latest .
```

### Run Container

```bash
docker run -d \
  --name bob-ui \
  -p 8080:80 \
  bobcamera/bob-ui:latest
```

### Docker Compose

Add to your `docker-compose.yml`:

```yaml
services:
  ui:
    build: ./ui
    ports:
      - "8080:80"
    depends_on:
      - backend
    environment:
      - NGINX_BACKEND_HOST=backend
      - NGINX_BACKEND_PORT=8080
```

## API Integration

The UI expects the following backend endpoints:

### REST API

- `GET /api/system/health` - System health status
- `GET /api/cameras` - List cameras
- `POST /api/cameras` - Create camera
- `PUT /api/cameras/:id` - Update camera
- `DELETE /api/cameras/:id` - Delete camera
- `POST /api/cameras/:id/test` - Test camera connection
- `GET /api/tracks` - List tracks (paginated)
- `GET /api/recordings` - List recordings (paginated)
- `GET /api/config` - Get configuration
- `PUT /api/config` - Update configuration
- `GET /api/metrics` - Get system metrics
- `GET /api/logs` - Get recent logs

### WebSocket

- `ws://host/ws/telemetry` - Real-time metrics
- `ws://host/ws/events` - Detection events
- `ws://host/ws/logs` - Live log stream

### Adapting to Different APIs

If your backend uses different endpoints, update `src/app/services/api.ts`:

```typescript
// Example: Different endpoint structure
async getSystemHealth(): Promise<SystemHealth> {
  // Change from /api/system/health to /api/v1/health
  const response = await this.client.get('/v1/health')
  return SystemHealthSchema.parse(response.data)
}
```

## State Management

The app uses Zustand with separate slices:

- **systemSlice**: Health, metrics, WebSocket status
- **camerasSlice**: Camera CRUD with optimistic updates
- **tracksSlice**: Tracks, filters, pagination
- **settingsSlice**: Config management, UI preferences

### Accessing State

```typescript
import { useAppStore } from '@/app/store'

function MyComponent() {
  const cameras = useAppStore((state) => state.cameras)
  const fetchCameras = useAppStore((state) => state.fetchCameras)
  
  useEffect(() => {
    fetchCameras()
  }, [])
  
  return <div>{cameras.length} cameras</div>
}
```

## Component Library

### Common Components

- `Card` - Container with optional header
- `StatusPill` - Status indicator with icon
- `Table` - Data table with sorting and pagination
- `Spinner` - Loading indicator
- `Toggle` - Accessible switch
- `EmptyState` - Empty/error state display
- `Toast` - Notification system

### Usage Example

```typescript
import { Card } from '@/app/components/common/Card'
import { StatusPill } from '@/app/components/common/StatusPill'

function Example() {
  return (
    <Card title="System Status">
      <StatusPill status="ok" label="Healthy" />
    </Card>
  )
}
```

## Troubleshooting

### Backend Connection Issues

1. Check `.env` has correct `VITE_API_BASE_URL`
2. Verify backend is running and accessible
3. Check browser console for CORS errors
4. Ensure Nginx proxy is configured correctly

### WebSocket Not Connecting

1. Verify `VITE_WS_BASE_URL` in `.env`
2. Check WebSocket endpoint in browser DevTools
3. Ensure backend WebSocket server is running
4. Check for firewall/proxy blocking WebSocket

### Build Errors

1. Clear node_modules: `rm -rf node_modules && npm install`
2. Clear Vite cache: `rm -rf node_modules/.vite`
3. Check Node.js version: `node --version` (should be 20+)

### Type Errors

1. Run type check: `npx tsc --noEmit`
2. Update types: `npm update @types/react @types/react-dom`
3. Check for missing imports

## Performance

### Optimization Techniques

- Code splitting with React.lazy()
- Memoization with useMemo/useCallback
- Virtual scrolling for large lists
- Debounced search inputs
- Optimistic UI updates
- Service worker for offline support (future)

### Lighthouse Scores

Target scores (desktop):
- Performance: ≥ 90
- Accessibility: 100
- Best Practices: 100
- SEO: 100

## Contributing

1. Create a feature branch
2. Make changes with tests
3. Run linter and tests
4. Submit pull request

### Code Style

- Use TypeScript for all new code
- Follow ESLint rules
- Format with Prettier
- Write tests for new features
- Document complex logic

## License

See main repository LICENSE file.

## Documentation

For comprehensive documentation, see:

- **[Documentation Index](DOCUMENTATION.md)** - Complete documentation hub
- **[Getting Started](docs/guides/getting-started.md)** - Setup and development guide
- **[Architecture](docs/guides/architecture.md)** - System architecture and design
- **[Project Status](docs/status/project-status.md)** - Current progress and roadmap
- **[Page Docs](docs/pages/)** - Individual page documentation
- **[Fixes](docs/fixes/)** - Bug fixes and resolutions

## Support

- GitHub Issues: https://github.com/bobcamera/bobcamera/issues
- Documentation: https://github.com/bobcamera/bobcamera/wiki