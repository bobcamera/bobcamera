# BOB Camera UI

Modern React-based frontend for the BOB (Bird, Object, Bat) Camera tracking system.

## 📚 Documentation

**→ [Complete Documentation Index](DOCUMENTATION.md)** - Start here for all documentation

### Quick Links
- [Getting Started Guide](docs/guides/getting-started.md) - Setup and development (5 min quick start)
- [Architecture Guide](docs/guides/architecture.md) - System design and patterns
- [Project Status](docs/status/project-status.md) - Current progress (75% complete)
- [Page Documentation](docs/pages/) - Detailed docs for each page

### 🧪 Testing & PR Preparation
- **[TEST_QUICK_START.md](TEST_QUICK_START.md)** - ⚡ Fast-track testing guide (30min - 2hrs)
- [Comprehensive Testing Guide](docs/guides/comprehensive-testing-guide.md) - Full testing strategy (4-5hrs)
- [QA Requirements Report](docs/status/QA_REQUIREMENTS_REPORT.md) - Complete QA verification

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
- **Mantine v8** - Component library
- **TailwindCSS 4** - Utility-first styling
- **Axios** - HTTP client
- **Zod** - Runtime schema validation
- **Vitest** - Unit testing
- **React Testing Library** - Component testing

## Getting Started

### Prerequisites

- **Node.js 20+** and npm 10+ (for local builds)
- **Docker** 20.10+ (for containerized builds)
- Backend service running (for runtime, not required for build)

### Local Installation & Development

#### Install dependencies with npm ci

```bash
# Install dependencies deterministically (recommended)
npm ci

# Or use npm install if you want to update package-lock.json
npm install

# Copy environment template
cp .env.example .env

# Edit .env with your backend URLs (defaults work for local dev)
```

#### Start development server

```bash
# Start Vite dev server with hot reload
npm run dev

# Open http://localhost:5173
```

The dev server includes:
- Hot module replacement (HMR)
- Proxy to backend API (configured in vite.config.ts)
- Source maps for debugging
- Mock mode support (VITE_MOCK_MODE=true)

#### Type checking

```bash
# Type check without emitting
npx tsc --noEmit

# Or use npm script
npm run typecheck
```

#### Building locally

```bash
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

# Format code with Prettier
npm run format

# Check formatting without changing files
npm run format:check
```

## Environment Variables

### Configuration

Create a `.env` file in the `ui/` directory (copy from `.env.example`):

```bash
cp .env.example .env
```

### Available Variables

**Backend API**:
- `VITE_API_BASE_URL` - REST API base path (default: `/api`)
- `VITE_WS_BASE_URL` - WebSocket base URL (default: `ws://localhost:8080/ws`)
- `VITE_ROS2_WS_URL` - ROS2 bridge WebSocket (default: `ws://localhost:9090`)

**Video Stream**:
- `VITE_STREAM_PROTOCOL` - Stream type: `ros2`, `hls`, `mjpeg`, or `none` (default: `ros2`)
- `VITE_STREAM_URL` - Stream endpoint path (default: `/stream`)

**Feature Flags**:
- `VITE_MOCK_MODE` - Use mock backend for development (default: `false`)
- `VITE_ENABLE_RECORDINGS` - Show recordings page (default: `true`)
- `VITE_ENABLE_SYSTEM_METRICS` - Show system metrics (default: `true`)
- `VITE_ENABLE_LOGS` - Show logs page (default: `true`)

**UI Configuration**:
- `VITE_APP_TITLE` - App title in header (default: `BOB Camera System`)
- `VITE_DEBUG` - Verbose console logging (default: `false`)

### Important Notes

- **Build-time embedding**: All `VITE_*` variables are embedded at build time, not read at runtime
- **For runtime configuration**: Use the backend `/api/config` endpoint
- **Git commit**: See [Build & Version Metadata](#build--version-metadata) below
- **Case-sensitive**: Variable names are case-sensitive in Vite

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
├── Dockerfile                   # Alpine + Nginx production image
├── nginx.conf                   # Nginx reverse proxy & SPA routing
├── package.json
├── tsconfig.json
├── vite.config.ts
├── vitest.config.ts
└── tailwind.config.js
```

## Docker Deployment

### 📦 Build & Version Metadata

The build process automatically captures git commit information:

**Priority for commit hash** (checked in order):
1. `GIT_COMMIT` environment variable (passed via Docker build-arg)
2. Local git repository (`git rev-parse --short HEAD`)
3. Fallback to `'unknown'` (expected in CI/Docker contexts without .git)

This is handled gracefully - builds never fail due to missing git info.

### Build Docker Image

#### Quick Build (no git tracking)
```bash
# Builds with GIT_COMMIT=unknown
docker build -t bobcamera/bob-ui:latest .
```

#### Build with git commit info
```bash
# Auto-detect current commit and pass to build
docker build \
  --build-arg GIT_COMMIT=$(git rev-parse --short HEAD) \
  -t bobcamera/bob-ui:latest \
  .
```

#### Using the Makefile (recommended)
```bash
# Auto-detects git commit
make docker-build

# Or with custom tag
make docker-build IMAGE_TAG=bobcamera/bob-ui:v1.0.0
```

### Run Container

```bash
# Run UI on port 8080
docker run -d \
  --name bob-ui \
  -p 8080:80 \
  bobcamera/bob-ui:latest

# Access at http://localhost:8080
```

### Docker Compose

Add to your `docker-compose.yml`:

```yaml
services:
  ui:
    build:
      context: ./ui
      # Optional: pass git commit at build time
      args:
        GIT_COMMIT: ${GIT_COMMIT:-unknown}
    ports:
      - "8080:80"
    depends_on:
      - backend
    environment:
      # Override API URLs if backend is not on localhost
      # (Note: These are build-time, adjust image or mount config)
      # For dynamic URLs, use backend /api/config endpoint
```

### Build Architecture

The Dockerfile uses **multi-stage building**:

```
Stage 1: Builder (node:18-alpine)
├─ Install build dependencies
├─ Run npm ci (deterministic installs)
├─ Build React app → dist/
└─ Accept GIT_COMMIT build-arg (optional)

Stage 2: Production (nginx:stable-alpine)
├─ Copy nginx config from builder
├─ Copy dist/ from builder
├─ Serve with Nginx on port 80
└─ Include healthcheck
```

**Benefits**:
- ✅ No Node.js in final image (smaller size)
- ✅ Reproducible builds regardless of .git presence
- ✅ Deterministic with `npm ci --ignore-scripts`
- ✅ Proper layer caching for faster rebuilds
- ✅ Works in CI/CD without git history

### Build Reproducibility

For CI/CD pipelines that don't have .git:

```bash
# Simulate "no .git" scenario
git archive --format=tar HEAD | tar -x -C /tmp/bob-ui-archive
cd /tmp/bob-ui-archive/ui

# Build will still succeed, using GIT_COMMIT=unknown
docker build -t bobcamera/bob-ui:latest .
```

### Size & Performance

- **Final image**: ~120 MB (Nginx Alpine + React bundle)
- **Build time**: ~2-3 minutes (depends on npm install cache)
- **Startup time**: <1 second
- **Memory usage**: ~50-100 MB at runtime

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

### Local Build Issues

#### "npm install fails with permission error"
```bash
# Clear npm cache
npm cache clean --force

# Reinstall cleanly
rm -rf node_modules package-lock.json
npm ci
```

#### "Cannot find module" or type errors
```bash
# Clear all caches and reinstall
rm -rf node_modules .vite
npm ci

# Type check
npm run typecheck
```

#### "Node.js version mismatch"
```bash
# Check your version (should be 20+)
node --version

# Use nvm to manage versions
nvm install 20
nvm use 20
```

### Docker Build Issues

#### "fatal: not a git repository" during Docker build
**This is expected and not an error!** The build will complete with `GIT_COMMIT=unknown`

Solution: This happens because `.git` is excluded from Docker context (by design).
- ✅ Build completes successfully
- ✅ UI functions normally
- ✅ Version display shows "unknown" (graceful fallback)

#### "docker build hangs or times out"
```bash
# Check Docker resources
docker system df

# Try with increased timeout
docker build --progress=plain -t bobcamera/bob-ui:latest .

# Or use BuildKit
DOCKER_BUILDKIT=1 docker build -t bobcamera/bob-ui:latest .
```

#### "npm ci fails in Docker"
```bash
# Clear Docker build cache and rebuild
docker build --no-cache -t bobcamera/bob-ui:latest .

# Check Docker disk space
docker system prune -a  # ⚠️ removes all unused images/containers
```

### Runtime Issues

#### UI loads but shows blank page
1. Check browser console for JavaScript errors
2. Verify backend is running: `curl http://localhost:8080/api/system/health`
3. Check network tab in DevTools for failed requests
4. Ensure VITE_API_BASE_URL matches your deployment

#### "Cannot connect to backend" or API 404 errors
```bash
# Verify backend is running
curl -i http://localhost:8080/api/system/health

# Check your .env file
cat .env | grep VITE_API

# For Docker, verify networks
docker network ls
docker network inspect bridge
```

#### WebSocket connection fails
1. Verify `VITE_WS_BASE_URL` in `.env` points to correct host
2. Check WebSocket is not blocked by firewall
3. Verify backend WebSocket endpoint is running
4. In browser DevTools → Network → WS, check connection status

#### Video stream not showing
```bash
# Verify stream is accessible
curl -i http://localhost:8080/stream

# Check VITE_STREAM_PROTOCOL setting
cat .env | grep VITE_STREAM

# Try mjpeg if ros2 doesn't work (faster debugging)
# Edit .env and set VITE_STREAM_PROTOCOL=mjpeg
```

### Troubleshooting Matrix

| Problem | Cause | Solution |
|---------|-------|----------|
| `npm run build` fails locally | Missing build deps | `npm ci && npm run build` |
| Docker build fails with git error | `.git` not in context | Normal behavior, build succeeds with `GIT_COMMIT=unknown` |
| Docker build very slow | npm cache miss | Use `--no-cache` only when needed |
| UI blank screen | API unreachable | Check backend health: `curl /api/system/health` |
| WebSocket fails | Wrong URL or firewall | Verify `VITE_WS_BASE_URL` in `.env` |
| Version shows "unknown" | No git in Docker context | Expected; use `--build-arg GIT_COMMIT=<hash>` to override |
| Module not found after git pull | Dependency changes | `rm -rf node_modules && npm ci` |
| TypeScript errors | Type mismatch | `npm run typecheck` → fix errors |
| Hot reload not working | HMR config issue | Restart dev server or clear `.vite` cache |

### Debug Mode

Enable debug logging to troubleshoot issues:

```bash
# In .env
VITE_DEBUG=true

# Then check browser console for detailed logs
npm run dev
```

### Getting Help

If you encounter issues:

1. **Check logs**: Browser console + terminal output
2. **Verify environment**: Correct `.env` file, backend running
3. **Clear caches**: `npm ci` + Docker `--no-cache`
4. **Search existing issues**: GitHub issue tracker
5. **Report with details**:
   - Output of `node --version && npm --version`
   - Your `.env` file (sanitized)
   - Full error message from console/terminal
   - Steps to reproduce

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