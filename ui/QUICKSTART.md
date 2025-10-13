# BOB Camera UI - Quick Start Guide

Get the BOB Camera UI running in 5 minutes!

## Prerequisites

- ✅ Node.js 20+ installed
- ✅ npm or pnpm installed
- ✅ BOB Camera backend running (or mock API)

## Step 1: Install Dependencies

```bash
cd ui
npm install
```

## Step 2: Configure Environment

```bash
# Copy the example environment file
cp .env.example .env

# Edit .env with your settings
# For local development with backend on localhost:8080:
VITE_API_BASE_URL=http://localhost:8080/api
VITE_WS_BASE_URL=ws://localhost:8080/ws
VITE_STREAM_PROTOCOL=mjpeg
```

## Step 3: Start Development Server

```bash
npm run dev
```

The UI will be available at **http://localhost:5173**

## Step 4: Explore the UI

### Dashboard (/)
- System health overview
- Active cameras and tracks
- Real-time metrics

### Cameras (/cameras)
- Add/edit/delete cameras
- Test camera connections
- Enable/disable cameras

### Live View (/live)
- Real-time video streams
- Detection overlays
- Confidence threshold controls

### Tracks (/tracks)
- Browse all detections
- Filter by camera, class, confidence
- View track details

### Recordings (/recordings)
- Browse recorded clips
- Download recordings
- Filter by date and camera

### Settings (/settings)
- Configure detection parameters
- Adjust tracking settings
- Manage storage options

### System (/system)
- Monitor CPU, GPU, memory, disk
- View service health
- Check version information

### Logs (/logs)
- Real-time log streaming
- Filter by log level
- Export logs

## Development Workflow

### Running Tests

```bash
# Watch mode
npm run test

# With UI
npm run test:ui

# Coverage report
npm run test:coverage
```

### Linting & Formatting

```bash
# Lint code
npm run lint

# Format code
npm run format
```

### Building for Production

```bash
# Type check
npx tsc --noEmit

# Build
npm run build

# Preview build
npm run preview
```

## Docker Deployment

### Quick Start with Docker

```bash
# Build image
docker build -t bobcamera/bob-ui:latest .

# Run container
docker run -d \
  --name bob-ui \
  -p 8080:80 \
  bobcamera/bob-ui:latest

# Access at http://localhost:8080
```

### Docker Compose

Add to your `docker-compose.yml`:

```yaml
version: '3.8'

services:
  backend:
    image: bobcamera/bob-ros2-prod:latest
    # ... backend config ...

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

Then run:

```bash
docker-compose up -d
```

## Troubleshooting

### Backend Connection Failed

**Problem**: UI shows "Backend Disconnected"

**Solutions**:
1. Check backend is running: `curl http://localhost:8080/api/system/health`
2. Verify `.env` has correct `VITE_API_BASE_URL`
3. Check for CORS issues in browser console
4. Ensure backend allows requests from UI origin

### WebSocket Not Connecting

**Problem**: Live updates not working

**Solutions**:
1. Check `VITE_WS_BASE_URL` in `.env`
2. Verify WebSocket endpoint: Open DevTools → Network → WS
3. Check firewall/proxy settings
4. Ensure backend WebSocket server is running

### Build Errors

**Problem**: `npm run build` fails

**Solutions**:
1. Clear cache: `rm -rf node_modules/.vite`
2. Reinstall: `rm -rf node_modules && npm install`
3. Check Node version: `node --version` (should be 20+)
4. Run type check: `npx tsc --noEmit`

### Port Already in Use

**Problem**: `Error: Port 5173 is already in use`

**Solutions**:
1. Kill process: `lsof -ti:5173 | xargs kill -9` (Mac/Linux)
2. Use different port: `npm run dev -- --port 3000`
3. Check for other Vite instances

### Slow Performance

**Problem**: UI is laggy or slow

**Solutions**:
1. Check browser DevTools → Performance
2. Disable browser extensions
3. Clear browser cache
4. Check backend response times
5. Reduce polling frequency in code

## Common Tasks

### Adding a New Page

1. Create page component in `src/app/pages/NewPage/index.tsx`
2. Add route in `src/app/router.tsx`
3. Add navigation item in `src/app/components/layout/Sidebar.tsx`
4. Add tests in `src/app/pages/NewPage/NewPage.test.tsx`

### Adding a New API Endpoint

1. Add Zod schema in `src/app/services/schema.ts`
2. Add method in `src/app/services/api.ts`
3. Add store action in appropriate slice
4. Use in component with `useAppStore`

### Customizing Styles

1. Global styles: Edit `src/index.css`
2. Tailwind config: Edit `tailwind.config.js`
3. Component styles: Use Tailwind classes
4. Custom utilities: Add to `src/lib/utils.ts`

### Debugging

1. **React DevTools**: Install browser extension
2. **Zustand DevTools**: Enabled in development
3. **Network Tab**: Check API requests/responses
4. **Console**: Check for errors and warnings
5. **Source Maps**: Debug TypeScript in browser

## Next Steps

- 📖 Read [README.md](./README.md) for detailed documentation
- 🏗️ Read [ARCHITECTURE.md](./ARCHITECTURE.md) for architecture details
- 🧪 Write tests for your features
- 🎨 Customize the theme and branding
- 🚀 Deploy to production

## Getting Help

- **Issues**: https://github.com/bobcamera/bobcamera/issues
- **Discussions**: https://github.com/bobcamera/bobcamera/discussions
- **Wiki**: https://github.com/bobcamera/bobcamera/wiki

## Tips & Tricks

### Hot Reload Not Working

```bash
# Restart dev server
npm run dev
```

### Clear All Caches

```bash
rm -rf node_modules/.vite
rm -rf dist
npm run dev
```

### Check Bundle Size

```bash
npm run build
npx vite-bundle-visualizer
```

### Update Dependencies

```bash
# Check for updates
npm outdated

# Update all
npm update

# Update specific package
npm install package@latest
```

### Environment-Specific Builds

```bash
# Development
npm run build -- --mode development

# Staging
npm run build -- --mode staging

# Production
npm run build -- --mode production
```

## Success! 🎉

You should now have the BOB Camera UI running. Start by:

1. Adding a camera in the Cameras page
2. Viewing the live stream in Live View
3. Monitoring system health in System page
4. Checking logs in Logs page

Happy tracking! 🎯