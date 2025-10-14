# Getting Started with BOB Camera UI

Get the BOB Camera UI running in 5 minutes and start developing!

## Prerequisites

Before you begin, ensure you have:

- ✅ **Node.js 20+** installed ([Download](https://nodejs.org/))
- ✅ **npm 10+** (comes with Node.js)
- ✅ **Git** installed
- ✅ **Code editor** (VS Code recommended)
- ✅ **BOB Camera backend** running (optional for development)

## Quick Start (5 Minutes)

### 1. Navigate to UI Directory
```bash
cd c:\bobcamera\ui
```

### 2. Install Dependencies
```bash
npm install
```
This will install all required packages (~500MB, takes 2-3 minutes).

### 3. Configure Environment
```bash
# Copy the example environment file
cp .env.example .env
```

Edit `.env` if needed (defaults work for local development):
```env
VITE_API_BASE_URL=/api
VITE_WS_BASE_URL=ws://localhost:8080/ws
VITE_STREAM_PROTOCOL=mjpeg
```

### 4. Start Development Server
```bash
npm run dev
```

You should see:
```
VITE v7.1.2  ready in 500 ms

➜  Local:   http://localhost:5173/
➜  Network: use --host to expose
```

### 5. Open in Browser
Navigate to [http://localhost:5173](http://localhost:5173)

You should see the BOB Camera dashboard! 🎉

### 6. Start Backend (Optional)
If you need the backend running, in a separate terminal:
```bash
cd c:\bobcamera
./run.sh your_config.yaml
```

## What's Included

### Pages

- **Dashboard** (`/`) - System overview with metrics and health status
- **Cameras** (`/cameras`) - Camera management with CRUD operations
- **Live View** (`/live`) - Real-time video streaming with ROS2 integration
- **Tracks** (`/tracks`) - Detection history browser with advanced filtering
- **Recordings** (`/recordings`) - Video clip library with playback
- **Settings** (`/settings`) - System configuration editor
- **System** (`/system`) - System health monitoring
- **Logs** (`/logs`) - Real-time log viewer

### Features

✅ **Dark/Light Mode** - Toggle in header  
✅ **Responsive Design** - Works on mobile and desktop  
✅ **Real-time Updates** - WebSocket integration  
✅ **Offline Mode** - Graceful degradation  
✅ **Type Safety** - Full TypeScript support  
✅ **Modern UI** - Mantine v7 components  
✅ **Fast Development** - Vite HMR  

## Development Workflow

### Daily Development
```bash
# Start dev server (with hot reload)
npm run dev

# In another terminal, run tests in watch mode
npm run test

# Run linting
npm run lint

# Format code
npm run format
```

### Before Committing
```bash
# Type check
npm run typecheck

# Run all tests
npm run test:ci

# Check formatting
npm run format:check

# Lint code
npm run lint
```

### Building for Production
```bash
# Create production build
npm run build

# Preview production build locally
npm run preview
```

## Project Structure

```
ui/
├── src/
│   ├── app/
│   │   ├── components/
│   │   │   ├── layout/          # AppShell, HeaderBar, Sidebar
│   │   │   └── common/          # Reusable components
│   │   ├── pages/               # Route pages
│   │   │   ├── Dashboard/       # ✅ Implemented
│   │   │   ├── Cameras/         # ✅ Implemented
│   │   │   ├── LiveView/        # ✅ Implemented
│   │   │   ├── Tracks/          # ✅ Implemented
│   │   │   ├── Recordings/      # ✅ Implemented
│   │   │   ├── Settings/        # ✅ Implemented
│   │   │   ├── System/          # ⏳ Partial
│   │   │   └── Logs/            # ⏳ Partial
│   │   ├── services/            # API and WebSocket clients
│   │   │   ├── api.ts           # REST API client
│   │   │   ├── ws.ts            # WebSocket client
│   │   │   ├── ros2Client.ts    # ROS2 WebSocket client
│   │   │   └── schema.ts        # Zod schemas
│   │   ├── store/               # Zustand state management
│   │   │   ├── index.ts         # Store setup
│   │   │   ├── systemSlice.ts   # System state
│   │   │   ├── camerasSlice.ts  # Cameras state
│   │   │   ├── tracksSlice.ts   # Tracks state
│   │   │   └── settingsSlice.ts # Settings state
│   │   └── router.tsx           # Route definitions
│   ├── test/                    # Test utilities
│   ├── main.tsx                 # App entry point
│   └── index.css                # Global styles
├── docs/                        # Documentation
├── public/                      # Static assets
├── .env.example                 # Environment template
├── vite.config.ts               # Vite configuration
└── package.json                 # Dependencies
```

## Common Development Tasks

### Creating a New Component

1. **Create Component File**
```tsx
// src/app/components/common/MyComponent.tsx
import { Card, Text } from '@mantine/core'

interface MyComponentProps {
  title: string
  value: string
}

export default function MyComponent({ title, value }: MyComponentProps) {
  return (
    <Card shadow="sm" padding="lg" radius="md" withBorder>
      <Text size="sm" c="dimmed">{title}</Text>
      <Text size="xl" fw={600}>{value}</Text>
    </Card>
  )
}
```

2. **Create Test File**
```tsx
// src/app/components/common/MyComponent.test.tsx
import { describe, it, expect } from 'vitest'
import { render, screen } from '@/test/utils'
import MyComponent from './MyComponent'

describe('MyComponent', () => {
  it('renders title and value', () => {
    render(<MyComponent title="Test" value="123" />)
    expect(screen.getByText('Test')).toBeInTheDocument()
    expect(screen.getByText('123')).toBeInTheDocument()
  })
})
```

3. **Use in Page**
```tsx
import MyComponent from '@/app/components/common/MyComponent'

function MyPage() {
  return <MyComponent title="Users" value="42" />
}
```

### Adding a New API Endpoint

1. **Define Schema**
```tsx
// src/app/services/schema.ts
export const MyDataSchema = z.object({
  id: z.string(),
  name: z.string(),
  value: z.number(),
})

export type MyData = z.infer<typeof MyDataSchema>
```

2. **Add API Method**
```tsx
// src/app/services/api.ts
async getMyData(): Promise<MyData[]> {
  const response = await this.client.get('/my-data')
  return z.array(MyDataSchema).parse(response.data)
}
```

3. **Use in Component**
```tsx
import { apiClient } from '@/app/services/api'

function MyComponent() {
  const [data, setData] = useState<MyData[]>([])

  useEffect(() => {
    apiClient.getMyData().then(setData)
  }, [])

  return <div>{/* render data */}</div>
}
```

### Adding State Management

1. **Create Slice**
```tsx
// src/app/store/mySlice.ts
import { StateCreator } from 'zustand'

export interface MySlice {
  myData: MyData[]
  setMyData: (data: MyData[]) => void
}

export const createMySlice: StateCreator<MySlice> = (set) => ({
  myData: [],
  setMyData: (data) => set({ myData: data }),
})
```

2. **Add to Store**
```tsx
// src/app/store/index.ts
import { createMySlice, MySlice } from './mySlice'

export type AppStore = SystemSlice & CamerasSlice & MySlice

export const useAppStore = create<AppStore>()((...a) => ({
  ...createSystemSlice(...a),
  ...createCamerasSlice(...a),
  ...createMySlice(...a),
}))
```

3. **Use in Component**
```tsx
import { useAppStore } from '@/app/store'

function MyComponent() {
  const myData = useAppStore((state) => state.myData)
  const setMyData = useAppStore((state) => state.setMyData)

  return <div>{/* use myData */}</div>
}
```

### Adding a New Page

1. Create page component in `src/app/pages/MyPage/index.tsx`
2. Add route in `src/app/router.tsx`
3. Add navigation link in `src/app/components/layout/Sidebar.tsx`
4. Add tests in `src/app/pages/MyPage/MyPage.test.tsx`

## Using Mantine Components

### Example: Create a Card

```tsx
import { Card, Text, Button } from '@mantine/core'

function MyCard() {
  return (
    <Card shadow="sm" padding="lg" radius="md" withBorder>
      <Text size="lg" fw={600}>Card Title</Text>
      <Text size="sm" c="dimmed" mt="xs">
        Card description goes here
      </Text>
      <Button mt="md" fullWidth>
        Action
      </Button>
    </Card>
  )
}
```

### Example: Show Notification

```tsx
import { notifications } from '@mantine/notifications'

notifications.show({
  title: 'Success!',
  message: 'Operation completed successfully',
  color: 'green',
})
```

### Example: Open Modal

```tsx
import { openConfirmDialog } from '@/app/components/common/ConfirmDialog'

openConfirmDialog({
  title: 'Confirm Action',
  message: 'Are you sure you want to proceed?',
  onConfirm: () => {
    // Do something
  },
})
```

## Running Tests

### Watch Mode (Development)
```bash
npm run test
```
Tests run automatically when files change.

### Single Run (CI)
```bash
npm run test:ci
```
Runs all tests once with coverage report.

### UI Mode (Visual)
```bash
npm run test:ui
```
Opens Vitest UI in browser for interactive testing.

### Coverage Report
```bash
npm run test:coverage
```
Generates coverage report in `coverage/` directory.

## Troubleshooting

### Port 5173 Already in Use
```bash
# Kill the process using port 5173
npx kill-port 5173

# Or use a different port
npm run dev -- --port 3000
```

### Module Not Found Errors
```bash
# Clear node_modules and reinstall
rm -rf node_modules package-lock.json
npm install
```

### TypeScript Errors
```bash
# Clear TypeScript cache
rm -rf node_modules/.vite
npm run typecheck
```

### Backend Connection Issues
1. Check backend is running on port 8080
2. Verify `.env` has correct API URL
3. Check browser console for CORS errors
4. Try accessing http://localhost:8080/api/system/health directly

### WebSocket Not Connecting
1. Check `VITE_WS_BASE_URL` in `.env`
2. Verify WebSocket endpoint: Open DevTools → Network → WS
3. Check firewall/proxy settings
4. Ensure backend WebSocket server is running

### Mantine Styles Not Loading
1. Check `index.css` imports Mantine CSS
2. Verify `postcss.config.js` has Mantine preset
3. Clear Vite cache: `rm -rf node_modules/.vite`
4. Restart dev server

### Build Errors
1. Clear cache: `rm -rf node_modules/.vite`
2. Reinstall: `rm -rf node_modules && npm install`
3. Check Node version: `node --version` (should be 20+)
4. Run type check: `npx tsc --noEmit`

### Slow Performance
1. Check browser DevTools → Performance
2. Disable browser extensions
3. Clear browser cache
4. Check backend response times
5. Reduce polling frequency in code

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

Then run:
```bash
docker-compose up -d
```

## Useful Resources

### Documentation
- [Mantine UI](https://mantine.dev/) - Component library
- [React Router](https://reactrouter.com/) - Routing
- [Zustand](https://github.com/pmndrs/zustand) - State management
- [Vitest](https://vitest.dev/) - Testing framework
- [Vite](https://vitejs.dev/) - Build tool

### Tools
- [Tabler Icons](https://tabler-icons.io/) - Icon search
- [Mantine Colors](https://mantine.dev/colors-generator/) - Color generator
- [TypeScript Playground](https://www.typescriptlang.org/play) - TS testing

### VS Code Extensions (Recommended)
- ESLint
- Prettier
- Tailwind CSS IntelliSense
- TypeScript Vue Plugin (Volar)
- Error Lens
- GitLens

## Next Steps

### For New Developers
1. ✅ Complete this setup guide
2. 📖 Read [Architecture Guide](./architecture.md)
3. 🔍 Explore completed pages (Dashboard, Cameras, LiveView, Tracks, Settings, Recordings)
4. 🎨 Try modifying a component
5. 🧪 Run tests and see them pass
6. 📋 Pick a task from the project status

### For Experienced Developers
1. ✅ Complete setup
2. 📋 Review project status documentation
3. 🎯 Pick a feature to implement
4. 💻 Follow existing patterns in completed pages
5. 🧪 Write tests for new code
6. 📝 Update documentation
7. 🚀 Submit pull request

## Getting Help

- **Issues**: https://github.com/bobcamera/bobcamera/issues
- **Discussions**: https://github.com/bobcamera/bobcamera/discussions
- **Wiki**: https://github.com/bobcamera/bobcamera/wiki

## You're Ready! 🎉

You now have everything you need to start developing the BOB Camera UI. Happy coding! 🚀

If you get stuck, check the documentation or ask for help in the project chat.