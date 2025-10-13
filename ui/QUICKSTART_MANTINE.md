# BOB Camera UI - Quick Start with Mantine

## 🚀 Get Started in 5 Minutes

### 1. Install Dependencies

```bash
cd ui
npm install
```

### 2. Configure Environment

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` if needed (defaults work for local development):

```env
VITE_API_BASE_URL=/api
VITE_WS_BASE_URL=ws://localhost:8080/ws
VITE_STREAM_PROTOCOL=mjpeg
```

### 3. Start Development Server

```bash
npm run dev
```

Open [http://localhost:5173](http://localhost:5173) in your browser.

### 4. Start Backend (if not running)

In a separate terminal:

```bash
cd ..
./run.sh your_config.yaml
```

## 📦 What's Included

### Pages

- **Dashboard** (`/`) - System overview with metrics
- **Cameras** (`/cameras`) - Camera management
- **Live View** (`/live`) - Real-time video feed
- **Tracks** (`/tracks`) - Detection history
- **Recordings** (`/recordings`) - Saved clips
- **Settings** (`/settings`) - Configuration
- **System** (`/system`) - System health
- **Logs** (`/logs`) - Log viewer

### Features

✅ **Dark/Light Mode** - Toggle in header  
✅ **Responsive Design** - Works on mobile and desktop  
✅ **Real-time Updates** - WebSocket integration  
✅ **Offline Mode** - Graceful degradation  
✅ **Type Safety** - Full TypeScript support  
✅ **Modern UI** - Mantine components  
✅ **Fast Development** - Vite HMR  

## 🎨 Using Mantine Components

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

## 🔧 Common Tasks

### Add a New Page

1. Create page component in `src/app/pages/MyPage/index.tsx`
2. Add route in `src/app/router.tsx`
3. Add navigation link in `src/app/components/layout/Sidebar.tsx`

### Add API Endpoint

1. Add method to `src/app/services/api.ts`
2. Define types in `src/app/services/schema.ts`
3. Use in component:

```tsx
import { apiClient } from '@/app/services/api'

const data = await apiClient.getMyData()
```

### Add State

1. Create slice in `src/app/store/mySlice.ts`
2. Add to store in `src/app/store/index.ts`
3. Use in component:

```tsx
import { useAppStore } from '@/app/store'

const myData = useAppStore((state) => state.myData)
const setMyData = useAppStore((state) => state.setMyData)
```

## 🐛 Troubleshooting

### Backend Not Connecting

Check that:
1. Backend is running on port 8080
2. `.env` has correct API URL
3. Vite proxy is working (check console)

### WebSocket Not Connecting

Check that:
1. Backend WebSocket is on `/ws/events`
2. `.env` has correct WS URL
3. Browser console for errors

### Styles Not Working

Check that:
1. Mantine CSS is imported in `index.css`
2. PostCSS config is correct
3. No conflicting Tailwind classes

## 📚 Learn More

- [Full Documentation](./MANTINE_SETUP.md)
- [Mantine Docs](https://mantine.dev/)
- [Component Examples](https://mantine.dev/core/button/)

## 🎯 Next Steps

1. Explore the Dashboard page
2. Try toggling dark/light mode
3. Check the Cameras page
4. View live detections
5. Customize the theme in `src/app/theme/index.ts`

Happy coding! 🚀