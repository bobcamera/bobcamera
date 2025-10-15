import { createBrowserRouter, RouterProvider } from 'react-router-dom'
import { AppShell } from './components/layout/AppShell'
import { Dashboard } from './pages/Dashboard'
import { Cameras } from './pages/Cameras'
import { LiveView } from './pages/LiveView'
import Tracks from './pages/Tracks'
import Recordings from './pages/Recordings'
import { Settings } from './pages/Settings'
import System from './pages/System'
// NOTE: Logs page not yet implemented - uncomment when pages/Logs/index.tsx is created
// import Logs from './pages/Logs'
import { ErrorBoundary } from './components/common/ErrorBoundary'

const router = createBrowserRouter([
  {
    path: '/',
    element: <AppShell />,
    errorElement: <ErrorBoundary />,
    children: [
      {
        index: true,
        element: <Dashboard />,
      },
      {
        path: 'cameras',
        element: <Cameras />,
      },
      {
        path: 'live',
        element: <LiveView />,
      },
      {
        path: 'tracks',
        element: <Tracks />,
      },
      {
        path: 'recordings',
        element: <Recordings />,
      },
      {
        path: 'settings',
        element: <Settings />,
      },
      {
        path: 'system',
        element: <System />,
      },
      // Logs route commented out - page not yet implemented
      // {
      //   path: 'logs',
      //   element: <Logs />,
      // },
    ],
  },
])

export function AppRouter() {
  return <RouterProvider router={router} />
}
