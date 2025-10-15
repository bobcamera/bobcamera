import { BrowserRouter, Routes, Route } from 'react-router-dom'
import { useEffect } from 'react'
import { Header } from './components/Header'
import { Sidebar } from './components/Sidebar'
import { Dashboard } from './routes/dashboard'
import { Settings } from './routes/settings'
import { useAppStore } from './store/useAppStore'
import { wsClient } from './lib/ws'
import { apiClient } from './lib/api'
import { mockDetectionGenerator } from './lib/mock/mockDetections'

function App() {
  const { mockMode, setBackendStatus, setMockMode, addDetections } = useAppStore()

  useEffect(() => {
    // Initialize the application
    const initialize = async () => {
      try {
        console.log('[App] Checking backend health...')
        // Check if backend is available
        const isHealthy = await apiClient.checkHealth()
        
        if (isHealthy) {
          console.log('[App] Backend is healthy, connecting...')
          setBackendStatus('online')
          setMockMode(false)
          // Connect to WebSocket
          wsClient.connect()
        } else {
          console.log('[App] Backend is offline, starting mock mode...')
          setBackendStatus('offline')
          setMockMode(true)
          // Start mock detection generator
          mockDetectionGenerator.start((detections) => {
            console.log('[App] Mock detections generated:', detections.length)
            addDetections(detections)
          })
        }
      } catch (error) {
        console.error('[App] Initialization failed:', error)
        setBackendStatus('offline')
        setMockMode(true)
        // Start mock detection generator
        mockDetectionGenerator.start((detections) => {
          console.log('[App] Mock detections generated (error path):', detections.length)
          addDetections(detections)
        })
      }
    }

    initialize()

    // Cleanup on unmount
    return () => {
      wsClient.disconnect()
      mockDetectionGenerator.stop()
    }
  }, [])

  // Periodically check backend health
  useEffect(() => {
    const healthCheck = setInterval(async () => {
      try {
        const isHealthy = await apiClient.checkHealth()
        
        if (isHealthy && mockMode) {
          // Backend came back online
          setBackendStatus('online')
          setMockMode(false)
          mockDetectionGenerator.stop()
          wsClient.connect()
        } else if (!isHealthy && !mockMode) {
          // Backend went offline
          setBackendStatus('offline')
          setMockMode(true)
          wsClient.disconnect()
          mockDetectionGenerator.start((detections) => {
            addDetections(detections)
          })
        }
      } catch (error) {
        // Health check failed, continue with current mode
      }
    }, 10000) // Check every 10 seconds

    return () => clearInterval(healthCheck)
  }, [mockMode])

  return (
    <BrowserRouter>
      <div className="min-h-screen bg-gray-100">
        <Header />
        <div className="flex">
          <Sidebar />
          <main className="flex-1 min-h-screen">
            <Routes>
              <Route path="/" element={<Dashboard />} />
              <Route path="/settings" element={<Settings />} />
            </Routes>
          </main>
        </div>
      </div>
    </BrowserRouter>
  )
}

export default App
