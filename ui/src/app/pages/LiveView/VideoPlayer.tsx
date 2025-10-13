import { useEffect, useRef, useState } from 'react'
import { useAppStore } from '@/app/store'
import { Card } from '@/app/components/common/Card'
import { Spinner } from '@/app/components/common/Spinner'
import { AlertCircle, Maximize, Minimize } from 'lucide-react'
import type { Camera } from '@/app/services/schema'

interface VideoPlayerProps {
  camera: Camera
  showOverlay: boolean
  showGrid: boolean
  showBoxes: boolean
  showCentroids: boolean
  confidenceThreshold: number
}

export function VideoPlayer({
  camera,
  showOverlay,
  showGrid,
  showBoxes,
  showCentroids,
  confidenceThreshold,
}: VideoPlayerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const videoRef = useRef<HTMLVideoElement>(null)
  const [isLoading, setIsLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [isFullscreen, setIsFullscreen] = useState(false)
  
  const liveEvents = useAppStore((state) => state.liveEvents)
  const streamProtocol = import.meta.env.VITE_STREAM_PROTOCOL || 'mjpeg'

  useEffect(() => {
    setIsLoading(true)
    setError(null)

    // Simulate stream loading
    const timer = setTimeout(() => {
      setIsLoading(false)
    }, 1000)

    return () => clearTimeout(timer)
  }, [camera.id])

  useEffect(() => {
    if (!canvasRef.current || !videoRef.current || !showOverlay) return

    const canvas = canvasRef.current
    const video = videoRef.current
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Match canvas size to video
    canvas.width = video.videoWidth || 1920
    canvas.height = video.videoHeight || 1080

    const drawOverlay = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height)

      // Draw grid
      if (showGrid) {
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)'
        ctx.lineWidth = 1
        const gridSize = 50
        for (let x = 0; x < canvas.width; x += gridSize) {
          ctx.beginPath()
          ctx.moveTo(x, 0)
          ctx.lineTo(x, canvas.height)
          ctx.stroke()
        }
        for (let y = 0; y < canvas.height; y += gridSize) {
          ctx.beginPath()
          ctx.moveTo(0, y)
          ctx.lineTo(canvas.width, y)
          ctx.stroke()
        }
      }

      // Draw detections from live events
      const recentEvents = liveEvents
        .filter((e) => e.cameraId === camera.id)
        .filter((e) => e.confidence * 100 >= confidenceThreshold)
        .slice(-10) // Last 10 events

      recentEvents.forEach((event) => {
        if (!event.bbox) return

        const { x, y, width, height } = event.bbox
        const confidence = Math.round(event.confidence * 100)

        // Draw bounding box
        if (showBoxes) {
          ctx.strokeStyle = getColorForClass(event.class)
          ctx.lineWidth = 3
          ctx.strokeRect(x, y, width, height)

          // Draw label
          const label = `${event.class} ${confidence}%`
          ctx.fillStyle = getColorForClass(event.class)
          ctx.fillRect(x, y - 25, ctx.measureText(label).width + 10, 25)
          ctx.fillStyle = 'white'
          ctx.font = '14px sans-serif'
          ctx.fillText(label, x + 5, y - 7)
        }

        // Draw centroid
        if (showCentroids) {
          const cx = x + width / 2
          const cy = y + height / 2
          ctx.fillStyle = getColorForClass(event.class)
          ctx.beginPath()
          ctx.arc(cx, cy, 5, 0, 2 * Math.PI)
          ctx.fill()
          ctx.strokeStyle = 'white'
          ctx.lineWidth = 2
          ctx.stroke()
        }
      })

      requestAnimationFrame(drawOverlay)
    }

    const animationId = requestAnimationFrame(drawOverlay)
    return () => cancelAnimationFrame(animationId)
  }, [
    camera.id,
    showOverlay,
    showGrid,
    showBoxes,
    showCentroids,
    confidenceThreshold,
    liveEvents,
  ])

  const getColorForClass = (className: string): string => {
    const colors: Record<string, string> = {
      bird: '#3b82f6', // blue
      bat: '#8b5cf6', // purple
      insect: '#10b981', // green
      uap: '#ef4444', // red
      unknown: '#6b7280', // gray
    }
    return colors[className.toLowerCase()] || colors.unknown
  }

  const toggleFullscreen = () => {
    if (!document.fullscreenElement) {
      canvasRef.current?.parentElement?.requestFullscreen()
      setIsFullscreen(true)
    } else {
      document.exitFullscreen()
      setIsFullscreen(false)
    }
  }

  const getStreamUrl = (): string => {
    // In production, this would be the actual stream URL
    // For now, return a placeholder or the camera URL
    if (streamProtocol === 'hls') {
      return `/api/cameras/${camera.id}/stream.m3u8`
    }
    if (streamProtocol === 'mjpeg') {
      return `/api/cameras/${camera.id}/stream.mjpeg`
    }
    return camera.url || ''
  }

  return (
    <Card>
      <div className="relative aspect-video bg-gray-900">
        {isLoading && (
          <div className="absolute inset-0 flex items-center justify-center">
            <Spinner size="lg" />
          </div>
        )}

        {error && (
          <div className="absolute inset-0 flex items-center justify-center">
            <div className="text-center">
              <AlertCircle className="mx-auto h-12 w-12 text-red-500" />
              <p className="mt-2 text-sm text-white">{error}</p>
            </div>
          </div>
        )}

        {/* Video element (hidden, used for HLS) */}
        <video
          ref={videoRef}
          className="absolute inset-0 h-full w-full object-contain"
          style={{ display: streamProtocol === 'hls' ? 'block' : 'none' }}
          autoPlay
          muted
          playsInline
        />

        {/* MJPEG Image */}
        {streamProtocol === 'mjpeg' && !isLoading && !error && (
          <img
            src={getStreamUrl()}
            alt={`${camera.name} stream`}
            className="absolute inset-0 h-full w-full object-contain"
            onError={() => setError('Failed to load stream')}
          />
        )}

        {/* Overlay Canvas */}
        {showOverlay && (
          <canvas
            ref={canvasRef}
            className="absolute inset-0 h-full w-full object-contain"
          />
        )}

        {/* Controls Overlay */}
        <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-black/60 to-transparent p-4">
          <div className="flex items-center justify-between text-white">
            <div className="space-y-1">
              <p className="text-sm font-medium">{camera.name}</p>
            </div>
            <button
              onClick={toggleFullscreen}
              className="rounded-lg bg-white/20 p-2 backdrop-blur-sm hover:bg-white/30"
              aria-label={isFullscreen ? 'Exit fullscreen' : 'Enter fullscreen'}
            >
              {isFullscreen ? (
                <Minimize className="h-5 w-5" />
              ) : (
                <Maximize className="h-5 w-5" />
              )}
            </button>
          </div>
        </div>

        {/* Stream Info */}
        <div className="absolute right-4 top-4 rounded-lg bg-black/60 px-3 py-1 text-xs text-white backdrop-blur-sm">
          {streamProtocol.toUpperCase()}
        </div>
      </div>
    </Card>
  )
}