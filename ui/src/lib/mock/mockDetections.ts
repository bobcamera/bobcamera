import type { Detection } from '../../store/useAppStore'

const birdClasses = [
  'Robin', 'Sparrow', 'Crow', 'Blue Jay', 'Cardinal', 
  'Finch', 'Pigeon', 'Hawk', 'Eagle', 'Owl'
]

const otherClasses = ['Plane', 'Cloud', 'Unknown', 'Edge']

export function generateMockDetections(count: number = 5): Detection[] {
  const detections: Detection[] = []
  const timestamp = Date.now()
  
  for (let i = 0; i < count; i++) {
    // Random position within a 1920x1080 frame
    const x = Math.random() * 1600 // Leave some margin
    const y = Math.random() * 800
    const width = 50 + Math.random() * 150
    const height = 50 + Math.random() * 150
    
    // Mostly birds, some other objects
    const isBird = Math.random() > 0.3
    const className = isBird 
      ? birdClasses[Math.floor(Math.random() * birdClasses.length)]
      : otherClasses[Math.floor(Math.random() * otherClasses.length)]
    
    // Higher confidence for birds
    const baseConfidence = isBird ? 0.7 : 0.4
    const confidence = baseConfidence + Math.random() * 0.3
    
    detections.push({
      id: `mock-${timestamp}-${i}`,
      x: Math.round(x),
      y: Math.round(y),
      width: Math.round(width),
      height: Math.round(height),
      confidence: Math.round(confidence * 100) / 100,
      class: className,
      timestamp: timestamp + i
    })
  }
  
  return detections
}

export class MockDetectionGenerator {
  private interval: ReturnType<typeof setInterval> | undefined = undefined
  private callback: ((detections: Detection[]) => void) | null = null
  
  start(onDetections: (detections: Detection[]) => void, intervalMs: number = 500) {
    this.callback = onDetections
    
    this.interval = setInterval(() => {
      if (this.callback) {
        // Generate 0-8 detections per frame
        const count = Math.floor(Math.random() * 9)
        const detections = generateMockDetections(count)
        this.callback(detections)
      }
    }, intervalMs)
  }
  
  stop() {
    if (this.interval) {
      clearInterval(this.interval)
      this.interval = undefined
    }
    this.callback = null
  }
  
  isRunning(): boolean {
    return this.interval !== undefined
  }
}

export const mockDetectionGenerator = new MockDetectionGenerator()