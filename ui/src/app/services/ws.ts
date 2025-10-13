import { type WSEvent, WSEventSchema } from './schema'

const WS_BASE_URL = import.meta.env.VITE_WS_BASE_URL || 'ws://localhost:8080/ws'

type WSEventHandler = (event: WSEvent) => void
type WSErrorHandler = (error: Event) => void
type WSStatusHandler = (status: 'connecting' | 'connected' | 'disconnected' | 'error') => void

class WebSocketClient {
  private ws: WebSocket | null = null
  private reconnectAttempts = 0
  private maxReconnectAttempts = 10
  private reconnectDelay = 1000
  private maxReconnectDelay = 30000
  private reconnectTimer: NodeJS.Timeout | null = null
  private isManualClose = false

  private eventHandlers: Set<WSEventHandler> = new Set()
  private errorHandlers: Set<WSErrorHandler> = new Set()
  private statusHandlers: Set<WSStatusHandler> = new Set()

  constructor() {}

  connect(endpoint: string = '/events'): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      console.warn('WebSocket already connected')
      return
    }

    this.isManualClose = false
    this.updateStatus('connecting')

    const url = `${WS_BASE_URL}${endpoint}`
    console.log('Connecting to WebSocket:', url)

    try {
      this.ws = new WebSocket(url)

      this.ws.onopen = () => {
        console.log('WebSocket connected')
        this.reconnectAttempts = 0
        this.reconnectDelay = 1000
        this.updateStatus('connected')
      }

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data)
          const parsedEvent = WSEventSchema.parse(data)
          this.eventHandlers.forEach((handler) => handler(parsedEvent))
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error)
        }
      }

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error)
        this.updateStatus('error')
        this.errorHandlers.forEach((handler) => handler(error))
      }

      this.ws.onclose = () => {
        console.log('WebSocket closed')
        this.updateStatus('disconnected')
        this.ws = null

        if (!this.isManualClose) {
          this.scheduleReconnect(endpoint)
        }
      }
    } catch (error) {
      console.error('Failed to create WebSocket:', error)
      this.updateStatus('error')
      this.scheduleReconnect(endpoint)
    }
  }

  disconnect(): void {
    this.isManualClose = true
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
    this.updateStatus('disconnected')
  }

  send(data: any): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data))
    } else {
      console.warn('WebSocket not connected, cannot send data')
    }
  }

  onEvent(handler: WSEventHandler): () => void {
    this.eventHandlers.add(handler)
    return () => this.eventHandlers.delete(handler)
  }

  onError(handler: WSErrorHandler): () => void {
    this.errorHandlers.add(handler)
    return () => this.errorHandlers.delete(handler)
  }

  onStatus(handler: WSStatusHandler): () => void {
    this.statusHandlers.add(handler)
    return () => this.statusHandlers.delete(handler)
  }

  private scheduleReconnect(endpoint: string): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('Max reconnection attempts reached')
      this.updateStatus('error')
      return
    }

    this.reconnectAttempts++
    const delay = Math.min(
      this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1),
      this.maxReconnectDelay
    )

    console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`)

    this.reconnectTimer = setTimeout(() => {
      this.connect(endpoint)
    }, delay)
  }

  private updateStatus(status: 'connecting' | 'connected' | 'disconnected' | 'error'): void {
    this.statusHandlers.forEach((handler) => handler(status))
  }

  getStatus(): 'connecting' | 'connected' | 'disconnected' | 'error' {
    if (!this.ws) return 'disconnected'
    switch (this.ws.readyState) {
      case WebSocket.CONNECTING:
        return 'connecting'
      case WebSocket.OPEN:
        return 'connected'
      case WebSocket.CLOSING:
      case WebSocket.CLOSED:
        return 'disconnected'
      default:
        return 'error'
    }
  }
}

// Create separate clients for different endpoints
export const telemetryClient = new WebSocketClient()
export const eventsClient = new WebSocketClient()
export const logsClient = new WebSocketClient()

// Legacy API wrapper for backward compatibility
export const wsClient = {
  connect: (endpoint: string) => {
    if (endpoint === 'logs') {
      logsClient.connect('/logs')
    } else {
      eventsClient.connect(`/${endpoint}`)
    }
  },
  disconnect: (endpoint: string) => {
    if (endpoint === 'logs') {
      logsClient.disconnect()
    } else {
      eventsClient.disconnect()
    }
  },
  on: (endpoint: string, eventType: string, handler: (data: any) => void) => {
    const client = endpoint === 'logs' ? logsClient : eventsClient
    return client.onEvent((event) => {
      if (event.type === eventType) {
        handler(event.data)
      }
    })
  },
  off: (_endpoint: string, _eventType: string, _handler: (data: any) => void) => {
    // Handler cleanup is done via the unsubscribe function returned by on()
    // This is a no-op for compatibility
  },
}

// Export for testing
export { WebSocketClient }