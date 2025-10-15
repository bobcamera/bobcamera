/**
 * ROS2 WebSocket Client using roslib.js
 * Connects to rosbridge_suite for ROS2 communication
 */

// ROSLIB is loaded from CDN in index.html
declare const ROSLIB: any

export interface ROS2Config {
  url: string
  reconnectInterval?: number
  maxReconnectAttempts?: number
}

export interface CompressedImage {
  format: string
  data: string // base64 encoded
}

export interface MonitoringStatus {
  sensitivity: number
  alive: number
  trackable: number
  started: number
  ended: number
  recording: boolean
  day_night_enum: number // 0=neutral, 1=day, 2=night
  percentage_cloud_cover: number
}

export interface BoundingBox2D {
  center: { x: number; y: number }
  size_x: number
  size_y: number
}

export interface Detection2D {
  bbox: BoundingBox2D
  results: Array<{
    hypothesis: {
      class_id: string
      score: number
    }
  }>
}

export interface BoundingBox2DArray {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  boxes: Detection2D[]
}

export class ROS2Client {
  private ros: ROSLIB.Ros | null = null
  private config: Required<ROS2Config>
  private reconnectTimer: NodeJS.Timeout | null = null
  private reconnectAttempts = 0
  private isConnecting = false

  // Event handlers
  public onConnected?: () => void
  public onDisconnected?: () => void
  public onError?: (error: Error) => void

  constructor(config: ROS2Config) {
    this.config = {
      url: config.url,
      reconnectInterval: config.reconnectInterval ?? 3000,
      maxReconnectAttempts: config.maxReconnectAttempts ?? 10,
    }
  }

  /**
   * Connect to ROS2 via rosbridge WebSocket
   */
  connect(): void {
    if (this.isConnecting || this.ros?.isConnected) {
      console.log('[ROS2Client] Already connected or connecting')
      return
    }

    this.isConnecting = true
    console.log(`[ROS2Client] Connecting to ${this.config.url}...`)

    try {
      this.ros = new ROSLIB.Ros({
        url: this.config.url,
      })

      this.ros!.on('connection', () => {
        console.log('[ROS2Client] Connected to rosbridge')
        this.isConnecting = false
        this.reconnectAttempts = 0
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer)
          this.reconnectTimer = null
        }
        this.onConnected?.()
      })

      this.ros!.on('error', (error: any) => {
        console.error('[ROS2Client] Connection error:', error)
        this.isConnecting = false
        this.onError?.(new Error(String(error)))
      })

      this.ros!.on('close', () => {
        console.log('[ROS2Client] Connection closed')
        this.isConnecting = false
        this.onDisconnected?.()
        this.scheduleReconnect()
      })
    } catch (error) {
      console.error('[ROS2Client] Failed to create ROS connection:', error)
      this.isConnecting = false
      this.onError?.(error as Error)
      this.scheduleReconnect()
    }
  }

  /**
   * Disconnect from ROS2
   */
  disconnect(): void {
    console.log('[ROS2Client] Disconnecting...')
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    this.reconnectAttempts = 0
    this.ros?.close()
    this.ros = null
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.ros?.isConnected ?? false
  }

  /**
   * Schedule reconnection attempt
   */
  private scheduleReconnect(): void {
    if (this.reconnectAttempts >= this.config.maxReconnectAttempts) {
      console.error('[ROS2Client] Max reconnection attempts reached')
      this.onError?.(new Error('Max reconnection attempts reached'))
      return
    }

    if (this.reconnectTimer) {
      return // Already scheduled
    }

    this.reconnectAttempts++
    console.log(
      `[ROS2Client] Scheduling reconnect attempt ${this.reconnectAttempts}/${this.config.maxReconnectAttempts} in ${this.config.reconnectInterval}ms`
    )

    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null
      this.connect()
    }, this.config.reconnectInterval)
  }

  /**
   * Subscribe to compressed image topic
   */
  subscribeToCompressedImage(
    topicName: string,
    callback: (message: CompressedImage) => void
  ): () => void {
    if (!this.ros) {
      throw new Error('ROS2 client not connected')
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: 'sensor_msgs/msg/CompressedImage',
    })

    topic.subscribe((message: any) => {
      callback({
        format: message.format,
        data: message.data,
      })
    })

    console.log(`[ROS2Client] Subscribed to ${topicName}`)

    // Return unsubscribe function
    return () => {
      topic.unsubscribe()
      console.log(`[ROS2Client] Unsubscribed from ${topicName}`)
    }
  }

  /**
   * Subscribe to monitoring status topic
   */
  subscribeToMonitoringStatus(
    topicName: string,
    callback: (status: MonitoringStatus) => void
  ): () => void {
    if (!this.ros) {
      throw new Error('ROS2 client not connected')
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: 'bob_interfaces/msg/MonitoringStatus',
    })

    topic.subscribe((message: any) => {
      callback({
        sensitivity: message.sensitivity,
        alive: message.alive,
        trackable: message.trackable,
        started: message.started,
        ended: message.ended,
        recording: message.recording,
        day_night_enum: message.day_night_enum,
        percentage_cloud_cover: message.percentage_cloud_cover,
      })
    })

    console.log(`[ROS2Client] Subscribed to ${topicName}`)

    return () => {
      topic.unsubscribe()
      console.log(`[ROS2Client] Unsubscribed from ${topicName}`)
    }
  }

  /**
   * Subscribe to bounding box array topic
   */
  subscribeToBoundingBoxes(
    topicName: string,
    callback: (boxes: BoundingBox2DArray) => void
  ): () => void {
    if (!this.ros) {
      throw new Error('ROS2 client not connected')
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: 'vision_msgs/msg/Detection2DArray',
    })

    topic.subscribe((message: any) => {
      callback(message as BoundingBox2DArray)
    })

    console.log(`[ROS2Client] Subscribed to ${topicName}`)

    return () => {
      topic.unsubscribe()
      console.log(`[ROS2Client] Unsubscribed from ${topicName}`)
    }
  }

  /**
   * Call application info service
   */
  async getApplicationInfo(): Promise<{
    version: string
    frame_width: number
    frame_height: number
    video_fps: number
  }> {
    if (!this.ros) {
      throw new Error('ROS2 client not connected')
    }

    return new Promise((resolve, reject) => {
      const service = new ROSLIB.Service({
        ros: this.ros!,
        name: '/bob/webapi/application/info',
        serviceType: 'bob_interfaces/srv/ApplicationInfo',
      })

      const request = new ROSLIB.ServiceRequest({})

      service.callService(
        request,
        (result: any) => {
          resolve({
            version: result.version,
            frame_width: result.frame_width,
            frame_height: result.frame_height,
            video_fps: result.video_fps,
          })
        },
        (error: any) => {
          reject(new Error(`Service call failed: ${error}`))
        }
      )
    })
  }
}

// Singleton instance
let ros2ClientInstance: ROS2Client | null = null

/**
 * Get or create ROS2 client singleton
 */
export function getROS2Client(): ROS2Client {
  if (!ros2ClientInstance) {
    const url = import.meta.env.VITE_ROS2_WS_URL || 'ws://localhost:9090'
    ros2ClientInstance = new ROS2Client({ url })
  }
  return ros2ClientInstance
}

/**
 * Reset ROS2 client (useful for testing)
 */
export function resetROS2Client(): void {
  if (ros2ClientInstance) {
    ros2ClientInstance.disconnect()
    ros2ClientInstance = null
  }
}