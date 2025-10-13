import { z } from 'zod'

// System Health Schema
export const SystemHealthSchema = z.object({
  status: z.enum(['ok', 'degraded', 'error']),
  cpuLoad: z.number().min(0).max(100),
  gpuLoad: z.number().min(0).max(100).optional(),
  memory: z.object({
    used: z.number(),
    total: z.number(),
    percent: z.number().min(0).max(100),
  }),
  disk: z.object({
    used: z.number(),
    total: z.number(),
    percent: z.number().min(0).max(100),
  }),
  uptime: z.number(),
  versions: z.object({
    ui: z.string(),
    backend: z.string(),
    ros2: z.string().optional(),
  }),
  temperature: z.number().optional(),
})

export type SystemHealth = z.infer<typeof SystemHealthSchema>

// Camera Schema
export const CameraSchema = z.object({
  id: z.string(),
  name: z.string(),
  enabled: z.boolean(),
  url: z.string().optional(),
  protocol: z.enum(['rtsp', 'onvif', 'usb', 'file']),
  status: z.enum(['online', 'offline', 'error', 'connecting']),
  lastSeen: z.string().datetime().optional(),
  resolution: z
    .object({
      width: z.number(),
      height: z.number(),
    })
    .optional(),
  fps: z.number().optional(),
  username: z.string().optional(),
  password: z.string().optional(),
})

export type Camera = z.infer<typeof CameraSchema>

// Detection/Track Schema
export const DetectionSchema = z.object({
  id: z.string(),
  trackId: z.string().optional(),
  cameraId: z.string(),
  timestamp: z.string().datetime(),
  bbox: z.object({
    x: z.number(),
    y: z.number(),
    width: z.number(),
    height: z.number(),
  }),
  class: z.string(),
  confidence: z.number().min(0).max(1),
  snapshotUrl: z.string().optional(),
  metadata: z.record(z.any()).optional(),
})

export type Detection = z.infer<typeof DetectionSchema>

// Track Schema (aggregated detections)
export const TrackSchema = z.object({
  id: z.string(),
  cameraId: z.string(),
  class: z.string(),
  firstSeen: z.string().datetime(),
  lastSeen: z.string().datetime(),
  detectionCount: z.number(),
  avgConfidence: z.number().min(0).max(1),
  thumbnailUrl: z.string().optional(),
  status: z.enum(['active', 'lost', 'completed']),
})

export type Track = z.infer<typeof TrackSchema>

// Recording Schema
export const RecordingSchema = z.object({
  id: z.string(),
  cameraId: z.string(),
  startTime: z.string().datetime(),
  endTime: z.string().datetime(),
  duration: z.number(),
  fileSize: z.number(),
  url: z.string(),
  filename: z.string().optional(),
  thumbnailUrl: z.string().optional(),
  detectionCount: z.number(),
  bookmarked: z.boolean().optional(),
})

export type Recording = z.infer<typeof RecordingSchema>

// Configuration Schema
export const ConfigSchema = z.object({
  detection: z.object({
    enabled: z.boolean(),
    confidence: z.number().min(0).max(1),
    nms: z.number().min(0).max(1),
    classes: z.array(z.string()),
  }),
  tracking: z.object({
    enabled: z.boolean(),
    maxAge: z.number(),
    minHits: z.number(),
    iouThreshold: z.number().min(0).max(1),
  }),
  storage: z.object({
    enabled: z.boolean(),
    path: z.string(),
    maxSize: z.number(),
    retention: z.number(),
  }),
  cameras: z.array(
    z.object({
      id: z.string(),
      profile: z.string(),
      settings: z.record(z.any()),
    })
  ),
  network: z.object({
    apiPort: z.number(),
    wsPort: z.number(),
    streamPort: z.number(),
  }),
})

export type Config = z.infer<typeof ConfigSchema>

// Metrics Schema
export const MetricsSchema = z.object({
  timestamp: z.string().datetime(),
  cpu: z.number().min(0).max(100),
  gpu: z.number().min(0).max(100).optional(),
  memory: z.number().min(0).max(100),
  disk: z.number().min(0).max(100),
  temperature: z.number().optional(),
  fps: z.number().optional(),
})

export type Metrics = z.infer<typeof MetricsSchema>

// Log Entry Schema
export const LogEntrySchema = z.object({
  timestamp: z.string().datetime(),
  level: z.enum(['debug', 'info', 'warn', 'error']),
  message: z.string(),
  source: z.string().optional(),
  metadata: z.record(z.any()).optional(),
})

export type LogEntry = z.infer<typeof LogEntrySchema>

// WebSocket Event Schemas
export const WSTelemetrySchema = z.object({
  type: z.literal('telemetry'),
  data: MetricsSchema,
})

export const WSDetectionSchema = z.object({
  type: z.literal('detection'),
  data: DetectionSchema,
})

export const WSLogSchema = z.object({
  type: z.literal('log'),
  data: LogEntrySchema,
})

export const WSEventSchema = z.discriminatedUnion('type', [
  WSTelemetrySchema,
  WSDetectionSchema,
  WSLogSchema,
])

export type WSEvent = z.infer<typeof WSEventSchema>

// Paginated Response Schema
export const PaginatedResponseSchema = <T extends z.ZodTypeAny>(itemSchema: T) =>
  z.object({
    items: z.array(itemSchema),
    total: z.number(),
    page: z.number(),
    pageSize: z.number(),
    hasMore: z.boolean(),
  })

export type PaginatedResponse<T> = {
  items: T[]
  total: number
  page: number
  pageSize: number
  hasMore: boolean
}