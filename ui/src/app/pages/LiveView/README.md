# LiveView Page - ROS2 Integration

## Overview

The LiveView page provides real-time video streaming from BOB cameras using ROS2 WebSocket communication via rosbridge_suite.

## Architecture

### ROS2 Communication

- **Library**: `roslib.js` - JavaScript client for ROS2 via rosbridge
- **Protocol**: WebSocket connection to rosbridge_suite
- **Default URL**: `ws://localhost:9090`

### Video Streaming

BOB uses **compressed JPEG frames** streamed via ROS2 topics:

```
Topic: /bob/camera{N}/annotated/resized/compressed
Type: sensor_msgs/msg/CompressedImage
Format: JPEG with base64 encoding
```

The frames are **pre-annotated** with bounding boxes by the backend, so the UI receives ready-to-display images.

### Monitoring Status

Real-time tracking and recording status:

```
Topic: /bob/camera{N}/monitoring/status
Type: bob_interfaces/msg/MonitoringStatus
Fields:
  - sensitivity: number
  - alive: number (active tracked objects)
  - trackable: number
  - started: number
  - ended: number
  - recording: boolean
  - day_night_enum: 0=neutral, 1=day, 2=night
  - percentage_cloud_cover: number
```

## Components

### ROS2Client (`services/ros2Client.ts`)

Singleton WebSocket client for ROS2 communication:

```typescript
import { getROS2Client } from '@/app/services/ros2Client'

const ros2Client = getROS2Client()

// Connect
ros2Client.connect()

// Subscribe to compressed images
const unsubscribe = ros2Client.subscribeToCompressedImage(
  '/bob/camera1/annotated/resized/compressed',
  (message) => {
    // message.data contains base64 JPEG
    imageElement.src = `data:image/jpeg;base64,${message.data}`
  }
)

// Subscribe to monitoring status
ros2Client.subscribeToMonitoringStatus(
  '/bob/camera1/monitoring/status',
  (status) => {
    console.log('Tracking:', status.alive, 'objects')
  }
)

// Cleanup
unsubscribe()
```

**Features:**
- Auto-reconnect with exponential backoff
- Connection state management
- Multiple topic subscriptions
- Service call support
- TypeScript typed messages

### VideoPlayer Component

Displays live video stream with overlays:

**Props:**
- `camera` - Camera configuration
- `showOverlay` - Enable/disable overlay canvas
- `showGrid` - Show alignment grid
- `showBoxes` - Show bounding boxes (from WebSocket events)
- `showCentroids` - Show object centroids
- `confidenceThreshold` - Filter detections by confidence

**Features:**
- Real-time JPEG frame display
- FPS calculation and display
- Monitoring status overlay
- Canvas-based custom overlays
- Fullscreen support
- Error handling and loading states

### LiveView Page

Main page component:

**Features:**
- Camera selector (tabs for enabled cameras)
- Overlay controls (toggle boxes, grid, centroids)
- Confidence threshold slider
- Camera info panel
- Auto-select first enabled camera

## Configuration

### Environment Variables

```bash
# .env
VITE_ROS2_WS_URL=ws://localhost:9090
VITE_STREAM_PROTOCOL=ros2
```

### Camera ID Mapping

BOB uses numeric camera IDs in topic names:
- Camera ID `1` → `/bob/camera1/...`
- Camera ID `2` → `/bob/camera2/...`

The UI camera IDs must match the ROS2 topic naming.

## Usage

### Development

1. **Start BOB backend** with rosbridge:
   ```bash
   cd ~/bobcamera
   ./run.sh my_config.yaml
   ```

2. **Start UI dev server**:
   ```bash
   cd ui
   npm run dev
   ```

3. **Navigate to LiveView**:
   ```
   http://localhost:5173/live
   ```

### Production

The UI will connect to rosbridge at the configured URL. Ensure:
- rosbridge_suite is running on port 9090
- WebSocket connections are allowed
- Camera topics are publishing

## Troubleshooting

### No Video Stream

**Check:**
1. Is rosbridge running? `ros2 run rosbridge_server rosbridge_websocket`
2. Are camera topics publishing? `ros2 topic list | grep camera`
3. Is the WebSocket URL correct in `.env`?
4. Check browser console for connection errors

**Test connection:**
```bash
# List all topics
ros2 topic list

# Echo camera topic
ros2 topic echo /bob/camera1/annotated/resized/compressed
```

### Connection Errors

**Common issues:**
- **CORS errors**: rosbridge needs proper CORS configuration
- **Port blocked**: Check firewall allows port 9090
- **Wrong URL**: Verify `VITE_ROS2_WS_URL` matches rosbridge address

### Performance Issues

**Optimize:**
- Reduce camera resolution in BOB config
- Lower FPS in BOB config
- Disable overlay rendering if not needed
- Use annotated stream (pre-rendered boxes) instead of custom overlays

## ROS2 Topics Reference

### Image Topics

```
/bob/camera{N}/annotated/resized/compressed
  - Pre-annotated frames with bounding boxes
  - Type: sensor_msgs/msg/CompressedImage
  - Format: JPEG, base64 encoded

/bob/camera{N}/raw/compressed
  - Raw camera frames without annotations
  - Type: sensor_msgs/msg/CompressedImage
```

### Status Topics

```
/bob/camera{N}/monitoring/status
  - Tracking and recording status
  - Type: bob_interfaces/msg/MonitoringStatus
  - Rate: ~1 Hz

/bob/detection/allsky/boundingboxes
  - Raw bounding box detections
  - Type: vision_msgs/msg/Detection2DArray
```

### Services

```
/bob/webapi/application/info
  - Get application version and config
  - Type: bob_interfaces/srv/ApplicationInfo
  - Returns: version, frame_width, frame_height, video_fps
```

## Future Enhancements

- [ ] Multi-camera grid view (2x2, 3x3)
- [ ] Custom bounding box rendering from Detection2DArray
- [ ] Recording controls (start/stop)
- [ ] Snapshot capture
- [ ] PTZ camera controls
- [ ] Audio streaming
- [ ] Latency measurement and display
- [ ] Bandwidth monitoring
- [ ] Stream quality selection

## References

- [roslib.js Documentation](http://robotwebtools.org/jsdoc/roslibjs/current/)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
- [ROS2 sensor_msgs](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [ROS2 vision_msgs](https://github.com/ros-perception/vision_msgs)