# LiveView Page - Testing Guide

## 🎯 Overview

The LiveView page has been successfully implemented with full ROS2 WebSocket integration for real-time video streaming from BOB cameras. This guide will help you test all features.

## ✅ What's Been Implemented

### Core Features
- ✅ Real-time JPEG frame streaming via ROS2 WebSocket (rosbridge)
- ✅ Multi-camera support with tab-based switching
- ✅ Live FPS counter and performance monitoring
- ✅ Monitoring status display (tracking/recording/object count)
- ✅ Canvas-based overlay system (grid, bounding boxes, centroids)
- ✅ Confidence threshold filtering
- ✅ Fullscreen mode
- ✅ Auto-reconnect on connection loss
- ✅ Error handling with user-friendly messages
- ✅ Proper resource cleanup (no memory leaks)

### Technical Implementation
- ✅ ROS2Client service with singleton pattern
- ✅ TypeScript declarations for roslib.js
- ✅ Subscription lifecycle management
- ✅ Base64 JPEG decoding and display
- ✅ Real-time status updates from monitoring topics
- ✅ Environment-based configuration

## 🧪 Testing Checklist

### Prerequisites
1. **BOB Backend Running**
   - Ensure BOB is running with rosbridge_suite enabled
   - Default rosbridge port: 9090
   - Check with: `docker ps` (look for rosbridge container)

2. **UI Dev Server Running**
   ```powershell
   cd c:\bobcamera\ui
   npm run dev
   ```
   - Should be running on http://localhost:5173

3. **Camera Configuration**
   - At least one camera configured in BOB
   - Camera should be streaming video
   - Check camera status in BOB's web2 interface (http://localhost:8080)

### Test Scenarios

#### 1. Basic Video Streaming
**Steps:**
1. Navigate to http://localhost:5173/live
2. Select a camera from the tabs
3. Verify video stream appears

**Expected Results:**
- ✅ Video frames display smoothly
- ✅ FPS counter shows ~10-30 FPS (depends on camera)
- ✅ No console errors
- ✅ Latency indicator shows reasonable values (<500ms)

**Troubleshooting:**
- ❌ "Connecting..." message persists → Check rosbridge is running on port 9090
- ❌ "Connection failed" → Verify `VITE_ROS2_WS_URL` in `.env` file
- ❌ Black screen → Check camera is streaming in BOB backend
- ❌ Console error "Failed to subscribe" → Verify topic names match BOB's configuration

#### 2. Camera Switching
**Steps:**
1. Click on different camera tabs
2. Observe video stream changes

**Expected Results:**
- ✅ Video switches to new camera immediately
- ✅ FPS counter resets and starts counting
- ✅ Monitoring status updates for new camera
- ✅ Previous camera subscription is cleaned up (check console)

**Troubleshooting:**
- ❌ Video doesn't switch → Check camera ID matches ROS2 topic naming
- ❌ Multiple streams overlay → Subscription cleanup issue (check console for errors)

#### 3. Monitoring Status Display
**Steps:**
1. Observe the status indicators at top of video
2. Trigger tracking/recording in BOB backend
3. Watch status indicators update

**Expected Results:**
- ✅ Tracking status shows "Active" or "Idle"
- ✅ Recording status shows "On" or "Off"
- ✅ Object count updates in real-time
- ✅ Status colors change appropriately (green=active, gray=idle)

**Troubleshooting:**
- ❌ Status shows "Unknown" → Monitoring topic not publishing
- ❌ Status doesn't update → Check `/bob/camera{N}/monitoring/status` topic

#### 4. Overlay Controls
**Steps:**
1. Toggle "Show Grid" checkbox
2. Toggle "Show Boxes" checkbox
3. Toggle "Show Centroids" checkbox
4. Adjust confidence threshold slider

**Expected Results:**
- ✅ Grid overlay appears/disappears
- ✅ Bounding boxes appear/disappears (if available)
- ✅ Centroids appear/disappears (if available)
- ✅ Threshold slider filters detections by confidence

**Note:** Bounding boxes and centroids require the raw detection topic (`/bob/detection/allsky/boundingboxes`). If using pre-annotated streams, boxes are already drawn on the video.

#### 5. Fullscreen Mode
**Steps:**
1. Click fullscreen button (expand icon)
2. Press ESC to exit fullscreen

**Expected Results:**
- ✅ Video expands to fullscreen
- ✅ Controls remain accessible
- ✅ ESC key exits fullscreen
- ✅ Video continues streaming smoothly

#### 6. Connection Resilience
**Steps:**
1. Start video streaming
2. Stop rosbridge container: `docker stop <rosbridge-container>`
3. Wait 5 seconds
4. Restart rosbridge: `docker start <rosbridge-container>`

**Expected Results:**
- ✅ "Connection lost" message appears
- ✅ "Reconnecting..." message shows
- ✅ Connection automatically restores
- ✅ Video streaming resumes
- ✅ No manual refresh needed

**Troubleshooting:**
- ❌ Doesn't reconnect → Check auto-reconnect logic in ROS2Client
- ❌ Multiple reconnection attempts → Check exponential backoff is working

#### 7. Performance Testing
**Steps:**
1. Open browser DevTools (F12)
2. Go to Performance tab
3. Start recording
4. Let video stream for 30 seconds
5. Stop recording and analyze

**Expected Results:**
- ✅ Memory usage stable (no memory leaks)
- ✅ CPU usage reasonable (<30% for video decoding)
- ✅ No dropped frames in timeline
- ✅ FPS counter matches actual frame rate

**Troubleshooting:**
- ❌ Memory increases continuously → Subscription not cleaning up
- ❌ High CPU usage → Check image decoding performance
- ❌ Dropped frames → Network latency or backend performance issue

#### 8. Multi-Tab Testing
**Steps:**
1. Open LiveView in one tab
2. Open LiveView in another tab
3. Switch cameras in both tabs

**Expected Results:**
- ✅ Both tabs stream independently
- ✅ No interference between tabs
- ✅ Each tab maintains its own connection
- ✅ Closing one tab doesn't affect the other

**Troubleshooting:**
- ❌ Tabs interfere → Singleton pattern issue in ROS2Client
- ❌ Connection drops → Check WebSocket connection limits

#### 9. Error Handling
**Steps:**
1. Navigate to LiveView with rosbridge stopped
2. Select a non-existent camera
3. Disconnect network temporarily

**Expected Results:**
- ✅ Clear error messages displayed
- ✅ No cryptic console errors
- ✅ UI remains responsive
- ✅ Retry options available

#### 10. Mobile Responsiveness
**Steps:**
1. Open DevTools (F12)
2. Toggle device toolbar (Ctrl+Shift+M)
3. Test on different screen sizes

**Expected Results:**
- ✅ Video scales appropriately
- ✅ Controls remain accessible
- ✅ Tabs work on mobile
- ✅ Touch gestures work (if applicable)

## 🔍 Console Debugging

### Expected Console Messages
```
[ROS2Client] Connecting to ws://localhost:9090...
[ROS2Client] Connected successfully
[ROS2Client] Subscribed to /bob/camera1/annotated/resized/compressed
[ROS2Client] Subscribed to /bob/camera1/monitoring/status
```

### Warning Messages (Non-Critical)
```
[ROS2Client] Reconnecting... (attempt 1/10)
[VideoPlayer] Waiting for first frame...
```

### Error Messages (Critical)
```
[ROS2Client] Connection failed: WebSocket error
[ROS2Client] Max reconnection attempts reached
[VideoPlayer] Failed to decode image frame
```

## 📊 Performance Benchmarks

### Expected Performance
- **FPS:** 10-30 FPS (depends on camera and backend)
- **Latency:** 100-500ms (network + processing)
- **Memory:** ~50-100MB per video stream
- **CPU:** 10-30% (video decoding)

### Performance Issues
- **Low FPS (<10):** Backend performance issue or network congestion
- **High Latency (>1s):** Network issues or backend overload
- **Memory Leak:** Subscription not cleaning up properly
- **High CPU (>50%):** Image decoding inefficiency

## 🐛 Common Issues & Solutions

### Issue: "Cannot connect to ROS2 WebSocket"
**Solution:**
1. Check rosbridge is running: `docker ps | grep rosbridge`
2. Verify port 9090 is accessible: `Test-NetConnection localhost -Port 9090`
3. Check `.env` file has correct `VITE_ROS2_WS_URL`
4. Restart dev server after changing `.env`

### Issue: "Video stream not displaying"
**Solution:**
1. Check camera is streaming in BOB backend
2. Verify topic name matches: `/bob/camera{N}/annotated/resized/compressed`
3. Check browser console for subscription errors
4. Verify camera ID in UI matches backend camera ID

### Issue: "FPS counter shows 0"
**Solution:**
1. Check frames are being received (console logs)
2. Verify image decoding is working (no console errors)
3. Check network tab for WebSocket messages
4. Ensure camera is actively streaming (not paused)

### Issue: "Monitoring status shows 'Unknown'"
**Solution:**
1. Check monitoring topic is publishing: `/bob/camera{N}/monitoring/status`
2. Verify topic message format matches expected schema
3. Check console for subscription errors
4. Ensure backend is publishing monitoring data

### Issue: "Memory leak / browser slows down"
**Solution:**
1. Check subscriptions are being unsubscribed on unmount
2. Verify canvas cleanup is happening
3. Check for event listener leaks
4. Use browser DevTools Memory profiler to identify leaks

## 🎨 Visual Testing

### Expected UI Elements
- ✅ Camera tabs at top
- ✅ Video player in center
- ✅ FPS counter (top-left)
- ✅ Latency indicator (top-right)
- ✅ Monitoring status badges (top-center)
- ✅ Control panel (right side)
- ✅ Fullscreen button (bottom-right)

### Visual Issues
- ❌ Video stretched/distorted → Check aspect ratio calculation
- ❌ Overlays misaligned → Check canvas coordinate mapping
- ❌ UI elements overlapping → Check z-index and positioning
- ❌ Text unreadable → Check contrast and font sizes

## 📝 Test Report Template

```markdown
## LiveView Testing Report

**Date:** YYYY-MM-DD
**Tester:** Your Name
**Environment:** Development / Staging / Production

### Test Results
- [ ] Basic Video Streaming: PASS / FAIL
- [ ] Camera Switching: PASS / FAIL
- [ ] Monitoring Status: PASS / FAIL
- [ ] Overlay Controls: PASS / FAIL
- [ ] Fullscreen Mode: PASS / FAIL
- [ ] Connection Resilience: PASS / FAIL
- [ ] Performance: PASS / FAIL
- [ ] Multi-Tab: PASS / FAIL
- [ ] Error Handling: PASS / FAIL
- [ ] Mobile Responsiveness: PASS / FAIL

### Issues Found
1. [Issue description]
   - Severity: Critical / High / Medium / Low
   - Steps to reproduce: ...
   - Expected: ...
   - Actual: ...

### Performance Metrics
- FPS: XX FPS
- Latency: XXX ms
- Memory: XX MB
- CPU: XX%

### Notes
[Any additional observations]
```

## 🚀 Next Steps After Testing

1. **If all tests pass:**
   - Mark LiveView as production-ready
   - Update IMPLEMENTATION_STATUS.md
   - Move to next page (Tracks)

2. **If issues found:**
   - Document issues in GitHub Issues
   - Prioritize by severity
   - Fix critical issues before moving on

3. **Performance optimization:**
   - Profile with Chrome DevTools
   - Optimize image decoding if needed
   - Consider WebWorkers for heavy processing

4. **User feedback:**
   - Gather feedback from actual users
   - Iterate on UI/UX improvements
   - Add requested features

## 📚 Additional Resources

- [ROS2Client Service Documentation](src/app/services/ros2Client.ts)
- [VideoPlayer Component Documentation](src/app/pages/LiveView/VideoPlayer.tsx)
- [LiveView Technical README](src/app/pages/LiveView/README.md)
- [Implementation Summary](LIVEVIEW_PAGE_SUMMARY.md)
- [roslib.js Documentation](http://robotwebtools.org/jsdoc/roslibjs/current/)
- [rosbridge Protocol Specification](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/ROSBRIDGE_PROTOCOL.md)

---

**Happy Testing! 🎉**

If you encounter any issues not covered in this guide, please:
1. Check browser console for errors
2. Check ROS2 topic availability: `ros2 topic list`
3. Check rosbridge logs: `docker logs <rosbridge-container>`
4. Create a GitHub issue with detailed reproduction steps