# Comprehensive Testing Guide for PR Submission

**Purpose**: Ensure the modern React UI (v0.9.0) is fully tested and production-ready before submitting a Pull Request.

**Date**: January 2025  
**Target**: Upstream bobcamera repository

---

## Testing Strategy Overview

Since you don't have RTSP cameras available, we'll use a **three-tier testing approach**:

1. **Mock Mode Testing** - UI-only testing without backend
2. **Backend with Video Files** - Full stack testing with simulated video
3. **Manual UI Testing** - Comprehensive feature verification

---

## Phase 1: Mock Mode Testing (No Backend Required)

### Setup

```bash
cd c:\bobcamera\ui

# Enable mock mode
# Edit .env and set:
# VITE_MOCK_MODE=true

# Start dev server
npm run dev
```

### Test Checklist

#### ✅ Navigation & Layout
- [ ] All 8 menu items visible in sidebar
- [ ] Active route highlighting works
- [ ] Header shows version badges (FE v0.9.0, BE v1.7.5, git hash)
- [ ] Dark mode toggle works
- [ ] Responsive layout on different window sizes

#### ✅ Dashboard Page
- [ ] Loads without errors
- [ ] Shows "Backend Offline" or mock data
- [ ] Metric cards display properly
- [ ] No console errors

#### ✅ Cameras Page
- [ ] Empty state shows: "No cameras configured"
- [ ] "Add Camera" button visible
- [ ] Modal opens when clicking "Add Camera"
- [ ] Form validation works
- [ ] No TypeScript errors in console

#### ✅ Live View Page
- [ ] Empty state shows: "No cameras available"
- [ ] Helpful message displayed
- [ ] No crashes or errors

#### ✅ Tracks Page
- [ ] Empty state shows: "No tracks recorded"
- [ ] Table structure visible
- [ ] Filter controls present
- [ ] No errors

#### ✅ Recordings Page
- [ ] Empty state shows: "No recordings available"
- [ ] Grid/list layout renders
- [ ] No errors

#### ✅ Settings Page
- [ ] Settings form loads
- [ ] All input fields render
- [ ] Toggle switches work
- [ ] No errors

#### ✅ System Page
- [ ] System health section visible
- [ ] Service status cards render
- [ ] No errors

#### ✅ Logs Page
- [ ] Log viewer renders
- [ ] Filter controls present
- [ ] Empty state or mock logs shown
- [ ] No errors

---

## Phase 2: Backend Testing with Video Files

### Prerequisites

You'll need to run the backend in WSL (Windows Subsystem for Linux) with Docker.

### Setup Backend

```bash
# In WSL terminal (Ubuntu)
cd ~/bobcamera

# Copy the video simulator config
cp src/ros2/src/bob_launch/config/video_simulator_config.yaml my_test_config.yaml

# Verify video files exist
ls -la media/fisheye_videos/

# Expected files:
# - mike-drone.mp4
# - mikeg-30min.mp4
```

### Start Backend

```bash
# In WSL terminal
cd ~/bobcamera

# Run with video simulator config
./run.sh my_test_config.yaml
```

**Expected Output**:
- Docker containers starting
- ROS2 nodes launching
- Video processing from files
- Backend API on port 8080
- Rosbridge WebSocket on port 9090

### Configure UI for Backend

```bash
# In PowerShell (Windows)
cd c:\bobcamera\ui

# Edit .env and set:
# VITE_MOCK_MODE=false
# VITE_API_BASE_URL=/api
# VITE_WS_BASE_URL=ws://localhost:8080/ws
# VITE_ROS2_WS_URL=ws://localhost:9090

# Start UI dev server
npm run dev
```

### Test Checklist with Backend

#### ✅ Backend Connection
- [ ] Open browser to http://localhost:5173
- [ ] Check browser console - no connection errors
- [ ] Version badges show correct versions
- [ ] Backend status indicator shows "Online" or "Connected"

#### ✅ Dashboard Page
- [ ] System metrics load from backend
- [ ] Camera count shows (should be 1 from video file)
- [ ] Health status displays
- [ ] Real-time updates work (if WebSocket connected)

#### ✅ Cameras Page
- [ ] Camera list loads from backend
- [ ] Video file camera appears in list
- [ ] Camera status shows (enabled/disabled)
- [ ] Can view camera details
- [ ] Can edit camera settings (test with non-destructive changes)
- [ ] "Test Connection" button works

#### ✅ Live View Page
- [ ] Camera selector shows available camera
- [ ] Video stream loads (ROS2 WebSocket)
- [ ] Video plays smoothly
- [ ] Detection overlays appear (if objects detected)
- [ ] Controls work (play/pause if available)
- [ ] FPS counter displays

#### ✅ Tracks Page
- [ ] Track history loads from backend
- [ ] Table shows detection data
- [ ] Pagination works
- [ ] Filtering works (by date, type, etc.)
- [ ] Detail drawer opens when clicking track
- [ ] Track details display correctly

#### ✅ Recordings Page
- [ ] Recording list loads from backend
- [ ] Thumbnails display (if available)
- [ ] Can play recording
- [ ] Can download recording
- [ ] Pagination works
- [ ] Date filtering works

#### ✅ Settings Page
- [ ] Current settings load from backend
- [ ] Can modify settings
- [ ] Save button works
- [ ] Settings persist after refresh
- [ ] Validation works

#### ✅ System Page
- [ ] System health metrics load
- [ ] Service status shows (ROS2 nodes, etc.)
- [ ] CPU/Memory metrics display
- [ ] Real-time updates work

#### ✅ Logs Page
- [ ] Logs load from backend
- [ ] Log entries display with timestamps
- [ ] Log level filtering works
- [ ] Search/filter functionality works
- [ ] Auto-refresh works (if enabled)
- [ ] Can scroll through logs

---

## Phase 3: Integration Testing

### WebSocket Testing

#### ✅ Real-time Updates
- [ ] Open Dashboard in one tab
- [ ] Open Live View in another tab
- [ ] Verify both receive real-time updates
- [ ] Check WebSocket connection in DevTools → Network → WS
- [ ] Verify no connection drops or errors

#### ✅ Reconnection Handling
- [ ] Stop backend (Ctrl+C in WSL)
- [ ] UI shows "Backend Offline" or connection error
- [ ] Restart backend
- [ ] UI automatically reconnects
- [ ] Data resumes flowing

### Error Handling

#### ✅ Network Errors
- [ ] Disconnect network briefly
- [ ] UI shows appropriate error messages
- [ ] Reconnect network
- [ ] UI recovers gracefully

#### ✅ Invalid Data
- [ ] UI handles missing data gracefully
- [ ] Empty states show when appropriate
- [ ] No crashes or white screens

---

## Phase 4: Browser Compatibility

Test in multiple browsers:

### ✅ Chrome/Edge (Chromium)
- [ ] All features work
- [ ] No console errors
- [ ] Performance is good

### ✅ Firefox
- [ ] All features work
- [ ] No console errors
- [ ] WebSocket connections work

### ✅ Safari (if available)
- [ ] All features work
- [ ] No console errors

---

## Phase 5: Performance Testing

### ✅ Load Time
- [ ] Initial page load < 3 seconds
- [ ] Navigation between pages is instant
- [ ] No lag or stuttering

### ✅ Memory Usage
- [ ] Open DevTools → Performance → Memory
- [ ] Monitor memory usage over 5 minutes
- [ ] No memory leaks (memory should stabilize)

### ✅ Video Streaming
- [ ] Video plays smoothly (no stuttering)
- [ ] FPS stays consistent
- [ ] No dropped frames

---

## Phase 6: Build & Production Testing

### ✅ Production Build
```bash
cd c:\bobcamera\ui

# Type check
npm run typecheck
# Expected: No errors

# Build
npm run build
# Expected: Build succeeds, dist/ folder created

# Preview production build
npm run preview
# Expected: Server starts on port 4173
```

### ✅ Production Testing
- [ ] Open http://localhost:4173
- [ ] All features work in production build
- [ ] No console errors
- [ ] Performance is good or better than dev

---

## Phase 7: Code Quality Checks

### ✅ Linting
```bash
npm run lint
# Expected: No errors (warnings are okay)
```

### ✅ Type Checking
```bash
npm run typecheck
# Expected: No TypeScript errors
```

### ✅ Tests (if available)
```bash
npm run test:ci
# Expected: All tests pass
```

---

## Phase 8: Documentation Review

### ✅ Documentation Completeness
- [ ] README.md is up to date
- [ ] DOCUMENTATION.md index is complete
- [ ] All guides are accurate
- [ ] QA Requirements Report is current
- [ ] Mantine v8 migration documented

### ✅ Code Comments
- [ ] Complex functions have JSDoc comments
- [ ] Component props are documented
- [ ] No TODO comments left unresolved

---

## Common Issues & Solutions

### Issue: Backend won't start in WSL

**Solution**:
```bash
# Check Docker is running
docker ps

# If not, start Docker Desktop on Windows
# Ensure WSL integration is enabled in Docker Desktop settings

# Rebuild if needed
cd ~/bobcamera
docker-compose down
docker-compose build
```

### Issue: Video files not found

**Solution**:
```bash
# Check if video files exist
ls -la ~/bobcamera/media/fisheye_videos/

# If missing, you may need to download or use different videos
# Update my_test_config.yaml with correct paths
```

### Issue: UI can't connect to backend

**Solution**:
```bash
# Check backend is running
curl http://localhost:8080/api/health

# Check ports are accessible from Windows
# WSL should expose ports automatically

# Verify .env settings in UI
cat c:\bobcamera\ui\.env
```

### Issue: ROS2 WebSocket not connecting

**Solution**:
```bash
# Check rosbridge is running
# In WSL:
ros2 node list | grep rosbridge

# Check port 9090 is accessible
curl http://localhost:9090

# Verify VITE_ROS2_WS_URL in .env
```

---

## Pre-PR Checklist

Before submitting your PR, ensure:

### ✅ Code Quality
- [ ] All TypeScript errors resolved
- [ ] No ESLint errors
- [ ] Code is formatted (Prettier)
- [ ] No console.log statements left in code
- [ ] No commented-out code blocks

### ✅ Testing
- [ ] All Phase 1 tests pass (Mock Mode)
- [ ] All Phase 2 tests pass (Backend)
- [ ] All Phase 3 tests pass (Integration)
- [ ] Production build works
- [ ] No regressions in existing features

### ✅ Documentation
- [ ] All documentation is up to date
- [ ] CHANGELOG or commit messages are clear
- [ ] Breaking changes are documented
- [ ] Migration guide is complete (Mantine v8)

### ✅ Git
- [ ] All changes committed
- [ ] Commit messages are descriptive
- [ ] Branch is up to date with main
- [ ] No merge conflicts

### ✅ PR Description
- [ ] Clear title describing the changes
- [ ] Detailed description of what was changed
- [ ] Screenshots/videos of new features
- [ ] Testing instructions for reviewers
- [ ] Link to related issues (if any)

---

## PR Description Template

Use this template for your PR:

```markdown
## Description
Modern React UI implementation for BOB Camera using React 19, Mantine v8, and TypeScript.

## Changes
- ✅ Complete UI rewrite with modern tech stack
- ✅ 8 fully functional pages (Dashboard, Cameras, Live View, Tracks, Recordings, Settings, System, Logs)
- ✅ Real-time updates via WebSocket
- ✅ ROS2 video streaming integration
- ✅ Comprehensive documentation (23+ docs)
- ✅ Mantine v8 integration
- ✅ Full TypeScript coverage
- ✅ Responsive design with dark mode

## Testing
- ✅ Tested in mock mode (no backend)
- ✅ Tested with backend using video file simulator
- ✅ All pages functional and error-free
- ✅ WebSocket connections working
- ✅ Video streaming working
- ✅ Production build tested

## Screenshots
[Add screenshots of key pages]

## Documentation
- Complete documentation in `/ui/docs/`
- See `/ui/DOCUMENTATION.md` for index
- QA Requirements Report: `/ui/docs/status/QA_REQUIREMENTS_REPORT.md`

## Breaking Changes
None - This is a new UI that runs alongside the legacy UI.

## Migration Notes
- New UI runs on port 5173 (dev) or can be built for production
- Legacy UI remains at port 8080
- No changes to backend required
- See `/ui/docs/guides/getting-started.md` for setup

## Checklist
- [x] Code follows project style guidelines
- [x] Documentation is complete
- [x] All tests pass
- [x] Production build works
- [x] No console errors
- [x] Responsive design verified
- [x] Dark mode works
```

---

## Estimated Testing Time

- **Phase 1 (Mock Mode)**: 30-45 minutes
- **Phase 2 (Backend Setup)**: 30-60 minutes (first time)
- **Phase 2 (Backend Testing)**: 45-60 minutes
- **Phase 3 (Integration)**: 30 minutes
- **Phase 4 (Browsers)**: 20 minutes
- **Phase 5 (Performance)**: 20 minutes
- **Phase 6 (Production)**: 15 minutes
- **Phase 7 (Code Quality)**: 10 minutes
- **Phase 8 (Documentation)**: 15 minutes

**Total**: ~4-5 hours for comprehensive testing

---

## Quick Start Testing (Minimum Viable)

If you're short on time, focus on these critical tests:

1. **Mock Mode** (30 min)
   - All pages load without errors
   - Navigation works
   - Empty states display correctly

2. **Backend with Video** (60 min)
   - Backend starts successfully
   - UI connects to backend
   - Live View shows video stream
   - Tracks page shows detections
   - No console errors

3. **Production Build** (15 min)
   - Build succeeds
   - Production preview works

**Minimum Total**: ~2 hours

---

## Support

If you encounter issues during testing:

1. Check the [Common Issues](#common-issues--solutions) section above
2. Review the [Getting Started Guide](getting-started.md)
3. Check the [QA Requirements Report](../status/QA_REQUIREMENTS_REPORT.md)
4. Open an issue in the repository

---

**Good luck with your testing! You've got this! 🚀**