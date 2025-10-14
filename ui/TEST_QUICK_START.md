# Quick Start Testing Guide

**Goal**: Get your UI tested as quickly as possible before submitting your PR.

---

## Option 1: Mock Mode Testing (Fastest - 30 minutes)

**No backend required!** Test the UI in isolation.

### Steps

```powershell
# 1. Navigate to UI directory
cd c:\bobcamera\ui

# 2. Enable mock mode
# Open .env file and change:
# VITE_MOCK_MODE=false  →  VITE_MOCK_MODE=true

# 3. Start dev server
npm run dev

# 4. Open browser
# Go to: http://localhost:5173
```

### What to Test

✅ **Quick Checklist** (10 minutes):
- [ ] All 8 pages load without errors
- [ ] Navigation works
- [ ] Version badges show in header
- [ ] Dark mode toggle works
- [ ] No red errors in browser console (F12)

✅ **Detailed Checklist** (30 minutes):
- [ ] Dashboard shows empty/offline state
- [ ] Cameras page shows "No cameras configured"
- [ ] Live View shows "No cameras available"
- [ ] Tracks page shows "No tracks recorded"
- [ ] Recordings page shows "No recordings available"
- [ ] Settings page loads and form works
- [ ] System page loads
- [ ] Logs page loads

**Result**: If all pages load without errors, your UI is solid! ✅

---

## Option 2: Full Backend Testing (Complete - 2 hours)

**Test with real backend and video streaming.**

### Prerequisites

- WSL installed (Windows Subsystem for Linux)
- Docker Desktop installed and running
- WSL integration enabled in Docker Desktop

### Part A: Setup Backend (30 minutes first time, 5 minutes after)

```bash
# Open WSL terminal (Ubuntu)
# You can open it from Windows Start Menu → Ubuntu

# 1. Navigate to bobcamera
cd ~/bobcamera

# 2. Create test config (first time only)
cp src/ros2/src/bob_launch/config/video_simulator_config.yaml my_test_config.yaml

# 3. Verify video files exist
ls -la media/fisheye_videos/
# Should see: mike-drone.mp4 and mikeg-30min.mp4

# 4. Start backend
./run.sh my_test_config.yaml

# Wait for startup messages...
# You should see:
# - Docker containers starting
# - ROS2 nodes launching
# - "rosbridge_websocket started" (important!)
```

**Troubleshooting**:
- If Docker errors: Make sure Docker Desktop is running on Windows
- If file not found: Check the path in my_test_config.yaml
- If permission denied: Run `chmod +x run.sh`

### Part B: Setup UI (5 minutes)

```powershell
# In PowerShell (Windows)
cd c:\bobcamera\ui

# 1. Disable mock mode
# Open .env file and ensure:
# VITE_MOCK_MODE=false

# 2. Verify backend URLs
# In .env file, should have:
# VITE_API_BASE_URL=/api
# VITE_WS_BASE_URL=ws://localhost:8080/ws
# VITE_ROS2_WS_URL=ws://localhost:9090

# 3. Start UI
npm run dev

# 4. Open browser
# Go to: http://localhost:5173
```

### Part C: Test Everything (60 minutes)

#### 1. Connection Check (5 min)
- [ ] Open browser console (F12)
- [ ] No connection errors
- [ ] Version badges show in header
- [ ] Backend status shows "Connected" or "Online"

#### 2. Dashboard (5 min)
- [ ] System metrics load
- [ ] Camera count shows (should be 1)
- [ ] Health status displays
- [ ] No errors

#### 3. Cameras (10 min)
- [ ] Camera list shows video file camera
- [ ] Can view camera details
- [ ] Status shows enabled/disabled
- [ ] "Test Connection" works

#### 4. Live View (15 min) ⭐ **MOST IMPORTANT**
- [ ] Camera selector shows camera
- [ ] Video stream loads and plays
- [ ] Can see video from file playing
- [ ] Detection overlays appear (if objects detected)
- [ ] No stuttering or lag
- [ ] FPS counter shows

**If Live View works, you're 90% there!** 🎉

#### 5. Tracks (10 min)
- [ ] Track history loads
- [ ] Table shows detections
- [ ] Can click on track for details
- [ ] Pagination works

#### 6. Recordings (10 min)
- [ ] Recording list loads
- [ ] Can play recording
- [ ] Can download recording

#### 7. Settings (5 min)
- [ ] Settings load
- [ ] Can modify and save
- [ ] Changes persist

#### 8. System (5 min)
- [ ] System health loads
- [ ] Service status shows

#### 9. Logs (5 min)
- [ ] Logs load
- [ ] Can filter logs
- [ ] Timestamps show

### Part D: Production Build Test (10 minutes)

```powershell
# In PowerShell
cd c:\bobcamera\ui

# 1. Type check
npm run typecheck
# Should show: No errors

# 2. Build
npm run build
# Should complete successfully

# 3. Preview
npm run preview
# Opens on http://localhost:4173

# 4. Quick test
# Open browser to http://localhost:4173
# Verify main pages load
```

---

## Option 3: Minimum Viable Testing (Fastest Path to PR - 1 hour)

**The absolute minimum to feel confident.**

### Phase 1: Mock Mode (20 min)
```powershell
cd c:\bobcamera\ui
# Set VITE_MOCK_MODE=true in .env
npm run dev
```
- [ ] Visit all 8 pages
- [ ] No console errors
- [ ] All pages render

### Phase 2: Backend Quick Test (30 min)
```bash
# WSL
cd ~/bobcamera
./run.sh my_test_config.yaml
```
```powershell
# PowerShell
cd c:\bobcamera\ui
# Set VITE_MOCK_MODE=false in .env
npm run dev
```
- [ ] Dashboard loads with data
- [ ] Live View shows video
- [ ] No console errors

### Phase 3: Build (10 min)
```powershell
npm run typecheck
npm run build
npm run preview
```
- [ ] All commands succeed
- [ ] Preview works

**Done!** You're ready to submit your PR! 🚀

---

## Common Issues

### "Backend won't start"
```bash
# Check Docker
docker ps

# If empty, start Docker Desktop on Windows
# Then try again
```

### "UI can't connect to backend"
```bash
# In WSL, check backend is running:
curl http://localhost:8080/api/health

# Should return JSON with health status
```

### "Video not loading in Live View"
```bash
# Check rosbridge is running:
# In WSL:
ros2 node list | grep rosbridge

# Should show: /rosbridge_websocket
```

### "Port already in use"
```powershell
# Kill process on port 5173:
Get-Process -Id (Get-NetTCPConnection -LocalPort 5173).OwningProcess | Stop-Process -Force

# Or use different port:
npm run dev -- --port 5174
```

---

## Testing Checklist Summary

### Must Have ✅
- [ ] All pages load without errors
- [ ] Navigation works
- [ ] No TypeScript errors (`npm run typecheck`)
- [ ] Production build works (`npm run build`)

### Should Have ✅
- [ ] Backend connection works
- [ ] Live View shows video
- [ ] Tracks page shows data
- [ ] WebSocket connections work

### Nice to Have ✅
- [ ] All CRUD operations work
- [ ] Real-time updates work
- [ ] Performance is good
- [ ] Tested in multiple browsers

---

## Time Estimates

| Testing Level | Time Required | Confidence Level |
|---------------|---------------|------------------|
| **Mock Mode Only** | 30 min | 60% - UI works, backend unknown |
| **Minimum Viable** | 1 hour | 80% - Core features verified |
| **Full Backend** | 2 hours | 95% - Everything tested |
| **Comprehensive** | 4-5 hours | 99% - Production ready |

---

## Ready to Submit PR?

### Pre-submission Checklist

- [ ] All tests pass (at minimum: Mock Mode + Build)
- [ ] No TypeScript errors
- [ ] No console errors
- [ ] Documentation is up to date
- [ ] Commit messages are clear
- [ ] Branch is up to date with main

### PR Title Suggestion
```
feat: Modern React UI implementation with Mantine v8
```

### PR Description
See the template in [comprehensive-testing-guide.md](docs/guides/comprehensive-testing-guide.md#pr-description-template)

---

## Need Help?

1. Check [Comprehensive Testing Guide](docs/guides/comprehensive-testing-guide.md) for detailed instructions
2. Check [Getting Started Guide](docs/guides/getting-started.md) for setup help
3. Check [QA Requirements Report](docs/status/QA_REQUIREMENTS_REPORT.md) for what's been verified

---

**You've got this! Your UI is solid and well-documented. Time to share it with the world! 🎉**