# PR #240 Verification Commands

These are the exact commands reviewers should run to verify the implementation.

## 1. Verify Local Build Works

```bash
cd /home/dev/code/bobcamera/ui
npm install --force
npm run build
```

**Expected Output:**
- `✓ 8624 modules transformed`
- `✓ built in 5.XX seconds`
- No errors

**Verify Git Hash Embedded:**
```bash
grep -o "gitHash:\"[^\"]*\"" dist/assets/index-*.js
# Should output: gitHash:"<current-git-hash>"
```

---

## 2. Verify Docker Build with Git Commit

```bash
cd /home/dev/code/bobcamera/ui
GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
docker build --build-arg GIT_COMMIT=$GIT_COMMIT -t bobcamera/bob-ui:test .
```

**Expected Output:**
- `✓ built in XX.XXs` (in builder stage)
- `naming to docker.io/bobcamera/bob-ui:test done`

---

## 3. Verify Docker Build Without .git (CI Simulation)

```bash
cd /home/dev/code/bobcamera/ui
docker build --build-arg GIT_COMMIT=unknown -t bobcamera/bob-ui:test .
```

**Expected Output:**
- Same as #2, but explicitly using `unknown` as GIT_COMMIT
- Build succeeds without errors

---

## 4. Verify Container Runs

```bash
docker run -d -p 8080:80 --name bob-ui-test bobcamera/bob-ui:test
sleep 2
curl -s http://localhost:8080 | head -20
docker stop bob-ui-test
docker rm bob-ui-test
```

**Expected Output:**
- HTML content returned (doctype, html, head, body tags visible)
- HTTP 200 response
- Container exits cleanly

---

## 5. Verify Using Makefile

```bash
cd /home/dev/code/bobcamera/ui
make help          # Shows all available targets
make docker-build  # Auto-detects git and builds
```

**Expected Output:**
- `help` shows targets: install, dev, build, docker-build, docker-run, docker-clean
- `docker-build` auto-detects git hash and builds successfully

---

## 6. Verify Documentation Exists

```bash
# Check key documentation files exist
[ -f "/home/dev/code/bobcamera/PR_240_IMPLEMENTATION_SUMMARY.md" ] && echo "✓ Summary doc exists"
[ -f "/home/dev/code/bobcamera/PR_240_COMMENT.md" ] && echo "✓ PR comment template exists"
[ -f "/home/dev/code/bobcamera/ui/src/lib/buildInfo.ts" ] && echo "✓ buildInfo.ts exists"
[ -f "/home/dev/code/bobcamera/ui/Makefile" ] && echo "✓ Makefile exists"

# Check key sections in README
grep -q "🚀 UI Quick Start" /home/dev/code/bobcamera/README.md && echo "✓ UI Quick Start in root README"
grep -q "## Docker Deployment" /home/dev/code/bobcamera/ui/README.md && echo "✓ Docker Deployment in UI README"
grep -q "GIT_COMMIT" /home/dev/code/bobcamera/ui/.env.example && echo "✓ GIT_COMMIT documented in .env.example"
```

**Expected Output:**
- All checks should pass with ✓ marks

---

## 7. Verify No Uncommitted Backend Changes

```bash
cd /home/dev/code/bobcamera
git status | grep -E "(src/boblib|src/ros2|docker/)" && echo "❌ Backend files changed!" || echo "✓ No backend changes"
```

**Expected Output:**
- `✓ No backend changes`

---

## Quick Verification Script

Run this all-in-one script:

```bash
#!/bin/bash
set -e

echo "PR #240 Verification"
echo "===================="
echo ""

echo "1. Checking files exist..."
[ -f "ui/src/lib/buildInfo.ts" ] && echo "   ✓ buildInfo.ts"
[ -f "ui/Makefile" ] && echo "   ✓ Makefile"
[ -f "PR_240_IMPLEMENTATION_SUMMARY.md" ] && echo "   ✓ Summary doc"

echo ""
echo "2. Running local build..."
cd ui
npm install --force > /dev/null 2>&1 && echo "   ✓ npm install"
npm run build 2>&1 | grep -q "✓ built" && echo "   ✓ build succeeds"

echo ""
echo "3. Verifying git hash..."
grep -q "gitHash:" dist/assets/index-*.js && echo "   ✓ git hash embedded"

echo ""
echo "4. Testing Docker build..."
docker build --build-arg GIT_COMMIT=test123 -t bob-ui:test . > /dev/null 2>&1 && echo "   ✓ Docker build succeeds"

echo ""
echo "5. Testing container..."
docker run --rm -p 8089:80 bob-ui:test &
CONTAINER_PID=$!
sleep 2
curl -s http://localhost:8089 | grep -q "<!doctype html" && echo "   ✓ Container serves HTML"
kill $CONTAINER_PID 2>/dev/null

echo ""
echo "✅ All verifications passed!"
```

Save as `verify_pr240.sh` and run:
```bash
cd /home/dev/code/bobcamera
bash verify_pr240.sh
```

---

## Full Test Matrix (Takes ~10 minutes)

```bash
cd /home/dev/code/bobcamera

echo "Test 1: Local Build"
cd ui && npm install --force && npm run build && cd ..

echo "Test 2: Docker with Git"
GIT_COMMIT=$(git rev-parse --short HEAD)
docker build --build-arg GIT_COMMIT=$GIT_COMMIT -t bob-ui:git-test ui/

echo "Test 3: Docker without Git"
docker build --build-arg GIT_COMMIT=unknown -t bob-ui:no-git-test ui/

echo "Test 4: Archive & Build (CI simulation)"
git archive --format=tar HEAD | tar -x -C /tmp/bob-archive-test
cd /tmp/bob-archive-test/ui
docker build -t bob-ui:archive-test .
cd -

echo "All tests passed!"
```

---

## Troubleshooting Verification

If any verification step fails, check:

1. **npm install fails**: `npm cache clean --force && npm install --force`
2. **npm build fails**: Check Node version is 20+ (`node --version`)
3. **Docker build fails**: Ensure Docker is running, disk space available
4. **Container fails to start**: Check port 8080 not already in use
5. **Git hash not embedded**: Re-run npm build after buildInfo.ts changes

---

## Success Criteria

All of the following should complete successfully:

- ✅ Local build completes in < 10 seconds
- ✅ Docker build completes in < 5 minutes
- ✅ Git hash embedded correctly
- ✅ Docker build works without .git (uses 'unknown')
- ✅ Container serves HTML on port 8080
- ✅ HTTP 200 response from container
- ✅ No backend files modified
- ✅ Documentation files exist and contain expected sections

If all above pass: **✅ Ready to merge!**