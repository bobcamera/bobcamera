# Version Badge Update - FE/BE Separation with Git Hash

## Overview
Updated the header version badge to display separate Frontend (FE) and Backend (BE) versions along with the current git commit hash.

## Changes Made

### 1. Package Version Update
**File:** `package.json`
- Updated version from `1.0.0` to `0.9.0`
- This reflects the current state of the UI as a work-in-progress fork

### 2. Header Component Update
**File:** `src/app/components/layout/HeaderBar.tsx`
- Changed from single version badge to three separate badges:
  - **FE v0.9.0** (blue badge) - Frontend version from package.json
  - **BE v1.7.5** (gray badge) - Backend version from upstream bobcamera
  - **fe67991** (gray monospace badge) - Current git commit hash

### 3. Store Updates
**File:** `src/app/store/systemSlice.ts`
- Added `gitHash` property to SystemSlice interface
- Added `setGitHash` action
- Updated default versions:
  - UI: `0.9.0`
  - Backend: `1.7.5` (from upstream bobcamera version.txt)
- Git hash is injected via `import.meta.env.VITE_GIT_HASH`

### 4. Build Configuration
**File:** `vite.config.ts`
- Added git hash extraction function using `execSync('git rev-parse --short HEAD')`
- Configured Vite to inject git hash as environment variable at build time
- Hash is automatically updated on each build

### 5. TypeScript Definitions
**File:** `src/vite-env.d.ts`
- Added type definitions for all Vite environment variables
- Includes `VITE_GIT_HASH` for proper TypeScript support

## Version Information

### Frontend Version: 0.9.0
- Custom UI implementation with Mantine v7
- Represents current development state
- Will increment as features are completed

### Backend Version: 1.7.5
- From upstream bobcamera repository
- Found in `/version.txt` at project root
- No backend changes made in this fork
- Safe to use upstream version number

### Git Hash
- Short hash (7 characters) of current commit
- Automatically extracted during build
- Helps identify exact code version deployed
- Format: `fe67991` (example)

## Repository Context

This is a fork of the original bobcamera project:
- **Upstream:** https://github.com/bobcamera/bobcamera.git
- **Fork:** https://github.com/twobitshortofabyte/bobcamera.git
- **Changes:** Frontend UI only (backend unchanged)

## Visual Result

Before:
```
BOB Camera [v1.0.0]
```

After:
```
BOB Camera [FE v0.9.0] [BE v1.7.5] [fe67991]
```

## Benefits

1. **Clear Separation** - Users can see FE and BE versions independently
2. **Traceability** - Git hash allows exact version identification
3. **Transparency** - Shows which parts have been modified (FE) vs unchanged (BE)
4. **Debugging** - Easier to identify which version is deployed
5. **Development Tracking** - FE version can increment independently

## Future Considerations

- FE version should be incremented as major features are completed
- BE version should be updated if/when backend changes are made
- Consider semantic versioning (MAJOR.MINOR.PATCH) for FE releases
- Git hash automatically updates on each commit

## Testing

To verify the changes:
```bash
cd ui
npm run dev
```

Check the header bar - you should see three badges with:
- FE version (blue)
- BE version (gray)
- Git hash (gray, monospace font)

## Related Files

- `/version.txt` - Backend version (1.7.5)
- `/ui/package.json` - Frontend version (0.9.0)
- Git repository - Commit hash source