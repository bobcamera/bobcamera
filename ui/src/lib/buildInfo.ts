/**
 * Build-time information helper
 * 
 * Provides commit hash and build metadata that gracefully handles
 * environments where git is unavailable (e.g., Docker builds).
 */

import { execSync } from 'node:child_process'

/**
 * Get the current git commit hash.
 * 
 * Priority:
 * 1. GIT_COMMIT environment variable (passed from Docker build-arg)
 * 2. git rev-parse --short HEAD (local repo)
 * 3. 'unknown' (fallback when git unavailable or not in repo)
 * 
 * @returns Short commit hash or 'unknown'
 */
export function getGitCommit(): string {
  // First, check if explicitly provided via env (Docker build-arg)
  if (process.env.GIT_COMMIT && process.env.GIT_COMMIT.trim()) {
    return process.env.GIT_COMMIT.trim()
  }

  // Try to get from local git repo
  try {
    return execSync('git rev-parse --short HEAD', {
      stdio: ['ignore', 'pipe', 'ignore'],
      encoding: 'utf-8',
    }).trim()
  } catch {
    // Git not available or not in a repo - expected in Docker context
    return 'unknown'
  }
}

/**
 * Get build metadata object
 * 
 * @returns Object with commit hash and build timestamp
 */
export function getBuildInfo() {
  return {
    commit: getGitCommit(),
    timestamp: new Date().toISOString(),
  }
}