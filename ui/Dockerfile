# Multi-stage Dockerfile for BOB Camera UI
# Build reproducibly regardless of .git presence in context

# Stage 1: Builder
# Using node:20-slim for better TailwindCSS v4 + native module support
# (Alpine's musl libc incompatible with some native binaries)
FROM node:20-slim AS builder

WORKDIR /app

# Install build dependencies (git is optional, doesn't cause build failure)
# python3 is needed for npm postinstall scripts
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    make \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy package files first for better layer caching
COPY package*.json ./

# Install dependencies with npm install --force
# This ensures all native modules (lightningcss, esbuild, etc.) compile correctly
# for the Docker build environment
RUN npm install --force

# Copy application source
COPY . .

# Accept optional git commit hash from build-arg
# Falls back to 'unknown' if not provided (expected in CI/Docker without .git)
ARG GIT_COMMIT=unknown
ENV GIT_COMMIT=$GIT_COMMIT

# Build the application
# vite.config.ts uses GIT_COMMIT env var if available, then tries git, then 'unknown'
RUN npm run build

# Stage 2: Production - Serve with Nginx
FROM nginx:stable-alpine

# Copy custom nginx config
COPY --from=builder /app/nginx.conf /etc/nginx/conf.d/default.conf

# Copy built assets from builder
COPY --from=builder /app/dist /usr/share/nginx/html

# Add healthcheck
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD wget --quiet --tries=1 --spider http://localhost:80/ || exit 1

# Expose port
EXPOSE 80

# Start nginx
CMD ["nginx", "-g", "daemon off;"]