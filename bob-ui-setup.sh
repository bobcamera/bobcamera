# ===== BOB CAMERA UI: end-to-end setup & build (WSL) =====
# Run this in WSL Ubuntu. Do NOT run in PowerShell.
set -euo pipefail

say() { printf "\n[***] %s\n" "$*"; }

# 0) Sanity checks
say "Verifying we are in WSL Linux…"
if ! uname -a | grep -qi "microsoft"; then
  echo "Not in WSL. Open Ubuntu (WSL) and run again."; exit 1
fi

# 1) Ensure repo path exists and cd there
REPO="${HOME}/code/bobcamera"
UI_DIR="${REPO}/ui"

if [ ! -d "$REPO" ]; then
  echo "Repo folder not found at ${REPO}. If needed, clone it:"
  echo "  mkdir -p ~/code && cd ~/code"
  echo "  gh repo clone bobcamera/bobcamera && cd bobcamera"
  echo "  gh pr checkout 240 || (git fetch origin pull/240/head:pr-240 && git checkout pr-240)"
  exit 1
fi

cd "$REPO"
say "Confirming branch and submodules…"
git fetch --all -q || true
git branch -vv || true
git submodule update --init --recursive || true

if [ ! -d "$UI_DIR" ]; then
  echo "Expected UI folder at ${UI_DIR} but it does not exist. Check the branch."; exit 1
fi

cd "$UI_DIR"

# 2) Base packages, nvm, Node 20
say "Installing base packages, nvm, and Node 20…"
sudo apt-get update -y
sudo apt-get install -y build-essential curl ca-certificates git

# nvm install (idempotent)
export NVM_DIR="$HOME/.nvm"
if [ ! -d "$NVM_DIR" ]; then
  curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
fi
# load nvm in this shell
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"

nvm install 20 >/dev/null
nvm use 20 >/dev/null

say "Node: $(node -v), npm: $(npm -v)"

# 3) Detect package manager, install deps, local build
PM="npm"
if [ -f "pnpm-lock.yaml" ]; then
  say "pnpm-lock.yaml found → using pnpm"
  PM="pnpm"
  corepack enable >/dev/null 2>&1 || true
  corepack prepare pnpm@latest --activate >/dev/null 2>&1 || true
  say "pnpm: $(pnpm -v)"
fi

say "Installing dependencies with ${PM}…"
if [ "$PM" = "pnpm" ]; then
  pnpm install
else
  if [ -f "package-lock.json" ]; then
    npm ci
  else
    npm install
  fi
fi

say "Running local TypeScript build with ${PM}…"
if [ "$PM" = "pnpm" ]; then
  pnpm run build
else
  npm run build
fi
say "LOCAL BUILD: PASS"

# 4) Docker availability from WSL
say "Checking Docker daemon from WSL…"
if ! docker info >/dev/null 2>&1; then
  echo "Docker not reachable from WSL. In Docker Desktop:"
  echo "  Settings → General → Use the WSL 2 based engine"
  echo "  Settings → Resources → WSL Integration → enable for Ubuntu"
  echo "Then reopen this WSL terminal and rerun."
  exit 1
fi

# 5) Build a temporary Docker image that targets /ui (no repo edits)
say "Building temporary Docker image for the UI (Node 20 → Nginx)…"
TMP_DOCKERFILE="$(mktemp)"
cat > "$TMP_DOCKERFILE" <<'DOCKERFILE'
# ---- Stage 1: build UI ----
FROM node:20-bullseye AS build
WORKDIR /app/ui

# Copy manifests first for better caching
COPY ui/package.json ui/package-lock.json* ui/pnpm-lock.yaml* ./ 2>/dev/null || true

# Install package manager and deps
# Prefer pnpm if lockfile exists
RUN bash -lc 'set -e; \
  if [ -f pnpm-lock.yaml ]; then \
    corepack enable && corepack prepare pnpm@8 --activate && pnpm install; \
  elif [ -f package-lock.json ]; then \
    npm ci; \
  else \
    npm install; \
  fi'

# Copy the rest of the UI and build
COPY ui/ ./
RUN bash -lc 'set -e; \
  if [ -f pnpm-lock.yaml ]; then pnpm run build; else npm run build; fi'

# ---- Stage 2: serve with nginx ----
FROM nginx:1.27-alpine
COPY --from=build /app/ui/dist /usr/share/nginx/html
EXPOSE 80
CMD ["nginx","-g","daemon off;"]
DOCKERFILE

# Build context is repo root because we COPY ui/…
cd "$REPO"
DOCKER_TAG="bobcamera-ui:pr-240-temp"
DOCKER_BUILDKIT=1 docker buildx build \
  --platform linux/amd64 \
  -f "$TMP_DOCKERFILE" \
  -t "$DOCKER_TAG" \
  --pull \
  --no-cache \
  .

say "DOCKER BUILD: PASS → image tagged as ${DOCKER_TAG}"

# 6) Optional: run the container locally on :8080
say "Starting container on http://localhost:8080 (Ctrl+C to stop)…"
docker run --rm -p 8080:80 "$DOCKER_TAG"
