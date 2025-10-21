.PHONY: help install dev build preview test lint format docker-build docker-run docker-clean

# Get git hash for consistent builds
GIT_COMMIT ?= $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)
IMAGE_TAG ?= bobcamera/bob-ui:dev
IMAGE_PROD ?= bobcamera/bob-ui:latest

help:
	@echo "BOB Camera UI - Build Targets"
	@echo "=============================="
	@echo ""
	@echo "Local Development:"
	@echo "  make install       - Install dependencies with npm ci"
	@echo "  make dev           - Start Vite dev server (http://localhost:5173)"
	@echo "  make build         - Build for production"
	@echo "  make preview       - Preview production build"
	@echo "  make test          - Run tests in watch mode"
	@echo "  make lint          - Lint code"
	@echo "  make format        - Format code with Prettier"
	@echo ""
	@echo "Docker:"
	@echo "  make docker-build  - Build Docker image (uses current git commit)"
	@echo "  make docker-run    - Run Docker container on port 8080"
	@echo "  make docker-clean  - Remove Docker image"
	@echo ""
	@echo "Examples:"
	@echo "  make docker-build              # Auto-detects git hash"
	@echo "  make docker-build GIT_COMMIT=abc1234  # Override git hash"
	@echo "  make docker-run                # Run on http://localhost:8080"

install:
	npm ci

dev:
	npm run dev

build:
	npm run build

preview: build
	npm run preview

test:
	npm run test

lint:
	npm run lint

format:
	npm run format

docker-build:
	@echo "Building Docker image with GIT_COMMIT=$(GIT_COMMIT)"
	docker build \
		--build-arg GIT_COMMIT=$(GIT_COMMIT) \
		-t $(IMAGE_TAG) \
		-t $(IMAGE_PROD) \
		.

docker-run:
	@echo "Running Docker container on http://localhost:8080"
	@echo "Press Ctrl+C to stop"
	docker run \
		--rm \
		-p 8080:80 \
		--name bob-ui \
		$(IMAGE_TAG)

docker-clean:
	@echo "Removing Docker image"
	docker rmi $(IMAGE_TAG) $(IMAGE_PROD) 2>/dev/null || true

# For CI/CD: reproducible build without .git
docker-ci:
	docker build \
		--build-arg GIT_COMMIT=unknown \
		-t $(IMAGE_TAG) \
		-t $(IMAGE_PROD) \
		.