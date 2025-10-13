---
description: Repository Information Overview
alwaysApply: true
---

# BOB Camera Repository Information

## Repository Summary
BOB (Bird, Object, Bat) Camera is a universal object tracker system designed for detecting, tracking, and recording various objects including birds, insects, bats, and UAPs (Unidentified Aerial Phenomena). The project uses Docker and Docker Compose for deployment and consists of multiple components including a ROS2-based backend and a web-based frontend.

## Repository Structure
- **src/**: Core source code
  - **boblib/**: C++ library for background subtraction and tracking algorithms
  - **ros2/**: ROS2-based UAP detection pipeline
  - **web2/**, **web3/**, **web4/**: Different web interface implementations
- **ui/**: React-based frontend application
- **docker/**: Docker configuration files and compose setups
- **wsl/**: Windows Subsystem for Linux support files
- **media/**: Sample media files for testing

## Projects

### ROS2 Backend
**Configuration File**: `src/ros2/dual_camera_config.yaml`

#### Language & Runtime
**Language**: C++, Python
**Version**: C++20, Python 3.12
**Build System**: CMake, Colcon (ROS2)
**Package Manager**: apt (system packages), pip (Python packages)

#### Dependencies
**Main Dependencies**:
- ROS2 Jazzy
- OpenCV 4.x
- CUDA (optional)
- QHY SDK (camera support)
- onvif2_zeep (for ONVIF camera protocol)

#### Build & Installation
```bash
./build.sh  # For ROS2 components
```

#### Docker
**Dockerfile**: `docker/Dockerfile`
**Image**: bobcamera/bob-ros2-prod:1.7.5
**Configuration**: Multi-stage build with separate stages for development, build, and production

### C++ Library (boblib)
**Configuration File**: `src/boblib/CMakeLists.txt`

#### Language & Runtime
**Language**: C++
**Version**: C++20
**Build System**: CMake
**Package Manager**: apt (system packages)

#### Dependencies
**Main Dependencies**:
- OpenCV 4.x
- OpenMP (optional)
- TBB (for parallel execution)

#### Build & Installation
```bash
cd src/boblib/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
cmake --build . -j$(nproc)
sudo make install
```

### Web UI (React)
**Configuration File**: `ui/package.json`

#### Language & Runtime
**Language**: TypeScript, React
**Version**: React 19.1.1, TypeScript 5.8.3
**Build System**: Vite
**Package Manager**: npm/pnpm

#### Dependencies
**Main Dependencies**:
- React 19.1.1
- React Router 7.8.0
- TailwindCSS 4.1.12
- Zustand 5.0.7 (state management)
- Radix UI components

#### Build & Installation
```bash
cd ui
npm install
npm run build
```

#### Docker
**Dockerfile**: `ui/Dockerfile`
**Image**: Uses nginx to serve the built static files

### Application Deployment
**Configuration File**: `docker-compose.yml`

#### Docker
**Compose Files**: 
- `docker/docker-compose.yaml`: Default setup
- `docker/docker-compose-cuda.yaml`: CUDA-enabled setup
- `docker/docker-compose-rstudio.yaml`: Setup with RStudio

#### Usage & Operations
**Key Commands**:
```bash
./setup.sh  # Install dependencies
./run.sh <config_file.yaml>  # Run the application
```

**Integration Points**:
- Web interface accessible at http://localhost:8080
- ROS2 topics for monitoring tracking state