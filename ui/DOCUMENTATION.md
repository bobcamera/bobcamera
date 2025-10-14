# BOB Camera UI - Documentation Index

Welcome to the BOB Camera UI documentation! This index provides quick access to all documentation organized by category.

## 📚 Quick Links

- **New to the project?** Start with [Getting Started](docs/guides/getting-started.md)
- **Want to understand the architecture?** Read [Architecture Guide](docs/guides/architecture.md)
- **Looking for page-specific docs?** Browse [Page Documentation](#page-documentation)
- **Need to check project status?** See [Project Status](docs/status/project-status.md)
- **Troubleshooting an issue?** Check [Fixes & Resolutions](#fixes--resolutions)

## 📖 Documentation Structure

```
ui/
├── DOCUMENTATION.md          # This file - main index
├── README.md                 # Project overview and quick reference
└── docs/
    ├── guides/               # Setup and development guides
    ├── pages/                # Page-specific documentation
    ├── fixes/                # Bug fixes and resolutions
    └── status/               # Project status and progress
```

## 🚀 Getting Started

### Essential Guides

| Document | Description | Audience |
|----------|-------------|----------|
| [Getting Started](docs/guides/getting-started.md) | Complete setup guide with quick start (5 min) | All developers |
| [Architecture Guide](docs/guides/architecture.md) | System architecture and design patterns | All developers |
| [Mantine Setup](docs/guides/mantine-setup.md) | Mantine UI v7 integration details | UI developers |
| [Mantine v8 Migration](docs/guides/mantine-v8-migration.md) | Mantine v7 → v8 upgrade report | UI developers |

### Quick Start (5 Minutes)

```bash
# 1. Install dependencies
cd ui && npm install

# 2. Configure environment
cp .env.example .env

# 3. Start development server
npm run dev

# 4. Open browser
# Navigate to http://localhost:5173
```

## 📄 Page Documentation

Detailed documentation for each page in the application.

### Completed Pages

| Page | Status | Documentation | Features |
|------|--------|---------------|----------|
| **Dashboard** | ✅ Complete | [cameras.md](docs/pages/cameras.md) | System overview, metrics, health |
| **Cameras** | ✅ Complete | [cameras.md](docs/pages/cameras.md) | Camera CRUD, configuration |
| **Live View** | ✅ Complete | [liveview.md](docs/pages/liveview.md) | Real-time streaming, ROS2 |
| **Tracks** | ✅ Complete | [tracks.md](docs/pages/tracks.md) | Detection history, filtering |
| **Recordings** | ✅ Complete | [recordings.md](docs/pages/recordings.md) | Video library, playback |
| **Settings** | ✅ Complete | [settings.md](docs/pages/settings.md) | System configuration |

### Live View Documentation

| Document | Description |
|----------|-------------|
| [liveview.md](docs/pages/liveview.md) | Page overview and features |
| [liveview-implementation.md](docs/pages/liveview-implementation.md) | Technical implementation details |
| [liveview-completion.md](docs/pages/liveview-completion.md) | Completion report |
| [liveview-testing.md](docs/pages/liveview-testing.md) | Testing guide and scenarios |

### Partial/Pending Pages

| Page | Status | Notes |
|------|--------|-------|
| **System** | ⏳ Partial | Needs Mantine upgrade |
| **Logs** | ⏳ Partial | Needs Mantine upgrade |

## 🔧 Development Guides

### Core Concepts

- **State Management**: Zustand with slice pattern (see [Architecture](docs/guides/architecture.md#state-management))
- **API Integration**: REST and WebSocket clients (see [Architecture](docs/guides/architecture.md#services-layer))
- **Component Library**: Mantine UI v8 (see [Mantine Setup](docs/guides/mantine-setup.md))
- **Type Safety**: TypeScript + Zod validation (see [Architecture](docs/guides/architecture.md#validation-layer))

### Common Tasks

#### Adding a New Page
1. Create page component in `src/app/pages/PageName/index.tsx`
2. Add route in `src/app/router.tsx`
3. Add navigation in `src/app/components/layout/Sidebar.tsx`
4. Add tests in `src/app/pages/PageName/PageName.test.tsx`
5. Document in `docs/pages/pagename.md`

#### Adding an API Endpoint
1. Define schema in `src/app/services/schema.ts`
2. Add method in `src/app/services/api.ts`
3. Use in component via `apiClient`

#### Adding State
1. Create slice in `src/app/store/mySlice.ts`
2. Add to store in `src/app/store/index.ts`
3. Use in component via `useAppStore`

## 📊 Project Status

Track the current state of the project and implementation progress.

| Document | Description | Last Updated |
|----------|-------------|--------------|
| [Project Status](docs/status/project-status.md) | Overall progress (75% complete) | Current |
| [Implementation Status](docs/status/implementation-status.md) | Component-by-component status | Current |
| [Build Status](docs/status/build-status.md) | Build and type checking status | Current |
| [Project Summary](docs/status/project-summary.md) | High-level project overview | Current |
| [QA Requirements Report](docs/status/QA_REQUIREMENTS_REPORT.md) | Comprehensive QA verification pass | January 2025 |

### Current Progress

- **Pages**: 6/8 complete (75%)
- **Components**: 18/30 complete (60%)
- **Test Coverage**: ~30% (target: 80%+)
- **Documentation**: 100% for completed features

## 🐛 Fixes & Resolutions

Documentation of bugs fixed and their resolutions.

| Issue | Document | Status | Severity |
|-------|----------|--------|----------|
| EmptyState Icon Rendering | [emptystate-icon-fix.md](docs/fixes/emptystate-icon-fix.md) | ✅ Fixed | Critical |
| Tracks Page Import Errors | [tracks-imports-fix.md](docs/fixes/tracks-imports-fix.md) | ✅ Fixed | Critical |
| Version Badge Display | [version-badge-update.md](docs/fixes/version-badge-update.md) | ✅ Fixed | Enhancement |

### Common Issues & Solutions

#### Backend Not Connecting
- Check backend is running on port 8080
- Verify `.env` has correct `VITE_API_BASE_URL`
- Check browser console for CORS errors

#### WebSocket Not Connecting
- Verify `VITE_WS_BASE_URL` in `.env`
- Check WebSocket endpoint in DevTools → Network → WS
- Ensure backend WebSocket server is running

#### Build Errors
- Clear cache: `rm -rf node_modules/.vite`
- Reinstall: `rm -rf node_modules && npm install`
- Check Node version: `node --version` (should be 20+)

## 🧪 Testing

### Test Commands

```bash
# Watch mode (development)
npm run test

# Single run (CI)
npm run test:ci

# UI mode (visual)
npm run test:ui

# Coverage report
npm run test:coverage
```

### Testing Documentation

- Unit tests for components and utilities
- Integration tests for page interactions
- E2E tests for critical user flows (planned)

See [Getting Started - Testing](docs/guides/getting-started.md#running-tests) for details.

## 🏗️ Architecture

### System Overview

```
┌─────────────────────────────────────────┐
│           Presentation Layer            │
│  (Pages, Components, Hooks)             │
├─────────────────────────────────────────┤
│          State Management Layer         │
│  (Zustand Slices, Computed Values)      │
├─────────────────────────────────────────┤
│           Services Layer                │
│  (API Client, WebSocket Client)         │
├─────────────────────────────────────────┤
│          Validation Layer               │
│  (Zod Schemas, Type Guards)             │
├─────────────────────────────────────────┤
│           Backend APIs                  │
│  (REST, WebSocket, Streams)             │
└─────────────────────────────────────────┘
```

See [Architecture Guide](docs/guides/architecture.md) for detailed information.

## 🎨 UI Components

### Component Library

The UI uses **Mantine v8** as the primary component library:

- **Layout**: Container, Grid, Group, Stack, Card
- **Inputs**: TextInput, NumberInput, Select, DatePicker, Switch, Slider
- **Display**: Table, Badge, Alert, Modal, Drawer, Tabs
- **Feedback**: Loader, Progress, Notification
- **Navigation**: Pagination, Menu, Breadcrumbs

See [Mantine Setup](docs/guides/mantine-setup.md) for usage examples.

### Custom Components

- **MetricCard**: Display metrics with icons and trends
- **StatusBadge**: Color-coded status indicators
- **DataTable**: Sortable, paginated tables
- **ConfirmDialog**: Confirmation modals
- **EmptyState**: Empty/error state displays
- **VideoPlayer**: ROS2 video streaming

## 🔗 External Resources

### Official Documentation

- [Mantine UI](https://mantine.dev/) - Component library
- [React Router](https://reactrouter.com/) - Routing
- [Zustand](https://github.com/pmndrs/zustand) - State management
- [Vitest](https://vitest.dev/) - Testing framework
- [Vite](https://vitejs.dev/) - Build tool
- [TypeScript](https://www.typescriptlang.org/) - Language
- [Zod](https://zod.dev/) - Schema validation

### Tools

- [Tabler Icons](https://tabler-icons.io/) - Icon library
- [Mantine Colors](https://mantine.dev/colors-generator/) - Color generator
- [TypeScript Playground](https://www.typescriptlang.org/play) - TS testing

## 📝 Contributing

### Documentation Guidelines

When adding or updating documentation:

1. **Location**:
   - Guides → `docs/guides/`
   - Page docs → `docs/pages/`
   - Bug fixes → `docs/fixes/`
   - Status updates → `docs/status/`

2. **Format**:
   - Use Markdown
   - Include table of contents for long docs
   - Add code examples where relevant
   - Include screenshots for UI features

3. **Naming**:
   - Use lowercase with hyphens: `my-document.md`
   - Be descriptive: `liveview-testing.md` not `test.md`

4. **Updates**:
   - Update this index when adding new docs
   - Update related docs when making changes
   - Keep status docs current

### Code Documentation

- Add JSDoc comments for complex functions
- Document component props with TypeScript interfaces
- Include usage examples in component files
- Write tests that serve as documentation

## 🚀 Deployment

### Production Build

```bash
# Type check
npm run typecheck

# Build
npm run build

# Preview
npm run preview
```

### Docker Deployment

```bash
# Build image
docker build -t bobcamera/bob-ui:latest .

# Run container
docker run -d --name bob-ui -p 8080:80 bobcamera/bob-ui:latest
```

See [README.md](README.md#docker-deployment) for Docker Compose setup.

## 📞 Support

### Getting Help

- **Issues**: [GitHub Issues](https://github.com/bobcamera/bobcamera/issues)
- **Discussions**: [GitHub Discussions](https://github.com/bobcamera/bobcamera/discussions)
- **Wiki**: [GitHub Wiki](https://github.com/bobcamera/bobcamera/wiki)

### Reporting Issues

When reporting issues, include:
- Steps to reproduce
- Expected vs actual behavior
- Browser and OS information
- Console errors (if any)
- Screenshots (if relevant)

## 📅 Version History

### Current Version: 1.0.0

- ✅ 6/8 pages complete
- ✅ Mantine v8 integration (upgraded from v7)
- ✅ ROS2 WebSocket streaming
- ✅ Comprehensive documentation
- ⏳ System and Logs pages pending

See [Project Status](docs/status/project-status.md) for detailed progress.

## 🎯 Roadmap

### Short Term
- Complete System page
- Complete Logs page
- Increase test coverage to 80%+
- Add E2E tests

### Medium Term
- Performance optimization
- Accessibility improvements
- Mobile responsiveness enhancements
- Advanced features (notifications, alerts)

### Long Term
- User authentication
- Multi-language support (i18n)
- Advanced analytics
- Automation rules

---

**Last Updated**: January 2024  
**Maintained By**: BOB Camera Development Team  
**License**: See main repository LICENSE file

---

## Quick Navigation

- [← Back to README](README.md)
- [Getting Started →](docs/guides/getting-started.md)
- [Architecture →](docs/guides/architecture.md)
- [Project Status →](docs/status/project-status.md)