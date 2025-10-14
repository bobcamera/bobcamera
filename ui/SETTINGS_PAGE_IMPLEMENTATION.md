# Settings Page Implementation

## Overview

The Settings page provides a comprehensive configuration interface for the BOB camera system. It allows users to configure detection, tracking, storage, and network parameters with real-time validation and a draft/save workflow.

## Features

### 1. **Tabbed Interface**
- **Detection Tab**: Configure object detection parameters
- **Tracking Tab**: Configure object tracking parameters
- **Storage Tab**: Configure recording storage settings
- **Network Tab**: Configure network ports and connectivity

### 2. **Draft/Save Workflow**
- Changes are made to a draft configuration
- Original configuration remains unchanged until saved
- Visual indicator for unsaved changes
- Ability to reset/discard changes
- Confirmation modal before discarding

### 3. **Smart Form Controls**

#### Detection Settings
- **Enable/Disable Toggle**: Master switch for detection
- **Confidence Threshold Slider**: Visual slider with marks (0.0 - 1.0)
- **NMS Threshold Slider**: Visual slider with marks (0.0 - 1.0)
- **Class Selection**: Multi-select dropdown for object classes
- **Disabled State**: All controls disabled when detection is off

#### Tracking Settings
- **Enable/Disable Toggle**: Master switch for tracking
- **Max Track Age**: Number input with validation (1-300 frames)
- **Min Hits**: Number input with validation (1-20 detections)
- **IOU Threshold Slider**: Visual slider with marks (0.0 - 1.0)
- **Disabled State**: All controls disabled when tracking is off

#### Storage Settings
- **Enable/Disable Toggle**: Master switch for storage
- **Storage Path**: Text input with monospace font
- **Max Storage Size**: Number input with GB suffix (auto-converts bytes)
- **Retention Period**: Number input with days suffix (1-365 days)
- **Disabled State**: All controls disabled when storage is off

#### Network Settings
- **API Port**: Number input (1024-65535)
- **WebSocket Port**: Number input (1024-65535)
- **Stream Port**: Number input (1024-65535)
- **Warning Alert**: Notifies users about restart requirements

### 4. **User Experience Enhancements**

#### Visual Feedback
- **Status Badges**: Show enabled/disabled state for each section
- **Unsaved Changes Alert**: Yellow alert banner when changes pending
- **Loading State**: Spinner while fetching configuration
- **Disabled State**: Visual indication when backend disconnected

#### Validation
- **Range Validation**: All numeric inputs have min/max constraints
- **Step Validation**: Sliders use appropriate step values
- **Type Safety**: TypeScript ensures type correctness

#### Help Text
- **Labels**: Clear, descriptive labels for all fields
- **Descriptions**: Helpful descriptions under each field
- **Tooltips**: Additional context for complex settings
- **Units**: Clear unit indicators (GB, days, frames, etc.)

### 5. **Error Handling**
- **Backend Disconnected**: Shows empty state with retry button
- **Save Errors**: Toast notification with error message
- **Network Errors**: Graceful degradation with user feedback

### 6. **Notifications**
- **Success**: Green notification when settings saved
- **Error**: Red notification when save fails
- **Info**: Blue notification when changes discarded

## Technical Implementation

### Component Structure

```tsx
Settings
├── Header (Title + Action Buttons)
├── Unsaved Changes Alert (conditional)
└── Tabs
    ├── Detection Tab
    │   ├── Enable Switch
    │   ├── Confidence Slider
    │   ├── NMS Slider
    │   └── Class Multi-Select
    ├── Tracking Tab
    │   ├── Enable Switch
    │   ├── Max Age Input
    │   ├── Min Hits Input
    │   └── IOU Slider
    ├── Storage Tab
    │   ├── Enable Switch
    │   ├── Path Input
    │   ├── Max Size Input
    │   └── Retention Input
    └── Network Tab
        ├── Warning Alert
        ├── API Port Input
        ├── WebSocket Port Input
        └── Stream Port Input
```

### State Management

The Settings page uses Zustand store with the following state:

```typescript
interface SettingsSlice {
  config: Config | null              // Current saved configuration
  draftConfig: Config | null         // Draft configuration (being edited)
  saveStatus: 'idle' | 'saving' | 'success' | 'error'
  error: string | null
  
  // Actions
  fetchConfig: () => Promise<void>
  saveConfig: (config: Config) => Promise<void>
  updateDraftConfig: (updates: Partial<Config>) => void
  resetDraftConfig: () => void
  hasPendingChanges: () => boolean
}
```

### Configuration Schema

```typescript
interface Config {
  detection: {
    enabled: boolean
    confidence: number      // 0.0 - 1.0
    nms: number            // 0.0 - 1.0
    classes: string[]      // Object classes to detect
  }
  tracking: {
    enabled: boolean
    maxAge: number         // Frames
    minHits: number        // Detections
    iouThreshold: number   // 0.0 - 1.0
  }
  storage: {
    enabled: boolean
    path: string           // Directory path
    maxSize: number        // Bytes
    retention: number      // Days
  }
  network: {
    apiPort: number        // 1024-65535
    wsPort: number         // 1024-65535
    streamPort: number     // 1024-65535
  }
}
```

### Mantine Components Used

| Component | Purpose |
|-----------|---------|
| `Container` | Page layout container |
| `Title` | Page title |
| `Text` | Text content and descriptions |
| `Tabs` | Tabbed interface |
| `Card` | Section containers |
| `Button` | Action buttons |
| `Group` | Horizontal layout |
| `Stack` | Vertical layout |
| `NumberInput` | Numeric input fields |
| `TextInput` | Text input fields |
| `Switch` | Toggle switches |
| `Slider` | Range sliders |
| `MultiSelect` | Multi-select dropdown |
| `Alert` | Warning/info messages |
| `Badge` | Status indicators |
| `Divider` | Visual separators |
| `Loader` | Loading spinner |
| `Center` | Centered content |

### API Integration

```typescript
// Fetch configuration
const config = await apiClient.getConfig()

// Save configuration
await apiClient.updateConfig(config)
```

## User Workflows

### 1. Viewing Settings
1. Navigate to `/settings`
2. Page loads current configuration
3. View settings organized by category

### 2. Editing Settings
1. Navigate to desired tab
2. Modify settings using form controls
3. See unsaved changes alert appear
4. Continue editing or save/reset

### 3. Saving Changes
1. Make desired changes
2. Click "Save Changes" button
3. See loading state on button
4. Receive success notification
5. Unsaved changes alert disappears

### 4. Discarding Changes
1. Make some changes
2. Click "Reset" button
3. Confirm in modal dialog
4. Changes are discarded
5. Form resets to saved values

### 5. Handling Errors
1. Backend disconnects during editing
2. Page shows empty state
3. Click "Retry" to reconnect
4. Resume editing

## Validation Rules

### Detection
- **Confidence**: 0.0 - 1.0, step 0.05
- **NMS**: 0.0 - 1.0, step 0.05
- **Classes**: At least one class recommended

### Tracking
- **Max Age**: 1 - 300 frames
- **Min Hits**: 1 - 20 detections
- **IOU Threshold**: 0.0 - 1.0, step 0.05

### Storage
- **Path**: Non-empty string
- **Max Size**: 1 - 10,000 GB (converted to bytes)
- **Retention**: 1 - 365 days

### Network
- **All Ports**: 1024 - 65535 (non-privileged ports)
- **Unique Ports**: Each port should be different

## Accessibility

- **Keyboard Navigation**: All controls accessible via keyboard
- **Focus Management**: Proper focus indicators
- **Screen Readers**: Descriptive labels and ARIA attributes
- **Color Contrast**: Meets WCAG AA standards
- **Disabled States**: Clear visual indication

## Performance Considerations

- **Debouncing**: Slider changes are debounced to prevent excessive updates
- **Memoization**: Draft config updates are optimized
- **Lazy Loading**: Tabs content loaded on demand
- **Efficient Rendering**: Only changed fields trigger re-renders

## Testing Scenarios

### 1. Basic Functionality
- [ ] Page loads without errors
- [ ] All tabs are accessible
- [ ] All form controls work correctly
- [ ] Save button saves changes
- [ ] Reset button discards changes

### 2. Draft Workflow
- [ ] Changes create draft config
- [ ] Unsaved changes alert appears
- [ ] Save applies changes
- [ ] Reset discards changes
- [ ] Confirmation modal works

### 3. Validation
- [ ] Numeric inputs respect min/max
- [ ] Sliders snap to step values
- [ ] Port numbers validated
- [ ] Path input accepts valid paths

### 4. Error Handling
- [ ] Backend disconnect shows empty state
- [ ] Save errors show notification
- [ ] Retry button reconnects
- [ ] Network errors handled gracefully

### 5. User Experience
- [ ] Loading state shows spinner
- [ ] Status badges update correctly
- [ ] Disabled states work properly
- [ ] Notifications appear and dismiss
- [ ] Help text is clear and helpful

## Future Enhancements

### 1. Advanced Features
- **Import/Export**: Export config as JSON, import from file
- **Presets**: Save and load configuration presets
- **Validation**: Real-time validation with error messages
- **Diff View**: Show changes before saving
- **History**: View configuration change history

### 2. Detection Settings
- **Model Selection**: Choose between different detection models
- **Custom Classes**: Add custom object classes
- **Region of Interest**: Define detection zones per camera
- **Schedule**: Configure detection schedules

### 3. Tracking Settings
- **Algorithm Selection**: Choose tracking algorithm (SORT, DeepSORT, etc.)
- **Advanced Tuning**: More granular tracking parameters
- **Track Filtering**: Filter tracks by duration, distance, etc.

### 4. Storage Settings
- **Cloud Storage**: Configure cloud backup options
- **Compression**: Configure video compression settings
- **Cleanup Rules**: Advanced retention policies
- **Storage Monitoring**: Real-time storage usage display

### 5. Network Settings
- **SSL/TLS**: Configure HTTPS and secure WebSocket
- **Authentication**: Configure API authentication
- **CORS**: Configure CORS settings
- **Rate Limiting**: Configure rate limiting

### 6. UI Improvements
- **Search**: Search within settings
- **Favorites**: Mark frequently used settings
- **Tooltips**: More detailed tooltips
- **Keyboard Shortcuts**: Quick access to common actions
- **Dark Mode**: Respect system theme preference

## Known Limitations

1. **No Real-time Validation**: Changes are validated only on save
2. **No Undo/Redo**: Only reset to last saved state
3. **No Conflict Resolution**: Last save wins (no merge conflicts)
4. **No Partial Saves**: Must save entire configuration
5. **No Change History**: No audit log of configuration changes

## Dependencies

- **@mantine/core**: UI components
- **@mantine/notifications**: Toast notifications
- **@mantine/modals**: Modal dialogs
- **@tabler/icons-react**: Icons
- **zustand**: State management

## Related Files

- `src/app/pages/Settings/index.tsx` - Main component
- `src/app/store/settingsSlice.ts` - State management
- `src/app/services/api.ts` - API client
- `src/app/services/schema.ts` - TypeScript types

## API Endpoints

- `GET /api/config` - Fetch configuration
- `PUT /api/config` - Update configuration

## Conclusion

The Settings page provides a comprehensive, user-friendly interface for configuring the BOB camera system. It follows best practices for form design, validation, and user feedback, while maintaining consistency with the rest of the application.

The draft/save workflow ensures users can experiment with settings without fear of breaking the system, while the clear visual feedback and validation help prevent configuration errors.

Future enhancements will focus on advanced features like presets, import/export, and more granular control over detection and tracking parameters.