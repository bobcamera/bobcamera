import { createFeatureSelector, createSelector } from '@ngrx/store';

import { VisionState } from'./vision.models';
import { featureKey } from './vision.reducer';

import { LoadingModel } from '../models';

// Selectors
const getVisionState = createFeatureSelector<VisionState>(featureKey);

export const getVisionHeading = createSelector(
  getVisionState,
  state => state.heading
);

export const getVisionMenuPanelExpanded = createSelector(
  getVisionState,
  state => state.menuPanelExpanded
);

export const getVisionContextPanelExpanded = createSelector(
  getVisionState,
  state => state.contextPanelExpanded
);

export const getVisionImageStreamType = createSelector(
  getVisionState,
  state => state.imageStreamType
);

export const getVisionDisplayPrivacyMaskControls = createSelector(
  getVisionState,
  state => state.displayPrivacyMaskControls
);

export const getVisionDisplayDetectionMaskControls = createSelector(
  getVisionState,
  state => state.displayDetectionMaskControls
);

export const getVisionDisplayAppState = createSelector(
  getVisionState,
  state => state.displayAppState
);

export const getVisionMessage = createSelector(
  getVisionState,
  state => state.message
);

export const getVisionError = createSelector(
  getVisionState,
  state => state.error
);

export const getVisionLoading = createSelector(
  getVisionState,
  state => <LoadingModel>{loading: state.loading, message: state.loadingMessage}
);

export const getVisionCameraPollingEnabled = createSelector(
  getVisionState,
  state => state.enableCameraPolling
);

export const getVisionCamera = createSelector(
  getVisionState,
  state => state.camera
);

export const getBobInfo = createSelector(
  getVisionState,
  state => state.bobInfo
);

export const getMaskEditMode = createSelector(
  getVisionState,
  state => state.maskEditMode
);

export const getMaskSvg = createSelector(
  getVisionState,
  state => state.maskSvg
);