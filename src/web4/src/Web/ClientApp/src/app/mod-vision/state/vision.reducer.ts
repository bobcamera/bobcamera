import { createFeatureSelector, createReducer, createSelector, on } from '@ngrx/store';
import { HttpErrorResponse } from '@angular/common/http';

import * as fromRoot from '../../state'
import * as VisionActions from './vision.actions'

import { LoadingModel, CameraDto, AppInfoDto } from '../models';

export const featureKey = 'vision';

export interface VisionState {
  heading: string;
  menuPanelExpanded: boolean;
  contextPanelExpanded: boolean;
  message: string;

  error: HttpErrorResponse;
  loading: boolean;
  loadingMessage: string;

  enableCameraPolling: boolean;
  camera: CameraDto;

  bobInfo: AppInfoDto;

  maskEditMode: boolean;
  maskSvg: string;
}

export interface State extends fromRoot.State {
  [featureKey]: VisionState;
}

const initialState: VisionState = {
  heading: 'Vision Component',
  menuPanelExpanded: false,
  contextPanelExpanded: false,
  message: '',

  error: null,
  loading: false,
  loadingMessage: '',
  
  enableCameraPolling: false,
  camera: null,

  bobInfo: null,

  maskEditMode: false,
  maskSvg: null
};


export const visionReducer = createReducer<VisionState>(
  initialState,

  on(VisionActions.setHeading, (state, action): VisionState => { return { ...state, heading: action.heading }; }),

  on(VisionActions.menuPanelToggle, (state) => ({ ...state, menuPanelExpanded: !state.menuPanelExpanded })),
  on(VisionActions.contextPanelToggle, (state) => ({ ...state, contextPanelExpanded: !state.contextPanelExpanded })),

  on(VisionActions.setMessage, (state, action): VisionState => { return { ...state, message: action.message }; }),
  on(VisionActions.clearMessage, (state, action): VisionState => { return { ...state, message: '' }; }),

  on(VisionActions.setCameraPolling, (state, action): VisionState => { return { ...state, enableCameraPolling: action.enabled }; }),

  on(VisionActions.getCameraDetails, (state, action): VisionState => { return { ...state, loading: true, loadingMessage: 'Loading Camera Details' }; }),
  on(VisionActions.getCameraDetailsSuccess, (state, action): VisionState => { return { ...state, loading: false, error: null, loadingMessage: null, camera: action.data }; }),

  on(VisionActions.setBobInfo, (state, action): VisionState => { return { ...state, bobInfo: action.info }; }),

  on(VisionActions.setMaskEditMode, (state, action): VisionState => { return { ...state, maskEditMode: action.enabled }; }),
  on(VisionActions.setMaskSvg, (state, action): VisionState => { return { ...state, maskSvg: action.mask }; }),
  on(VisionActions.clearMaskSvg, (state, action): VisionState => { return { ...state, maskSvg: null }; }),
)

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