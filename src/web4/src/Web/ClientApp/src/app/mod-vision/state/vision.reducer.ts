import { createFeatureSelector, createReducer, createSelector, on } from '@ngrx/store';
import { HttpErrorResponse } from '@angular/common/http';

import * as fromRoot from '../../state'
import * as VisionActions from './vision.actions'

import { LoadingModel, CameraDto, AppInfoDto } from '../models';

export const featureKey = 'vision';

export interface VisionState {
  heading: string;
  navPanelExpanded: boolean;
  message: string;

  error: HttpErrorResponse;
  loading: boolean;
  loadingMessage: string;

  enableCameraPolling: boolean;
  camera: CameraDto;

  bobInfo: AppInfoDto;

  maskEditMode: boolean;
}

export interface State extends fromRoot.State {
  [featureKey]: VisionState;
}

const initialState: VisionState = {
  heading: 'Vision Component',
  navPanelExpanded: true,
  message: '',

  error: null,
  loading: false,
  loadingMessage: '',
  
  enableCameraPolling: false,
  camera: null,

  bobInfo: null,

  maskEditMode: false,
};


export const visionReducer = createReducer<VisionState>(
  initialState,

  on(VisionActions.setHeading, (state, action): VisionState => { return { ...state, heading: action.heading }; }),

  on(VisionActions.navPanelExpanded, (state, action): VisionState => { return { ...state, navPanelExpanded: action.expanded }; }),

  on(VisionActions.setMessage, (state, action): VisionState => { return { ...state, message: action.message }; }),
  on(VisionActions.clearMessage, (state, action): VisionState => { return { ...state, message: '' }; }),

  on(VisionActions.setCameraPolling, (state, action): VisionState => { return { ...state, enableCameraPolling: action.enabled }; }),

  on(VisionActions.getCameraDetails, (state, action): VisionState => { return { ...state, loading: true, loadingMessage: 'Loading Camera Details' }; }),
  on(VisionActions.getCameraDetailsSuccess, (state, action): VisionState => { return { ...state, loading: false, error: null, loadingMessage: null, camera: action.data }; }),

  on(VisionActions.setBobInfo, (state, action): VisionState => { return { ...state, bobInfo: action.info }; }),

  on(VisionActions.setMaskEditMode, (state, action): VisionState => { return { ...state, maskEditMode: action.enabled }; }),
)

// Selectors
const getVisionState = createFeatureSelector<VisionState>(featureKey);

export const getVisionHeading = createSelector(
  getVisionState,
  state => state.heading
);

export const getVisionNavPanelExpanded = createSelector(
  getVisionState,
  state => state.navPanelExpanded
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