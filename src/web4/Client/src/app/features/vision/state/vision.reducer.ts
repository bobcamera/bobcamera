import { createFeatureSelector, createReducer, createSelector, on } from '@ngrx/store';
import { HttpErrorResponse } from '@angular/common/http';

import * as fromRoot from '../../../state'
import * as VisionActions from './vision.actions'

import { VisionState } from'./vision.models';

import { LoadingModel, CameraDto, AppInfoDto, ImageStreamTypeEnum } from '../models';

export const featureKey = 'vision';

export interface State extends fromRoot.AppState {
  [featureKey]: VisionState;
}

const initialState: VisionState = {
  heading: 'Vision Component',
  menuPanelExpanded: false,
  contextPanelExpanded: false,
  imageStreamType: ImageStreamTypeEnum.Annotated,
  displayPrivacyMaskControls: false,
  displayDetectionMaskControls: false,
  displayAppState: true,
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

  on(VisionActions.setImageStreamType, (state, action): VisionState => { 

    let imageStreamType: ImageStreamTypeEnum = ImageStreamTypeEnum.Annotated;
    let displayPrivacyMaskControls = false;
    let displayDetectionMaskControls = false;
    let displayAppState = true;
    let heading = '';

    switch(action.imageStreamType)
    {     
      case 'annotated':
        imageStreamType = ImageStreamTypeEnum.Annotated;
        heading = 'Annotated Image Stream';
        displayPrivacyMaskControls = false;
        displayDetectionMaskControls = false;
        displayAppState = true;
        break;
      case 'foregroundmask':
        imageStreamType = ImageStreamTypeEnum.ForegroundMask;
        heading = 'Foreground Mask Image Stream';
        displayPrivacyMaskControls = false;
        displayDetectionMaskControls = false;
        displayAppState = true;
        break;
      case 'privacymask':
        imageStreamType = ImageStreamTypeEnum.PrivacyMask;
        heading = 'Privacy Mask Image Stream';
        displayPrivacyMaskControls = true;
        displayDetectionMaskControls = false;
        displayAppState = false;
        break;
      case 'detectionmask':
        imageStreamType = ImageStreamTypeEnum.DetectionMask;
        heading = 'Detection Mask Image Stream';
        displayPrivacyMaskControls = false;
        displayDetectionMaskControls = true;
        displayAppState = false;
        break;
    }

    return { 
      ...state, 
      imageStreamType: imageStreamType,
      displayPrivacyMaskControls: displayPrivacyMaskControls,
      displayDetectionMaskControls: displayDetectionMaskControls,
      displayAppState: displayAppState,
      heading: heading
    };
  }),

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