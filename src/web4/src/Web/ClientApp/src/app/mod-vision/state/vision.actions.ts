import { HttpErrorResponse } from '@angular/common/http';
import { createAction, props } from '@ngrx/store';

import { CameraQuery, CameraDto, AppInfoDto, ImageStreamTypeEnum } from '../models';

export const setHeading = createAction('[Vision] Set Heading', props<{ heading: string }>());

export const menuPanelToggle = createAction('[Vision] menuPanelToggle');
export const contextPanelToggle = createAction('[Vision] contextPanelToggle');

export const setImageStreamType = createAction('[Vision] Set Message', props<{ imageStreamType: string }>());

export const setMessage = createAction('[Vision] Set Message', props<{ message: string }>());
export const clearMessage = createAction('[Vision] Clear Message');

// Camera
export const setCameraPolling = createAction('[Vision] Set Camera Polling', props<{ enabled: boolean }>());

export const getCameraDetails = createAction('[Vision] Get Camera Details', props<{ query: CameraQuery }>());
export const getCameraDetailsSuccess = createAction('[Vision] Get Camera Details Success', props<{ data: CameraDto }>());

export const setBobInfo = createAction('[Vision] Set Bob Info', props<{ info: AppInfoDto }>());

export const Error = createAction('[Vision] Error', props<{ error: HttpErrorResponse }>());
export const ClearError = createAction('[Vision] Clear Error');

// Masking
export const setMaskEditMode = createAction('[Vision] Set MaskEdit Mode', props<{ enabled: boolean }>());

export const setMaskSvg = createAction('[Vision] Set Mask Svg', props<{ mask: string }>());
export const clearMaskSvg = createAction('[Vision] Clear Mask Svg');