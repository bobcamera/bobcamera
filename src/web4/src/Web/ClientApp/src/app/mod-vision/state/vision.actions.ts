import { HttpErrorResponse } from '@angular/common/http';
import { createAction, props } from '@ngrx/store';

import { CameraQuery, CameraDto } from '../models';


export const setHeading = createAction('[Vision] Set Heading', props<{ heading: string }>());
export const navPanelExpanded = createAction('[Vision] Navigation Panel Expanded', props<{ expanded: boolean }>());

export const setMessage = createAction('[Vision] Set Message', props<{ message: string }>());
export const clearMessage = createAction('[Vision] Clear Message');

// Camera
export const setCameraPolling = createAction('[Vision] Set Camera Polling', props<{ enabled: boolean }>());

export const getCameraDetails = createAction('[Vision] Get Camera Details', props<{ query: CameraQuery }>());
export const getCameraDetailsSuccess = createAction('[Vision] Get Camera Details Success', props<{ data: CameraDto }>());

export const Error = createAction('[Vision] Error', props<{ error: HttpErrorResponse }>());
export const ClearError = createAction('[Vision] Clear Error');