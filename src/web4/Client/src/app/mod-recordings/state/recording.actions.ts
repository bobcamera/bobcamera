import { HttpErrorResponse } from '@angular/common/http';

import { createAction, props } from '@ngrx/store';
import { RecordingQuery, RecordingDto } from '../models';

export const setHeading = createAction('[Recordings] Set Heading', props<{ heading: string }>());

export const menuPanelToggle = createAction('[Recordings] menuPanelToggle');
export const contextPanelToggle = createAction('[Recordings] contextPanelToggle');

export const setMessage = createAction('[Recordings] Set Message', props<{ message: string }>());
export const clearMessage = createAction('[Recordings] Clear Message');

// Recordings
export const getRecordings = createAction('[Recordings] Get Recordings', props<{ query: RecordingQuery }>());
export const getRecordingsSuccess = createAction('[Recordings] Get Recordings Success', props<{ data: RecordingDto[] }>());

export const deleteRecording = createAction('[Recordings] Delete Recording', props<{ data: RecordingDto }>());
export const deleteRecordingSuccess = createAction('[Recordings] Delete Recording Success', props<{ data: RecordingDto }>());

export const Error = createAction('[Recordings] Error', props<{ error: HttpErrorResponse }>());
export const ClearError = createAction('[Recordings] Clear Error');