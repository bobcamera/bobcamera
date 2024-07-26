import { createAction, props } from '@ngrx/store';

export const setHeading = createAction('[Recordings] Set Heading', props<{ heading: string }>());
export const navPanelExpanded = createAction('[Recordings] Navigation Panel Expanded', props<{ expanded: boolean }>());

export const setMessage = createAction('[Recordings] Set Message', props<{ message: string }>());
export const clearMessage = createAction('[Recordings] Clear Message');