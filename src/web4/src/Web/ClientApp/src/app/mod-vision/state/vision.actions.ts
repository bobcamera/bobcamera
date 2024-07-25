import { createAction, props } from '@ngrx/store';

export const setHeading = createAction('[Vision] Set Heading', props<{ heading: string }>());
export const navPanelExpanded = createAction('[Vision] Navigation Panel Expanded', props<{ expanded: boolean }>());

export const setMessage = createAction('[Vision] Set Message', props<{ message: string }>());
export const clearMessage = createAction('[Vision] Clear Message');