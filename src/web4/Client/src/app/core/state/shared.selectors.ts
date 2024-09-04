import { createSelector, on } from '@ngrx/store';

import { selectCoreState } from './';
import { SharedState } from './shared.model';

// SELECTORS

export const selectSharedState = createSelector(
  selectCoreState,
  (state) => state.shared
);

export const getError = createSelector(selectSharedState, (state: SharedState) => state.error);
export const getNotification = createSelector(selectSharedState, (state: SharedState) => state.notification);