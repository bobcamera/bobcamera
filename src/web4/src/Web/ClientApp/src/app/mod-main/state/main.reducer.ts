import { HttpErrorResponse } from '@angular/common/http';
import { createReducer, createFeatureSelector, createSelector, on } from '@ngrx/store';

import * as fromRoot from '../../state'
import * as AppActions from './main.actions'

import { NotificationModel } from '../models'

export const featureKey = 'main';

export interface MainState {
  error: HttpErrorResponse;
  notification: NotificationModel;
}

export interface State extends fromRoot.State {
  [featureKey]: MainState;
}

export const initialState: MainState = {
  error: null,
  notification: null,
};

export const mainReducer = createReducer<MainState>(
    initialState,
    on(AppActions.ClearState, () => initialState),

    on(AppActions.Notification, (state, { notification }) => ({ ...state, notification: notification })),
    on(AppActions.ClearNotification, (state) => ({ ...state, notification: null })),

    on(AppActions.Error, (state, { error }) => ({ ...state, error: error })),
    on(AppActions.ClearError, (state) => ({ ...state, error: null })),
  );

// SELECTORS
export const selectCoreState = createFeatureSelector<MainState>(featureKey);

export const getError = createSelector(selectCoreState, (state: MainState) => state.error);
export const getNotification = createSelector(selectCoreState, (state: MainState) => state.notification);