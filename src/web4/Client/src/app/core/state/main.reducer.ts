import { HttpErrorResponse } from '@angular/common/http';
import { createReducer, createFeatureSelector, createSelector, on } from '@ngrx/store';

import * as fromRoot from '../../state'
import * as AppActions from './main.actions'

import { NotificationModel } from '../models'

export const NIGHT_MODE_THEME = 'BLACK-THEME';

export const featureKey = 'main';

export interface MainState {
  error: HttpErrorResponse;
  notification: NotificationModel;

  language: string;
  theme: string;
  autoNightMode: boolean;
  nightTheme: string;
  stickyHeader: boolean;
  pageAnimations: boolean;
  pageAnimationsDisabled: boolean;
  elementsAnimations: boolean;
  hour: number;  

  menuDrawerExpanded: boolean;
}

export interface State extends fromRoot.State {
  [featureKey]: MainState;
}

export const initialState: MainState = {
  error: null,
  notification: null,

  language: 'en',
  theme: 'DEFAULT-THEME',
  autoNightMode: false,
  nightTheme: NIGHT_MODE_THEME,
  stickyHeader: true,
  pageAnimations: true,
  pageAnimationsDisabled: false,
  elementsAnimations: true,
  hour: 0,

  menuDrawerExpanded: false,
};

export const mainReducer = createReducer<MainState>(
    initialState,
    on(AppActions.ClearState, () => initialState),

    on(AppActions.Notification, (state, { notification }) => ({ ...state, notification: notification })),
    on(AppActions.ClearNotification, (state) => ({ ...state, notification: null })),

    on(AppActions.MenuDrawerToggle, (state) => ({ ...state, menuDrawerExpanded: !state.menuDrawerExpanded })),

    on(AppActions.Error, (state, { error }) => ({ ...state, error: error })),
    on(AppActions.ClearError, (state) => ({ ...state, error: null })),
  );

// SELECTORS
export const selectCoreState = createFeatureSelector<MainState>(featureKey);

export const getMenuDrawerExpanded = createSelector(selectCoreState, (state: MainState) => state.menuDrawerExpanded);

export const getError = createSelector(selectCoreState, (state: MainState) => state.error);
export const getNotification = createSelector(selectCoreState, (state: MainState) => state.notification);

export const selectSettingsLanguage = createSelector(
  selectCoreState,
  (state: MainState) => state.language
);

export const selectTheme = createSelector(
  selectCoreState,
  (settings) => settings.theme
);

export const selectPageAnimations = createSelector(
  selectCoreState,
  (settings) => settings.pageAnimations
);

export const selectElementsAnimations = createSelector(
  selectCoreState,
  (settings) => settings.elementsAnimations
);

export const selectAutoNightMode = createSelector(
  selectCoreState,
  (settings) => settings.autoNightMode
);

export const selectNightTheme = createSelector(
  selectCoreState,
  (settings) => settings.nightTheme
);

export const selectHour = createSelector(
  selectCoreState,
  (settings) => settings.hour
);

export const selectIsNightHour = createSelector(
  selectAutoNightMode,
  selectHour,
  (autoNightMode, hour) => autoNightMode && (hour >= 21 || hour <= 7)
);

export const selectEffectiveTheme = createSelector(
  selectTheme,
  selectNightTheme,
  selectIsNightHour,
  (theme, nightTheme, isNightHour) =>
    (isNightHour ? nightTheme : theme).toLowerCase()
);