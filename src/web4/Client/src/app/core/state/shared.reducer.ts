import { createReducer, on } from '@ngrx/store';

import * as fromRoot from '../../state';
import * as AppActions from './shared.actions';

import { SharedState } from './shared.model';
import { GuiState } from './gui.model';
import { SettingsState } from './settings.model';

export const NIGHT_MODE_THEME = 'BLACK-THEME';

export const initialState: SharedState = {
  error: null,
  notification: null,
};

export const sharedReducer = createReducer<SharedState>(
    initialState,
    on(AppActions.ClearState, () => initialState),

    on(AppActions.Notification, (state, { notification }) => ({ ...state, notification: notification })),
    on(AppActions.ClearNotification, (state) => ({ ...state, notification: null })),

    on(AppActions.Error, (state, { error }) => ({ ...state, error: error })),
    on(AppActions.ClearError, (state) => ({ ...state, error: null })),
  );