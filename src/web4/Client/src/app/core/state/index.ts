import { createFeatureSelector, combineReducers, Action, } from '@ngrx/store';
import * as fromRouter from '@ngrx/router-store';

import { AppState } from '../../state';

import { SharedState } from './shared.model';
import { sharedReducer } from './shared.reducer';

import { GuiState } from './gui.model';
import { guiReducer } from './gui.reducer';

import { SettingsState } from './settings.model';
import { settingsReducer } from './settings.reducer';

/**
 * Router Selectors
 */
export const { selectRouteData } = fromRouter.getRouterSelectors();

export interface CoreState {
    shared: SharedState;
    gui: GuiState;
    settings: SettingsState;
}

export interface State extends AppState {
  core: CoreState;
}

export function coreReducers(state: CoreState | undefined, action: Action) {
  return combineReducers({
    shared: sharedReducer,
    gui: guiReducer,
    settings: settingsReducer,
  })(state, action);
}

/**
 * Core Selectors
 */ 

export const selectCoreState = createFeatureSelector<CoreState>('core');