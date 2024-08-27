import { Action, createReducer, on } from '@ngrx/store';

import { SettingsState, NIGHT_MODE_THEME } from './settings.model';

import {
  actionSettingsChangeAnimationsElements,
  actionSettingsChangeAnimationsPage,
  actionSettingsChangeAnimationsPageDisabled,
  actionSettingsChangeAutoNightMode,
  actionSettingsChangeHour,
  actionSettingsChangeLanguage,
  actionSettingsChangeStickyHeader,
  actionSettingsChangeTheme,
  actionSettingsChangeRosPort,
  actionSettingsChangeRosUrl
} from './settings.actions';

export const initialState: SettingsState = {
  language: 'en',
  theme: 'DEFAULT-THEME',
  autoNightMode: false,
  nightTheme: NIGHT_MODE_THEME,
  stickyHeader: true,
  pageAnimations: false,
  pageAnimationsDisabled: true,
  elementsAnimations: false,
  hour: 0,
  rosModel: { url: 'http://localhost', port: 9090}
};

export const settingsReducer = createReducer<SettingsState>(
  initialState,
  on(
    actionSettingsChangeLanguage,
    actionSettingsChangeTheme,
    actionSettingsChangeAutoNightMode,
    actionSettingsChangeStickyHeader,
    actionSettingsChangeAnimationsPage,
    actionSettingsChangeAnimationsElements,
    actionSettingsChangeHour,
    (state, action) => ({ ...state, ...action })
  ),

  on(
    actionSettingsChangeRosPort,
    (state, { rosPort }) => ({
      ...state,
      rosModel: { ...state.rosModel, port: rosPort }
    })
  ),

  on(
    actionSettingsChangeRosUrl,
    (state, { rosUrl }) => ({
      ...state,
      rosModel: { ...state.rosModel, url: rosUrl }
    })
  ),

  on(
    actionSettingsChangeAnimationsPageDisabled,
    (state, { pageAnimationsDisabled }) => ({
      ...state,
      pageAnimationsDisabled,
      pageAnimations: false
    })
  )
);