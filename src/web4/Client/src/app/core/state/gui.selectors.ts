import { createSelector } from '@ngrx/store';

import { GuiState } from './gui.model';

// SELECTORS

import { selectCoreState } from './';

export const selectGuiState = createSelector(
  selectCoreState,
  (state) => state.gui
);

export const getMenuDrawerExpanded = createSelector(selectGuiState, (state: GuiState) => state.menuDrawerExpanded);