import { createReducer, on } from '@ngrx/store';

import * as GuiActions from './gui.actions';

import { GuiState } from './gui.model';

export const initialState: GuiState = {
  menuDrawerExpanded: false, 
};

export const guiReducer = createReducer<GuiState>(
    initialState,
    on(GuiActions.ClearState, () => initialState),

    on(GuiActions.MenuDrawerToggle, (state) => ({ ...state, menuDrawerExpanded: !state.menuDrawerExpanded })),
  );