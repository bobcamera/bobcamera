import { ActionReducer } from '@ngrx/store';

import { State } from '../core/state';

export function clearState(reducer: ActionReducer<State>): ActionReducer<State> {
    return function (state, action) {
      if (action.type === '[Core] Clear State') {
        state = undefined;
      }
      return reducer(state, action);
    };
  }