import { ActionReducer } from '@ngrx/store';

import { State } from '../core/state';

export function debug(reducer: ActionReducer<State>): ActionReducer<State> {
  return function (state, action) {
    const newState = reducer(state, action);
    console.log(`[DEBUG] action: ${action.type}`, {
      payload: (<any>action).payload,
      oldState: state,
      newState
    });
    return newState;
  };
}

export function logger(reducer: ActionReducer<State>): ActionReducer<State> {
    return (state, action) => {
      const result = reducer(state, action);
      console.groupCollapsed(action.type);
      console.log('[DEBUG] prev state', state);
      console.log('[DEBUG] action', action);
      console.log('[DEBUG] next state', result);
      console.groupEnd();
  
      return result;
    };
  }