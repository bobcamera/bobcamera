import { InjectionToken } from '@angular/core';
import { ActionReducerMap, ActionReducer, MetaReducer, Action } from '@ngrx/store';
import * as fromRouter from '@ngrx/router-store';

import { environment } from '../../environments/environment';

import * as fromMain from '../core/state'
import * as fromVision from '../features/vision/state'
import * as fromPlayback from '../features/playback/state'

import { RouterStateUrl } from '../core/services';

export interface State {  
  router: fromRouter.RouterReducerState<RouterStateUrl>;
  main: fromMain.MainState;
  vision: fromVision.VisionState;
  playback: fromPlayback.RecordingState;
}

export const rootReducers = new InjectionToken<ActionReducerMap<State, Action>>('Root reducers token', {
  factory: () => ({
    router: fromRouter.routerReducer,
    main: fromMain.mainReducer,
    vision: fromVision.visionReducer,
    playback: fromPlayback.recordingReducer,
  }),
});

export function logger(reducer: ActionReducer<State>): ActionReducer<State> {
  return (state, action) => {
    const result = reducer(state, action);
    console.groupCollapsed(action.type);
    console.log('prev state', state);
    console.log('action', action);
    console.log('next state', result);
    console.groupEnd();

    return result;
  };
}

export function clearState(reducer: ActionReducer<State>): ActionReducer<State> {
  return function (state, action) {
    if (action.type === '[Main] Clear State') {
      state = undefined;
    }
    return reducer(state, action);
  };
}

export function applicationInit(reducer: ActionReducer<State>): ActionReducer<State> {
  return function (state, action) {
    //console.log(`Init state - lookup data loadingCount: ${state?.core?.lookupdata?.loadingCount}`);
    //console.log(`Init state - lookup data init: ${state?.core?.lookupdata?.init}`);
    return reducer(state, action);
  };
}

export const metaReducers: MetaReducer<State>[] = !environment.production
  ? [logger, clearState, applicationInit]
  : [clearState, applicationInit];


/**
 * Router Selectors
 */
 export const { selectRouteData } = fromRouter.getRouterSelectors();