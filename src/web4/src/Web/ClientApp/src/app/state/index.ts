import { InjectionToken } from '@angular/core';
import { ActionReducerMap, ActionReducer, MetaReducer, Action } from '@ngrx/store';
import * as fromRouter from '@ngrx/router-store';

import { environment } from '../../environments/environment';

import * as fromMain from '../mod-main/state'
import * as fromVision from '../mod-vision/state'
import * as fromRecording from '../mod-recordings/state'

import { RouterStateUrl } from '../mod-main/services';

export interface State {  
  router: fromRouter.RouterReducerState<RouterStateUrl>;
  main: fromMain.MainState;
  vision: fromVision.VisionState;
  recording: fromRecording.RecordingState;
}

export const rootReducers = new InjectionToken<ActionReducerMap<State, Action>>('Root reducers token', {
  factory: () => ({
    router: fromRouter.routerReducer,
    main: fromMain.mainReducer,
    vision: fromVision.visionReducer,
    recording: fromRecording.recordingReducer,
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