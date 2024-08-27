import { InjectionToken } from '@angular/core';
import { ActionReducerMap, ActionReducer, MetaReducer, Action } from '@ngrx/store';
import * as fromRouter from '@ngrx/router-store';

import { environment } from '../../environments/environment';

import * as fromCore from '../core/state';
import { clearState, logger, debug, initStateFromLocalStorage } from '../meta-reducers';

import * as fromVision from '../features/vision/state';
import * as fromPlayback from '../features/playback/state';

import { RouterStateUrl } from '../core/services';

export interface AppState {  
  router: fromRouter.RouterReducerState<RouterStateUrl>;
  core: fromCore.CoreState;
  vision: fromVision.VisionState;
  playback: fromPlayback.RecordingState;
}

export const appReducers = new InjectionToken<ActionReducerMap<AppState, Action>>('Root reducers token', {
  factory: () => ({
    router: fromRouter.routerReducer,
    core: fromCore.coreReducers,
    vision: fromVision.visionReducer,
    playback: fromPlayback.recordingReducer,
  }),
});

export const metaReducers: MetaReducer<AppState>[] = !environment.production
  ? [logger, initStateFromLocalStorage]
  : [initStateFromLocalStorage];


/**
 * Router Selectors
 */
 export const { selectRouteData } = fromRouter.getRouterSelectors();