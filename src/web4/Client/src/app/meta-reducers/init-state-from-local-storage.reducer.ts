import { ActionReducer, INIT, UPDATE } from '@ngrx/store';

import { LocalStorageService } from '../core/services/local-storage.service';

import { AppState } from '../state';

export function initStateFromLocalStorage(reducer: ActionReducer<AppState>): ActionReducer<AppState> {
  return function (state, action) {
    const newState = reducer(state, action);
    if ([INIT.toString(), UPDATE.toString()].includes(action.type)) {
      let loadedState = LocalStorageService.loadInitialState();

      //console.log(`loadedState: ${JSON.stringify(loadedState)}`)      

      let hydratedState = { 
        ...newState,
          core:  {
            ...newState.core, 
            settings: {
              language: loadedState.settings.language,
              theme: loadedState.settings.theme,
              autoNightMode: loadedState.settings.autoNightMode,
              nightTheme: loadedState.settings.nightTheme,
              stickyHeader: loadedState.settings.stickyHeader,
              pageAnimations: loadedState.settings.pageAnimations,
              pageAnimationsDisabled: loadedState.settings.pageAnimationsDisabled,
              elementsAnimations: loadedState.settings.elementsAnimations,
              hour: loadedState.settings.hour,
              rosModel: {
                url: loadedState.settings.rosModel?.url ?? 'http://localhost',
                port: loadedState.settings.rosModel?.port ?? 9090
              }
            }
          }
        };

        //console.log(`hydratedState: ${JSON.stringify(hydratedState)}`)

      return hydratedState;
    }
    return newState;
  };
}
