import { Injectable } from '@angular/core';
import { HttpErrorResponse } from '@angular/common/http';
import { Actions, ofType, createEffect } from '@ngrx/effects';
import { map, switchMap, filter, debounceTime } from 'rxjs/operators';

import { ErrorService } from '../services';

import * as SharedActions from './shared.actions'

@Injectable()
export class SharedEffects {

  // https://blog.angular-university.io/debug-rxjs/

  constructor(private _actions$: Actions, private errorService: ErrorService) {
  }

  notification$ = createEffect(() =>
    this._actions$.pipe(
        ofType(SharedActions.Notification),
        map((action) => action.notification),
        filter(notification => !!notification),
        filter(notification => !!notification.message),
        //debounceTime(1000),
        switchMap(() => [SharedActions.ClearNotification()])
    )
  );

  error$ = createEffect(() =>
    this._actions$.pipe(
        ofType(SharedActions.Error),
        map((action) => action.error),
        filter(error => !!error),        
        switchMap(() => [SharedActions.ClearError()])
    )
  );

  protected processError(error: HttpErrorResponse, notify: boolean = true): void {

    console.error(`Error during server communications. Code: ${error.status}. Error ${error.message}`);    

    switch (error.status) {
        case 401:
        case 403:
          //this._routerService.toAccessDenied();
        break;
        default:
          if (notify) {
            this.errorService.HandleError('Error during server communications', error);
          }
          break;
    }
  } 
}