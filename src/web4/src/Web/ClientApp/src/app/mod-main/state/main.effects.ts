import { Injectable } from '@angular/core';
import { HttpErrorResponse } from '@angular/common/http';
import { Actions, ofType, createEffect } from '@ngrx/effects';
import { of } from 'rxjs';
import { map, switchMap, catchError, filter } from 'rxjs/operators';

import { ErrorService } from '../services';

import * as AppActions from './main.actions'

@Injectable()
export class MainEffects {

  // https://blog.angular-university.io/debug-rxjs/

  constructor(private _actions$: Actions, private errorService: ErrorService) {
  }

  notification$ = createEffect(() =>
    this._actions$.pipe(
        ofType(AppActions.Notification),
        switchMap(() => [AppActions.ClearNotification()])
    )
  );

  error$ = createEffect(() =>
    this._actions$.pipe(
        ofType(AppActions.Error),
        switchMap(() => [AppActions.ClearError()])
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