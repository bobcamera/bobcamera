import { Injectable } from '@angular/core';
import { HttpErrorResponse } from '@angular/common/http';
import { Actions, ofType, createEffect } from '@ngrx/effects';
import { of } from 'rxjs';
import { map, switchMap, catchError, filter } from 'rxjs/operators';

import { ErrorService } from '../services';

import * as AppActions from './shared.actions'

@Injectable()
export class GuiEffects {

  // https://blog.angular-university.io/debug-rxjs/

  constructor(private _actions$: Actions, private errorService: ErrorService) {
  }
}