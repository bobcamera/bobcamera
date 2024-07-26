import { Injectable } from '@angular/core';
import { Actions } from '@ngrx/effects';

import { ErrorService } from '../../mod-main/services';
import { MainEffects } from '../../mod-main/state';

import * as RecordingActions from './recording.actions';

@Injectable()
export class RecordingEffects extends MainEffects {

  constructor(errorService: ErrorService, private actions$: Actions) { 
    super(errorService);
  }
}