import { Injectable } from '@angular/core';
import { Actions } from '@ngrx/effects';

import { ErrorService } from '../../mod-main/services';
import { MainEffects } from '../../mod-main/state';

import * as VisionActions from './vision.actions';

@Injectable()
export class VisionEffects extends MainEffects {

  constructor(errorService: ErrorService, private actions$: Actions) { 
    super(errorService);
  }
}