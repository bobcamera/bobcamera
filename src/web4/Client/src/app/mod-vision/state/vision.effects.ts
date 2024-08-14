import { Injectable } from '@angular/core';
import { HttpErrorResponse, HttpResponse } from '@angular/common/http';
import { Actions, createEffect, ofType } from '@ngrx/effects';
import { Store, Action } from '@ngrx/store';
import { of, timer } from 'rxjs';
import { withLatestFrom, mergeMap, map, debounceTime, switchMap, catchError, filter } from 'rxjs/operators';

import { ErrorService } from '../../mod-main/services';
import { MainEffects } from '../../mod-main/state';

import { VisionState, getVisionCameraPollingEnabled } from './vision.reducer';
import { VisionService } from '../services';
import * as VisionActions from './vision.actions';

import { CameraQuery, CameraDto } from '../models';

@Injectable()
export class VisionEffects extends MainEffects {

  constructor(errorService: ErrorService, private store: Store<VisionState>, private actions$: Actions, 
    private visionService: VisionService) { 
    super(actions$, errorService);
  }

  CAMERA_POLL_INTERVAL = 1 * 1000; // every second

  CameraPolling$ = createEffect(() =>
    this.actions$.pipe(
      withLatestFrom(this.store.select(getVisionCameraPollingEnabled)),
      filter(([action, enabled]) => enabled),
      switchMap(([action, state]) => timer(this.CAMERA_POLL_INTERVAL)),
      map(() => VisionActions.getCameraDetails({query: {cameraName: 'Camera Auto Retrieval'}}))
    )
  );

  cameraDetails$ = createEffect(() =>
    this.actions$.pipe(
      ofType(VisionActions.getCameraDetails),
      map((action) => action.query),
      switchMap((query: CameraQuery) =>
        this.visionService.cameraDetails(query).pipe(
            map((data: CameraDto) => VisionActions.getCameraDetailsSuccess({ data: data })),
            catchError((error: HttpErrorResponse) => { 
                this.processError(error);
                return of(VisionActions.Error({ error }));
            })
        )
      )
    )
  );
}