import { Injectable } from '@angular/core';
import { HttpErrorResponse, HttpResponse } from '@angular/common/http';
import { Actions, createEffect, ofType } from '@ngrx/effects';
import { of } from 'rxjs';
import { mergeMap, map, debounceTime, switchMap, catchError, filter } from 'rxjs/operators';

import { ErrorService } from '../../../core/services';
import { SharedEffects } from '../../../core/state/shared.effects';

import { PlaybackService } from '../services';
import * as RecordingActions from './recording.actions';

import { RecordingQuery, RecordingDto } from '../models';

@Injectable()
export class RecordingEffects extends SharedEffects {

  constructor(errorService: ErrorService, private actions$: Actions, private playbackService: PlaybackService) { 
    super(actions$, errorService);
  }

  recordings$ = createEffect(() =>
    this.actions$.pipe(
      ofType(RecordingActions.getRecordings),
      map((action) => action.query),
      switchMap((query: RecordingQuery) =>
        this.playbackService.recordings(query).pipe(
            map((data: RecordingDto[]) => RecordingActions.getRecordingsSuccess({ data: data })),
            catchError((error: HttpErrorResponse) => { 
                this.processError(error);
                return of(RecordingActions.Error({ error }));
            })
        )
      )
    )
  );

  deleteRecording$ = createEffect(() =>
    this.actions$.pipe(
        ofType(RecordingActions.deleteRecording),
        map((action) => action.data),
        switchMap((dto: RecordingDto) => {
            return this.playbackService.deleteRecording(dto).pipe(
                map(() => { 
                    //this._notificationService.Info('Work Item successfully deleted');
                    return RecordingActions.deleteRecordingSuccess({data: dto})
                }),
                catchError((error: HttpErrorResponse) => { 
                    this.processError(error);
                    return of(RecordingActions.Error({ error }));
                }))
            }
        )
    )
  );
}