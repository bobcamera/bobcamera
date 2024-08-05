import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store } from '@ngrx/store';
import { Subject, Observable } from 'rxjs';
import { map, switchMap } from 'rxjs/operators';

import * as VisionActions from '../../state/vision.actions';
import { VisionState, getVisionCamera } from '../../state/vision.reducer';

import { CameraDto, AppStateDto } from '../../models';

import { BobRosService } from '../../services';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss']
})
export class TestComponentComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();
  _cameraDetails$: Observable<CameraDto>;
  _visionData$: Observable<string>;
  _stateData$: Observable<AppStateDto>;

  constructor(private store: Store<VisionState>, public rosSvc: BobRosService) {
  }

  ngOnInit(): void {
    this.rosSvc.connected.subscribe((connected: boolean) => {
      console.log(`Connection status to ROS Bridge: ${connected}`);
      if (connected) {
        this.rosSvc.Subscribe();
      }
    });

    this._visionData$ = this.rosSvc.annotated;
    this._stateData$ = this.rosSvc.appState;

    this.rosSvc.connect(true);

    this.store.dispatch(VisionActions.setHeading({ heading: 'Vision Test Component' }));
    this._cameraDetails$ = this.store.select(getVisionCamera);

    /*this._visionData$.subscribe((data: string) => {
      console.log(`_visionData$: ${data}`);
    });*/
  }

  /*onOpenedChange(opened: boolean) {
    this.store.dispatch(VisionActions.setCameraPolling({ enabled: opened }));
  }*/

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();

    this.store.dispatch(VisionActions.setCameraPolling({ enabled: false }));
  }
}
