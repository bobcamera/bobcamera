import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store } from '@ngrx/store';
import { Subject, Observable } from 'rxjs';

import * as VisionActions from '../../state/vision.actions';
import { VisionState, getVisionCamera } from '../../state/vision.reducer';

import { CameraDto } from '../../models';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss']
})
export class TestComponentComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();
  _cameraDetails$: Observable<CameraDto>;

  constructor(private store: Store<VisionState>) {
  }

  ngOnInit(): void {
    this.store.dispatch(VisionActions.setHeading({ heading: 'Vision Test Component' }));
    this._cameraDetails$ = this.store.select(getVisionCamera);
  }

  onOpenedChange(opened: boolean) {
    this.store.dispatch(VisionActions.setCameraPolling({ enabled: opened }));
  }


  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();

    this.store.dispatch(VisionActions.setCameraPolling({ enabled: false }));
  }
}
