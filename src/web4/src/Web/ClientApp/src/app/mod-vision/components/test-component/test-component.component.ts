import { Component, OnDestroy, OnInit, ChangeDetectionStrategy } from '@angular/core';
import { ActivatedRoute } from '@angular/router';
import { Store } from '@ngrx/store';
import { Subject, Observable } from 'rxjs';
import { map, switchMap, distinctUntilChanged } from 'rxjs/operators';

import * as MainActions from '../../../mod-main/state/main.actions';
import { NotificationType } from '../../../mod-main/models';

import * as VisionActions from '../../state/vision.actions';
import { VisionState, getVisionCamera } from '../../state/vision.reducer';

import { CameraDto, AppStateDto } from '../../models';

import { BobRosService, SubscriptionType } from '../../services';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class TestComponentComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();
  _cameraDetails$: Observable<CameraDto>;
  _visionData$: Observable<string>;
  _stateData$: Observable<AppStateDto>;
  _displayType: string;

  constructor(private store: Store<VisionState>, public rosSvc: BobRosService, 
    private route: ActivatedRoute) {
  }

  ngOnInit(): void {
    this.rosSvc.connected.pipe(distinctUntilChanged()).subscribe((connected: boolean) => {
      console.log(`Connection status to ROS Bridge: ${connected}`);
      if (connected) {
        this.onDisplayTypeChanged(this._displayType);
        this.store.dispatch(MainActions.Notification({
          notification: { type: NotificationType.Information, message: "Connected to ROS Bridge." }}));
      } else {
        this.store.dispatch(MainActions.Notification({
          notification: { type: NotificationType.Error, message: "Connection lost to ROS Bridge." }}));
      }
    });

    this.rosSvc.connect(true);

    this.store.dispatch(VisionActions.setHeading({ heading: 'Vision Test Component' }));
    this._cameraDetails$ = this.store.select(getVisionCamera);

    /*this._visionData$.subscribe((data: string) => {
      console.log(`_visionData$: ${data}`);
    });*/

    this.route.params.pipe(
      map(params => params.type),
      distinctUntilChanged(),
    ).subscribe(changedParam => {
      console.log(`DisplayType: ${this._displayType}`);
      this.onDisplayTypeChanged(changedParam)
    });
  }

  onDisplayTypeChanged(type: string) {
    this._displayType = type;
    switch(type)
    {     
      case 'annotated':
        this._visionData$ = this.rosSvc.videoStream(SubscriptionType.Annotated);
        //this._displayType = 'Annotated Frame';
        break;
      case 'foregroundmask':
        this._visionData$ = this.rosSvc.videoStream(SubscriptionType.ForegroundMask);
        //this._displayType = 'ForegroundMask Frame';
        break;
      case 'privacymask':
        this._visionData$ = this.rosSvc.videoStream(SubscriptionType.PrivacyMask);
        //this._displayType = 'PrivacyMask Frame';
        break;
      case 'detectionmask':
        this._visionData$ = this.rosSvc.videoStream(SubscriptionType.DetectionMask);
        //this._displayType = 'DetectionMask Frame';
        break;
    }
  }

  onOpenedChange(opened: boolean) {
    if (this.rosSvc.isConnected) {
      if (opened) {
        this._stateData$ = this.rosSvc.appState(true);
      } else {
        this._stateData$ = this.rosSvc.appState(false);
      }
    }
    //this.store.dispatch(VisionActions.setCameraPolling({ enabled: opened }));
  }

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();

    //this.store.dispatch(VisionActions.setCameraPolling({ enabled: false }));

    this.rosSvc.disconnect();
  }
}
