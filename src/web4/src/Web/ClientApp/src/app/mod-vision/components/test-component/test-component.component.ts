import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, ViewChild } from '@angular/core';
import { ActivatedRoute } from '@angular/router';
import { Store } from '@ngrx/store';
import { Subject, Observable } from 'rxjs';
import { takeUntil, map, switchMap, distinctUntilChanged } from 'rxjs/operators';
import { MatDialog } from '@angular/material/dialog';

import * as MainActions from '../../../mod-main/state/main.actions';
import { NotificationType } from '../../../mod-main/models';

import { ConfirmationDialogComponent } from '../../../mod-main/components';

import * as VisionActions from '../../state/vision.actions';
import { VisionState, getVisionCamera, getBobInfo, getMaskEditMode } from '../../state/vision.reducer';

import { CameraDto, AppInfoDto, AppStateDto } from '../../models';

import { BobRosConnection, BobRosService, ImageStreamType } from '../../services';

import { MaskCreationComponent } from '../';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush,
  providers: [BobRosService]
})
export class TestComponentComponent implements OnInit, OnDestroy {

  _ngUnsubscribe$: Subject<void> = new Subject<void>();
  
  //_cameraDetails$: Observable<CameraDto>;
  _imageStream$: Observable<string>;
  _appState$: Observable<AppStateDto>;
  _appInfo$: Observable<AppInfoDto>;

  _maskEditMode$: Observable<boolean>;

  _imageStreamType: string;
  _displayPrivacyMaskControls: boolean;
  _displayDetectionMaskControls: boolean;
  _displayAppState: boolean;

  @ViewChild('privacymaskcreator', {static: false}) privacymaskcreator: MaskCreationComponent;

  constructor(private store: Store<VisionState>, private rosSvc: BobRosService, private _matDialog: MatDialog,
    private route: ActivatedRoute) {
  }

  ngOnInit(): void {

    this.route.params.pipe(
      map(params => params.type),
      distinctUntilChanged(),
    ).subscribe(changedParam => {
      console.log(`ImageStreamType: ${this._imageStreamType}`);
      this.onDisplayTypeChanged(changedParam)
    });    

    this.rosSvc.connected
    .pipe(distinctUntilChanged())
    .subscribe((connected: boolean) => {
      console.log(`Connection status to ROS Bridge: ${connected}`);
      if (connected) {
       
        this.store.dispatch(MainActions.Notification({
          notification: { type: NotificationType.Information, message: "Connected to ROS Bridge." }}));

          this.onDisplayTypeChanged(this._imageStreamType);

          let connection = new BobRosConnection();
          connection.open(false);          

          try {
            if (connection.isConnected) {
              this.rosSvc.svcAppInfo(connection);
            }
          } finally {
            connection.close();
          }          
      } else {
        this.store.dispatch(MainActions.Notification({
          notification: { type: NotificationType.Error, message: "Connection lost to ROS Bridge." }}));
      }
    });

    this.rosSvc.connect(true);

    this.store.dispatch(VisionActions.setHeading({ heading: 'Image Stream Component' }));
    //this._cameraDetails$ = this.store.select(getVisionCamera);
    this._appInfo$ = this.store.select(getBobInfo);
    this._maskEditMode$ = this.store.select(getMaskEditMode);
  }

  onDisplayTypeChanged(type: string) {
    this._imageStreamType = type;
    switch(type)
    {     
      case 'annotated':
        this._imageStream$ = this.rosSvc.subVideoStream(ImageStreamType.Annotated);
        this.store.dispatch(VisionActions.setHeading({ heading: 'Annotated Image Stream' }));
        this._displayPrivacyMaskControls = false;
        this._displayDetectionMaskControls = false;
        this._displayAppState = true;
        break;
      case 'foregroundmask':
        this._imageStream$ = this.rosSvc.subVideoStream(ImageStreamType.ForegroundMask);
        this.store.dispatch(VisionActions.setHeading({ heading: 'Foreground Mask Image Stream' }));
        this._displayPrivacyMaskControls = false;
        this._displayDetectionMaskControls = false;
        this._displayAppState = true;
        break;
      case 'privacymask':
        this._imageStream$ = this.rosSvc.subVideoStream(ImageStreamType.PrivacyMask);
        this.store.dispatch(VisionActions.setHeading({ heading: 'Privacy Mask Image Stream' }));
        this._displayPrivacyMaskControls = true;
        this._displayDetectionMaskControls = false;
        this._displayAppState = false;
        break;
      case 'detectionmask':
        this._imageStream$ = this.rosSvc.subVideoStream(ImageStreamType.DetectionMask);
        this.store.dispatch(VisionActions.setHeading({ heading: 'Detection Mask Image Stream' }));
        this._displayPrivacyMaskControls = false;
        this._displayDetectionMaskControls = true;
        this._displayAppState = false;
        break;
    }
  }

  onOpenedChange(opened: boolean) {
    console.log(`Side panel openend state: ${opened}`);

    this._appState$ = this.rosSvc.subAppState(opened);

    //this.store.dispatch(VisionActions.setCameraPolling({ enabled: opened }));
  }

  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();

    //this.store.dispatch(VisionActions.setCameraPolling({ enabled: false }));

    this.rosSvc.disconnect();
  }

  OnPMEdit(): void {

    let connection = new BobRosConnection();
    connection.open(false);

    try {
      if (connection.isConnected) {
        this.rosSvc.svcPrivacyMaskOverride(connection, false);
      }
    } finally {
      connection.close();
    }

    this.privacymaskcreator.clearMask();
  }

  OnPMCancel(): void {
    this.privacymaskcreator.redrawCanvas();

    let connection = new BobRosConnection();
    connection.open(false);

    try {
      if (connection.isConnected) {
        this.rosSvc.svcPrivacyMaskOverride(connection, true);
      }
    } finally {
      connection.close();
    }    
  }

  OnPMDelete(): void {
    this._matDialog.open(
      ConfirmationDialogComponent, {
        data: { 
          title: 'Confirm privacy mask delete',
          message: `Are you sure you want to delete your privacy mask?`,
          cancelButton: 'No',
          acceptButton: 'Yes'
        }
      })
      .afterClosed()
      .subscribe((accept: boolean) => {
        if (accept) {

          let connection = new BobRosConnection();
          connection.open(false);
      
          try {
            if (connection.isConnected) {
              this.rosSvc.svcDeletePrivacyMask(connection);
            }
          } finally {
            connection.close();
          }             
        }
      });
  }

  OnPMSave(): void {
    const svgContent = this.privacymaskcreator.getMaskAsSVG();
    console.log(svgContent);

    let connection = new BobRosConnection();
    connection.open(false);

    try {
      if (connection.isConnected) {
        this.rosSvc.svcSendMaskSVG(connection, svgContent, 'privacy-mask.svg');
      }
    } finally {
      connection.close();
    }

    this.privacymaskcreator.clearMask();

    connection = new BobRosConnection();
    connection.open(false);

    try {
      if (connection.isConnected) {
        this.rosSvc.svcPrivacyMaskOverride(connection, true);
      }
    } finally {
      connection.close();
    }
  }

  OnPMClear(): void {
    this.privacymaskcreator.clearMask();
  }

  OnEditModeChanged(editMode:boolean): void {
    this.store.dispatch(VisionActions.setMaskEditMode({ enabled: editMode }));
  }
}
