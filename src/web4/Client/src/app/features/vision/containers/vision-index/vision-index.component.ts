import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, ViewChild } from '@angular/core';
import { ActivatedRoute } from '@angular/router';
import { Store } from '@ngrx/store';
import { Subject, Observable, filter } from 'rxjs';
import { takeUntil, map, switchMap, distinctUntilChanged } from 'rxjs/operators';
import { MatDialog } from '@angular/material/dialog';

import { MainState } from '../../../../core/state';
import * as MainActions from '../../../../core/state/main.actions';
import { NotificationType, NotificationModel } from '../../../../core/models';

import { ConfirmationDialogComponent } from '../../../../core/components';

import * as VisionActions from '../../state/vision.actions';
import { VisionState, getVisionHeading, getVisionMenuPanelExpanded, getVisionContextPanelExpanded, getVisionCamera, getBobInfo, 
  getMaskEditMode, getMaskSvg, getVisionImageStreamType, getVisionDisplayAppState, getVisionDisplayDetectionMaskControls, 
  getVisionDisplayPrivacyMaskControls, getVisionMessage
 } from '../../state/vision.reducer';

import { CameraDto, AppInfoDto, AppStateDto, ImageStreamTypeEnum } from '../../models';

import { BobRosService } from '../../services';

import { MaskCreationComponent, MaskCreationSvgComponent } from '../../components';


@Component({
  selector: 'bob-vision-index',
  templateUrl: './vision-index.component.html',
  styleUrls: ['./vision-index.component.scss'],
  providers: [BobRosService]
})
export class VisionIndexComponent implements OnInit, OnDestroy {

  _ngUnsubscribe$: Subject<void> = new Subject<void>();
  
  _heading$: Observable<string>;
  _menuPanelExpanded$: Observable<boolean>;
  _contextPanelExpanded$: Observable<boolean>;
  //_cameraDetails$: Observable<CameraDto>;
  _imageStream$: Observable<string>;
  _appState$: Observable<AppStateDto>;
  _appInfo$: Observable<AppInfoDto>;
  _maskEditMode$: Observable<boolean>;
  _maskSvg$: Observable<string>;
  _imageStreamType$: Observable<ImageStreamTypeEnum>;
  _displayPrivacyMaskControls$: Observable<boolean>;
  _displayDetectionMaskControls$: Observable<boolean>;
  _displayAppState$: Observable<boolean>;

  @ViewChild('privacymaskcreator', {static: false}) privacymaskcreator: MaskCreationComponent;
  @ViewChild('detectionmaskcreator', {static: false}) detectionmaskcreator: MaskCreationSvgComponent;

  constructor(private mainStore: Store<MainState>, private store: Store<VisionState>, private rosSvc: BobRosService, 
    private _matDialog: MatDialog, private route: ActivatedRoute) {
  }

  ngOnInit(): void {

    this.rosSvc.connect(true);    

    this._heading$ = this.store.select(getVisionHeading);
    this._menuPanelExpanded$ = this.store.select(getVisionMenuPanelExpanded);
    //this._cameraDetails$ = this.store.select(getVisionCamera);
    this._appInfo$ = this.store.select(getBobInfo);
    this._maskEditMode$ = this.store.select(getMaskEditMode);
    this._maskSvg$ = this.store.select(getMaskSvg);
    this._contextPanelExpanded$ = this.store.select(getVisionContextPanelExpanded);
    this._imageStreamType$ = this.store.select(getVisionImageStreamType);
    this._displayPrivacyMaskControls$ = this.store.select(getVisionDisplayPrivacyMaskControls);
    this._displayDetectionMaskControls$ = this.store.select(getVisionDisplayDetectionMaskControls);
    this._displayAppState$ = this.store.select(getVisionDisplayAppState);

    this.route.params.pipe(
      takeUntil(this._ngUnsubscribe$),
      map(params => params.type),
      distinctUntilChanged(),
    ).subscribe(changedParam => {
      this.onDisplayTypeChanged(changedParam);
    });    

    this.rosSvc.connected
    .pipe(
      takeUntil(this._ngUnsubscribe$),
      distinctUntilChanged()
    )
    .subscribe((connected: boolean) => {
      console.log(`Connection status to ROS Bridge: ${connected}`);
      if (connected) {
       
        this.store.dispatch(MainActions.Notification({
          notification: { type: NotificationType.Information, message: "Connected to ROS Bridge." }}));
      } else {
        this.store.dispatch(MainActions.Notification({
          notification: { type: NotificationType.Error, message: "Connection lost to ROS Bridge." }}));
      }
    });

    this._imageStreamType$ .pipe(
      takeUntil(this._ngUnsubscribe$)
    )
    .subscribe((imageStreamType: ImageStreamTypeEnum) => {
      this._imageStream$ = this.rosSvc.subVideoStream(imageStreamType);
      if (imageStreamType === ImageStreamTypeEnum.DetectionMask) {
        this.rosSvc.svcGetMask('detection-mask.svg');
      }
    });

    this.OnBobInfoReload();

    this.store.select(getVisionMessage)
    .pipe(takeUntil(this._ngUnsubscribe$), filter(message => !!message))
    .subscribe((message: string) => {
      let notificationModel: NotificationModel = { type: NotificationType.Information, message: message };
      this.mainStore.dispatch(MainActions.Notification({ notification: notificationModel }));      
    });
  }

  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();

    //this.store.dispatch(VisionActions.setCameraPolling({ enabled: false }));

    this.rosSvc.disconnect();
  }

  onToggleMenuPanel() {
    this.store.dispatch(VisionActions.menuPanelToggle());
  }

  setMessage(message: string) {
    this.store.dispatch(VisionActions.setMessage({message: message}));
  }

  clearMessage() {
    this.store.dispatch(VisionActions.clearMessage());
  }

  onDisplayTypeChanged(type: string) {
    this.store.dispatch(VisionActions.setImageStreamType({ imageStreamType: type }));
  }

  onOpenedChange(opened: boolean) {
    console.log(`Side panel openend state: ${opened}`);
    this._appState$ = this.rosSvc.subAppState(opened);
    //this.store.dispatch(VisionActions.setCameraPolling({ enabled: opened }));
  }

  onToggleContextPanel() {
    this.store.dispatch(VisionActions.contextPanelToggle());
  }

  OnEditModeChanged(editMode: boolean): void {
    this.store.dispatch(VisionActions.setMaskEditMode({ enabled: editMode }));
  }  

  OnPMEdit(): void {
    this.rosSvc.svcPrivacyMaskOverride(false);
    this.privacymaskcreator.clearMask();
  }

  OnPMCancel(): void {
    this.privacymaskcreator.cancel();
    this.rosSvc.svcPrivacyMaskOverride(true);
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
          this.rosSvc.svcDeleteMask('privacy-mask.svg');
        }
      });
  }

  OnPMSave(): void {
    const svgContent = this.privacymaskcreator.getMaskAsSVG(false);
    this.rosSvc.svcSaveMask(svgContent, 'privacy-mask.svg');
    this.privacymaskcreator.clearMask();
    this.rosSvc.svcPrivacyMaskOverride(true);
  }

  OnPMClear(): void {
    this.privacymaskcreator.clearMask();
  }

  OnDMEdit(): void {
    this.rosSvc.svcDetectionMaskOverride(false);
    this.detectionmaskcreator.clearMask();
  }

  OnDMCancel(): void {
    this.detectionmaskcreator.cancel();
    this.rosSvc.svcDetectionMaskOverride(true);
  }

  OnDMDelete(): void {
    this._matDialog.open(
      ConfirmationDialogComponent, {
        width: '350px',
        data: { 
          title: 'Confirm detection mask delete',
          message: `Are you sure you want to delete your detection mask?`,
          cancelButton: 'No',
          acceptButton: 'Yes'
        }
      })
      .afterClosed()
      .subscribe((accept: boolean) => {
        if (accept) {
          this.rosSvc.svcDeleteMask('detection-mask.svg');
        }
      });
  }

  OnDMSave(): void {
    const svgContent = this.detectionmaskcreator.getMaskAsSVG(true);
    this.rosSvc.svcSaveMask(svgContent, 'detection-mask.svg');
    this.detectionmaskcreator.clearMask();
    this.rosSvc.svcDetectionMaskOverride(true);
  }

  OnDMClear(): void {
    this.detectionmaskcreator.clearMask();
  }

  OnBobInfoReload() : void {
    this.rosSvc.svcAppInfo();
  }
}
