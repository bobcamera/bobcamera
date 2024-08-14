import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store} from '@ngrx/store';
import { Subject, Observable } from 'rxjs';
import { takeUntil, filter } from 'rxjs/operators';

import * as MainActions from '../../../mod-main/state/main.actions';
import * as VisionActions from '../../state/vision.actions';

import { MainState } from '../../../mod-main/state/main.reducer';
import { VisionState, getVisionHeading, getVisionMessage, getVisionMenuPanelExpanded } from '../../state/vision.reducer';

import { NotificationType, NotificationModel } from '../../../mod-main/models';

@Component({
  selector: 'bob-vision-index',
  templateUrl: './vision-index.component.html',
  styleUrls: ['./vision-index.component.scss']
})
export class VisionIndexComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();
  _heading$: Observable<string>;
  _menuPanelExpanded$: Observable<boolean>;

  constructor(private mainStore: Store<MainState>, private store: Store<VisionState>) {
  }

  ngOnInit(): void {

    this._heading$ = this.store.select(getVisionHeading);
    this._menuPanelExpanded$ = this.store.select(getVisionMenuPanelExpanded);

    this.store.select(getVisionMessage)
    .pipe(takeUntil(this._ngUnsubscribe$), filter(message => !!message))
    .subscribe((message: string) => {
      let notificationModel: NotificationModel = { type: NotificationType.Information, message: message };
      this.mainStore.dispatch(MainActions.Notification({ notification: notificationModel }));      
    });
  }

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
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
}
