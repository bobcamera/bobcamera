import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store} from '@ngrx/store';
import { Subject, Observable } from 'rxjs';
import { takeUntil, filter } from 'rxjs/operators';

import * as MainActions from '../../../../core/state/shared.actions';
import * as RecordingActions from '../../state/recording.actions';

import { CoreState } from '../../../../core/state';
import { RecordingState, getRecordingHeading, getRecordingMessage, getRecordingItems, getRecordingMenuPanelExpanded } from '../../state/recording.reducer';

import { NotificationType, NotificationModel } from '../../../../core/models';
import { RecordingQuery, RecordingDto } from '../../models';

@Component({
  selector: 'bob-recording-index',
  templateUrl: './recording-index.component.html',
  styleUrls: ['./recording-index.component.scss']
})
export class RecordingIndexComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();
  _heading$: Observable<string>;
  _menuPanelExpanded$: Observable<boolean>;

  constructor(private mainStore: Store<CoreState>, private store: Store<RecordingState>) {
  }

  ngOnInit(): void {

    this._heading$ = this.store.select(getRecordingHeading);
    this._menuPanelExpanded$ = this.store.select(getRecordingMenuPanelExpanded);

    this.store.select(getRecordingMessage)
    .pipe(takeUntil(this._ngUnsubscribe$), filter(message => !!message))
    .subscribe((message: string) => {
      let notificationModel: NotificationModel = { type: NotificationType.Information, message: message };
      this.mainStore.dispatch(MainActions.Notification({ notification: notificationModel }));
      this.store.dispatch(RecordingActions.clearMessage());
    });

    this.store.select(getRecordingItems)
    .pipe(takeUntil(this._ngUnsubscribe$), filter(items => !!items))
    .subscribe((items: RecordingDto[]) => {
      if (items.length > 0) {
        let notificationModel: NotificationModel = { type: NotificationType.Information, message: `${items.length} recordings loaded` };
        this.mainStore.dispatch(MainActions.Notification({ notification: notificationModel }));
      }
    });    
  }

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }

  onToggleMenuPanel() {
    this.store.dispatch(RecordingActions.menuPanelToggle());
  }

  setMessage(message: string) {
    this.store.dispatch(RecordingActions.setMessage({message: message}));
  }

  loadRecords() {
    this.store.dispatch(RecordingActions.getRecordings({query: <RecordingQuery>{directory: ''}}));
  }

  deleteRecording() {
    this.store.dispatch(RecordingActions.deleteRecording({data: <RecordingDto>{id: 1, file: '1.txt'}}));
  }
}
