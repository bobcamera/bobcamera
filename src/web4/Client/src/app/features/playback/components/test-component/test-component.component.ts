import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store} from '@ngrx/store';
import { Subject, Observable } from 'rxjs';

import * as RecordingActions from '../../state/recording.actions';
import { RecordingState, getRecordingContextPanelExpanded } from '../../state/recording.reducer';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss']
})
export class TestComponentComponent implements OnInit, OnDestroy {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  _contextPanelExpanded$: Observable<boolean>;

  constructor(private store: Store<RecordingState>) {
  }

  ngOnInit(): void {
    this._contextPanelExpanded$ = this.store.select(getRecordingContextPanelExpanded);
    this.store.dispatch(RecordingActions.setHeading({ heading: 'Recording Test Component' }));
  }

  onToggleContextPanel() {
    this.store.dispatch(RecordingActions.contextPanelToggle());
  }

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
