import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store} from '@ngrx/store';
import { Subject } from 'rxjs';

import * as RecordingActions from '../../state/recording.actions';
import { RecordingState} from '../../state/recording.reducer';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss']
})
export class TestComponentComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  constructor(private store: Store<RecordingState>) {
  }

  ngOnInit(): void {
    this.store.dispatch(RecordingActions.setHeading({ heading: 'Recording Test Component' }));
  }

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
