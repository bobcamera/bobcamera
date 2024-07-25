import { Component, OnDestroy, OnInit } from '@angular/core';
import { Store} from '@ngrx/store';
import { Subject } from 'rxjs';

import * as VisionActions from '../../state/vision.actions';
import { VisionState} from '../../state/vision.reducer';

@Component({
  selector: 'bob-test-component',
  templateUrl: './test-component.component.html',
  styleUrls: ['./test-component.component.scss']
})
export class TestComponentComponent implements OnInit, OnDestroy {

  opened: boolean;

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  constructor(private store: Store<VisionState>) {
  }

  ngOnInit(): void {
    this.store.dispatch(VisionActions.setHeading({ heading: 'Test Component' }));
  }

  public ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
