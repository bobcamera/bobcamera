import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input } from '@angular/core';
import { Subject, Observable } from 'rxjs';

@Component({
  selector: 'bob-stream-display',
  templateUrl: './stream-display.component.html',
  styleUrls: ['./stream-display.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class StreamDisplayComponent implements OnInit, OnDestroy {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  @Input() ImageStream$: Observable<string>;

  constructor() {
  }

  ngOnInit(): void {
  }

  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
