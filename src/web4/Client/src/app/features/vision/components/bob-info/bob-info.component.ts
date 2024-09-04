import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input, Output, EventEmitter } from '@angular/core';
import { Subject, Observable } from 'rxjs';

import { AppInfoDto } from '../../models';

@Component({
  selector: 'bob-bob-info',
  templateUrl: './bob-info.component.html',
  styleUrls: ['./bob-info.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class BobInfoComponent  implements OnInit, OnDestroy {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  @Input() AppInfo$: Observable<AppInfoDto>;
  @Output() Reload = new EventEmitter();

  constructor() {
  }

  ngOnInit(): void {
  }
  
  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }

  onReload() {
    this.Reload.emit();
  }
}
