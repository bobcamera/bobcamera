import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input } from '@angular/core';
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

  constructor() {
  }

  ngOnInit(): void {
  }
  
  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
