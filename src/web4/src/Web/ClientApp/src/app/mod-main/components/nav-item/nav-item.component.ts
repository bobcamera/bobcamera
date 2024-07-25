import { Component, OnInit, Input, Output, ChangeDetectionStrategy, OnDestroy, EventEmitter } from '@angular/core';
import { Subject } from 'rxjs';

@Component({
  selector: 'bob-nav-item',
  templateUrl: './nav-item.component.html',
  styleUrls: ['./nav-item.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class NavItemComponent implements OnInit, OnDestroy {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  @Input() icon = '';
  @Input() routerLink : string[] = [''];
  @Input() hint = '';
  @Input() roles: string[];
  @Input() disabled? = false;
  @Output() secureClick = new EventEmitter();

  constructor() { }

  ngOnInit() {
  }

  ngOnDestroy() {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }

  clicked() {
    if (this.isEnabled()) {
      this.secureClick.emit();
    }
  }

  isEnabled(): boolean {
    return (this.disabled) ? false : true;
  }
}