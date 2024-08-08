import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input } from '@angular/core';
import { Subject, Observable } from 'rxjs';

import { IconDefinition } from '@fortawesome/free-solid-svg-icons';
import { faCircle, faImage, faMoon, faSun, faCloud, faThumbsUp, faThumbsDown } from '@fortawesome/free-solid-svg-icons';

import { AppInfoDto, AppStateDto } from '../../models';

@Component({
  selector: 'bob-bob-state',
  templateUrl: './bob-state.component.html',
  styleUrls: ['./bob-state.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class BobStateComponent implements OnInit, OnDestroy {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();

  @Input() AppState$: Observable<AppStateDto>;

  constructor() {
  }

  ngOnInit(): void {
  }

  formatCloudCover(percentage_cloud_cover: number): string {
    return `${Math.trunc(percentage_cloud_cover)}%`;
  }

  formatDayNightIcon(day_night_enum: number): IconDefinition {
    switch(day_night_enum)
    {
      case 1:
        return faSun;
      case 2:
        return faMoon;
    }
    return faImage;
  }

  formatDayNightClass(day_night_enum: number): string {
    switch(day_night_enum)
    {
      case 1:
        return 'day';
      case 2:
        return 'night';
    }
    return null;
  }

  formatTrackingIcon(alive: number): IconDefinition {
    /*if (alive > 0) {
      return  faThumbsUp;
    } 
    return faThumbsDown;*/
    return faCircle;
  }

  formatRecordingIcon(alive: number): IconDefinition {
    /*if (alive > 0) {
      return  faThumbsUp;
    } 
    return faThumbsDown;*/
    return faCircle;
  }

  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
