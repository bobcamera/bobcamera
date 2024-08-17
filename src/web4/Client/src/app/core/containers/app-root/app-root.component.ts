import { Component, OnInit, OnDestroy } from '@angular/core';
import { Router, NavigationEnd, ActivatedRoute } from '@angular/router';
import { Title } from '@angular/platform-browser';
import { Store } from '@ngrx/store';
import { Subject, Observable } from 'rxjs';
import { takeUntil, filter, map } from 'rxjs/operators';

import { CoreState } from '../../state';
import { MenuDrawerToggle } from '../../state/gui.actions';
import { getNotification } from '../../state/shared.selectors';
import { getMenuDrawerExpanded } from '../../state/gui.selectors';

import { NotificationHandler } from '../../services'
import { NotificationModel, NotificationType } from '../../models'

import { environment as env } from '../../../../environments/environment';

@Component({
  selector: 'bob-app-root',
  templateUrl: './app-root.component.html',
  styleUrls: ['./app-root.component.scss']
})
export class AppRootComponent implements OnInit, OnDestroy {

  //https://www.techiediaries.com/angular-material-navigation-toolbar-sidenav/

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();  
  _menuDrawExpanded$: Observable<boolean>;

  _currentYear: number = 0;

  constructor(private store: Store<CoreState>, private router: Router, private titleService: Title, 
    private notificationHandler: NotificationHandler) { }

  ngOnInit(): void {

    this._currentYear = new Date().getFullYear();

    this.router.events
    .pipe(
      takeUntil(this._ngUnsubscribe$),
      filter((event) => event instanceof NavigationEnd),
      map(() => {
        let route: ActivatedRoute = this.router.routerState.root;
        let routeTitle = '';
        while (route!.firstChild) {
          route = route.firstChild;
        }
        if (route.snapshot.data['title']) {
          routeTitle = route!.snapshot.data['title'];
        }
        return routeTitle;
      })
    )
    .subscribe((title: string) => {
      const t = title ? `${title} - ${env.appName}` : env.appName
      //console.debug(t);
      this.titleService.setTitle(t);
    });

    this.store.select(getNotification)
    .pipe(takeUntil(this._ngUnsubscribe$), filter(notification => !!notification))
    .subscribe((notification: NotificationModel) => this.handleNotification(notification));

    this._menuDrawExpanded$ = this.store.select(getMenuDrawerExpanded);
  }

  ngOnDestroy() {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }

  onToggleSideNav() {
    this.store.dispatch(MenuDrawerToggle());
  }
  
  name() {
    return env.appName;
  }

  version() {
    return env.version;
  }

  year() {
    return this._currentYear;
  }

  handleNotification(notificationModel: NotificationModel) {

    switch(notificationModel.type) {
      case NotificationType.Default:
        this.notificationHandler.default(notificationModel.message);
        break;
        case NotificationType.Information:
          this.notificationHandler.info(notificationModel.message);
          break;              
      case NotificationType.Success:
        this.notificationHandler.success(notificationModel.message);
        break;
      case NotificationType.Warning:
        this.notificationHandler.warn(notificationModel.message);
        break;
      case NotificationType.Error:
        this.notificationHandler.error(notificationModel.message);
        break;                  
    }
  }
}