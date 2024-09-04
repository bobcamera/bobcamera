import browser from 'browser-detect';
import { Component, OnInit } from '@angular/core';
import { MatSelectChange } from '@angular/material/select';
import { Router, NavigationEnd, ActivatedRoute } from '@angular/router';
import { Title } from '@angular/platform-browser';
import { Store, select } from '@ngrx/store';
import { Observable, Subject } from 'rxjs';
import { takeUntil, filter, map } from 'rxjs/operators';

import { environment as env } from '../../../../environments/environment';

import { CoreState } from '../../state';
import { MenuDrawerToggle } from '../../state/gui.actions';
import { getNotification } from '../../state/shared.selectors';
import { getMenuDrawerExpanded } from '../../state/gui.selectors';

import { actionSettingsChangeAnimationsPageDisabled, actionSettingsChangeLanguage } from '../../state/settings.actions';
import { selectSettingsStickyHeader, selectSettingsLanguage, selectEffectiveTheme } from '../../state/settings.selectors';

import { LocalStorageService } from '../../services';

import { NotificationHandler } from '../../services'
import { NotificationModel, NotificationType } from '../../models'

@Component({
  selector: 'bob-app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss'],
  //animations: [routeAnimations]
})
export class AppComponent implements OnInit {
  isProd = env.production;
  envName = env.appName;
  version = env.version;
  year = new Date().getFullYear();
  logo = 'assets/images/bob_logo_horizontal.png';
  languages = ['en', 'de', 'sk', 'fr', 'es', 'pt-br', 'zh-cn', 'he', 'ar'];
  navigation = [
    { link: 'about', label: 'bob.menu.about' },
    { link: 'vision', label: 'bob.menu.vision' },
    { link: 'playback', label: 'bob.menu.playback' },
  ];  
  navigationSideMenu = [
    ...this.navigation,
    { link: 'settings', label: 'bob.menu.settings' }
  ];

  ngUnsubscribe$: Subject<void> = new Subject<void>();  
  menuDrawExpanded$: Observable<boolean>;

  isAuthenticated$: Observable<boolean> | undefined;
  stickyHeader$: Observable<boolean> | undefined;
  language$: Observable<string> | undefined;
  theme$: Observable<string> | undefined;

  constructor(
    private store: Store<CoreState>, 
    private router: Router, 
    private titleService: Title, 
    private storageService: LocalStorageService,
    private notificationHandler: NotificationHandler
  ) {}

  private static isIEorEdgeOrSafari() {
    return ['ie', 'edge', 'safari'].includes(browser().name || '');
  }

  ngOnInit(): void {
    this.storageService.testLocalStorage();
    /*if (AppComponent.isIEorEdgeOrSafari()) {
      this.store.dispatch(
        actionSettingsChangeAnimationsPageDisabled({
          pageAnimationsDisabled: true
        })
      );
    }*/

    /*this.isAuthenticated$ = this.store.pipe(select(selectIsAuthenticated));*/
    this.stickyHeader$ = this.store.pipe(select(selectSettingsStickyHeader));
    this.language$ = this.store.pipe(select(selectSettingsLanguage));
    this.theme$ = this.store.pipe(select(selectEffectiveTheme));
    this.menuDrawExpanded$ = this.store.select(getMenuDrawerExpanded);

    this.router.events
    .pipe(
      takeUntil(this.ngUnsubscribe$),
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
    .pipe(takeUntil(this.ngUnsubscribe$), filter(notification => !!notification))
    .subscribe((notification: NotificationModel) => this.handleNotification(notification));        
  }

  onLoginClick() {
    //this.store.dispatch(authLogin());
  }

  onLogoutClick() {
    //this.store.dispatch(authLogout());
  }

  onLanguageSelect(event: MatSelectChange) {
    this.store.dispatch(
      actionSettingsChangeLanguage({ language: event.value })
    );
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
