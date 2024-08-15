import browser from 'browser-detect';
import { Component, OnInit } from '@angular/core';
import { MatSelectChange } from '@angular/material/select';
import { Store, select } from '@ngrx/store';
import { Observable } from 'rxjs';

import { environment as env } from '../../../../environments/environment';

import { MainState, selectSettingsLanguage, selectEffectiveTheme } from '../../state/main.reducer';

import { LocalStorageService } from '../../services';

/*import {
  actionSettingsChangeAnimationsPageDisabled,
  actionSettingsChangeLanguage
} from '../core/settings/settings.actions';*/

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
    { link: 'about', label: 'anms.menu.about' },
    { link: 'vision', label: 'anms.menu.vision' },
    { link: 'playback', label: 'anms.menu.playback' },
  ];  
  navigationSideMenu = [
    ...this.navigation,
    { link: 'settings', label: 'anms.menu.settings' }
  ];

  isAuthenticated$: Observable<boolean> | undefined;
  stickyHeader$: Observable<boolean> | undefined;
  language$: Observable<string> | undefined;
  theme$: Observable<string> | undefined;

  constructor(
    private store: Store<MainState>,
    private storageService: LocalStorageService
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

    /*this.isAuthenticated$ = this.store.pipe(select(selectIsAuthenticated));
    this.stickyHeader$ = this.store.pipe(select(selectSettingsStickyHeader));*/
    this.language$ = this.store.pipe(select(selectSettingsLanguage));
    this.theme$ = this.store.pipe(select(selectEffectiveTheme));
  }

  onLoginClick() {
    //this.store.dispatch(authLogin());
  }

  onLogoutClick() {
    //this.store.dispatch(authLogout());
  }

  onLanguageSelect(event: MatSelectChange) {
    /*this.store.dispatch(
      actionSettingsChangeLanguage({ language: event.value })
    );*/
  }
}
