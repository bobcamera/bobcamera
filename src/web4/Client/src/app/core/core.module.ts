import { NgModule, ModuleWithProviders, Optional, SkipSelf } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RouterModule } from '@angular/router';

import { FaIconLibrary } from '@fortawesome/angular-fontawesome';

import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../ui';

import { NotificationPublisher, NotificationHandler, ErrorService, ApiServiceBase, SvgService, LocalStorageService } from './services';
import { AppComponent, NotFoundPageComponent, AccessDeniedPageComponent } from './containers';
import { ConfirmationDialogComponent } from './components';

import * as fromCore from './state';

import { GuiEffects } from './state/gui.effects';
import { SharedEffects } from './state/shared.effects';
import { SettingsEffects } from './state/settings.effects';

import {
    faCog,
    faBars,
    faRocket,
    faPowerOff,
    faUserCircle,
    faPlayCircle
  } from '@fortawesome/free-solid-svg-icons';
  import {
    faGithub,
    faMediumM,
    faTwitter,
    faInstagram,
    faYoutube,
    faFacebook,
    faDiscord
  } from '@fortawesome/free-brands-svg-icons';

export const COMPONENTS = [ConfirmationDialogComponent];
//export const ENTRY_COMPONENTS = [ConfirmationDialogComponent]
export const CONTAINERS = [AppComponent, NotFoundPageComponent, AccessDeniedPageComponent];
export const PIPE_PROVIDERS = []
export const PROVIDERS = [NotificationPublisher, NotificationHandler, ErrorService, ApiServiceBase, SvgService, LocalStorageService ];
export const GUARDS = []
export const PIPES = [];
export const EFFECTS = [GuiEffects, SharedEffects, SettingsEffects];
export const DIRECTIVES = [];

@NgModule({
    imports: [
        StoreModule.forFeature('core', fromCore.coreReducers),
        EffectsModule.forFeature(EFFECTS),
        CommonModule,
        RouterModule,
        UiModule
    ],
    declarations: [COMPONENTS, PIPES, CONTAINERS, DIRECTIVES],
    providers: [PROVIDERS, GUARDS, PIPE_PROVIDERS],
    exports: [COMPONENTS, PIPES, CONTAINERS, DIRECTIVES]
})
export class CoreModule {
    static forRoot(): ModuleWithProviders<CoreModule> {
      return {
          ngModule: CoreModule,
          providers: [PROVIDERS, GUARDS, PIPE_PROVIDERS],
      };
  }

  constructor(
    @Optional()
    @SkipSelf()
    parentModule: CoreModule,
    faIconLibrary: FaIconLibrary
  ) {
    if (parentModule) {
      throw new Error('CoreModule is already loaded. Import only in AppModule');
    }
    faIconLibrary.addIcons(
      faCog,
      faBars,
      faRocket,
      faPowerOff,
      faUserCircle,
      faPlayCircle,
      faGithub,
      faMediumM,
      faTwitter,
      faInstagram,
      faYoutube,
      faFacebook,
      faDiscord      
    );
  }
}