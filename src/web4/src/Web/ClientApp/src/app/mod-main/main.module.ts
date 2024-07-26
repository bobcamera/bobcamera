import { NgModule, ModuleWithProviders } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RouterModule } from '@angular/router';

import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../mod-ui';

import { NotificationPublisher, NotificationHandler, ErrorService, ApiServiceBase } from './services';
import { AppRootComponent, NotFoundPageComponent, AccessDeniedPageComponent } from './containers';
import { NavItemComponent } from './components';

import * as fromMain from './state';

export const COMPONENTS = [NavItemComponent];
export const ENTRY_COMPONENTS = []
export const CONTAINERS = [AppRootComponent, NotFoundPageComponent, AccessDeniedPageComponent];
export const PIPE_PROVIDERS = []
export const PROVIDERS = [NotificationPublisher, NotificationHandler, ErrorService, ApiServiceBase ];
export const GUARDS = []
export const PIPES = [];
export const EFFECTS = [fromMain.MainEffects];
export const DIRECTIVES = [];

@NgModule({
    imports: [
        StoreModule.forFeature(fromMain.featureKey, fromMain.mainReducer),
        EffectsModule.forFeature(EFFECTS),
        CommonModule,
        RouterModule,
        UiModule
    ],
    declarations: [COMPONENTS, PIPES, CONTAINERS, DIRECTIVES],
    providers: [PROVIDERS, GUARDS, PIPE_PROVIDERS],
    exports: [COMPONENTS, PIPES, CONTAINERS, DIRECTIVES]
})
export class MainModule {
    static forRoot(): ModuleWithProviders<MainModule> {
      return {
          ngModule: MainModule,
          providers: [PROVIDERS, GUARDS, PIPE_PROVIDERS],
      };
  }
}