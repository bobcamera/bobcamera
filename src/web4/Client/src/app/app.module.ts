import { BrowserModule } from '@angular/platform-browser';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { NgModule } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { HttpClientModule, HTTP_INTERCEPTORS } from '@angular/common/http';

import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';
import { StoreDevtoolsModule } from '@ngrx/store-devtools';

import { AuthorizeInterceptor } from 'src/api-authorization/authorize.interceptor';

import { UiModule } from './mod-ui';
import { MainModule } from './mod-main';
import { AppRoutingModule } from './app-routing.module';

import { AppRootComponent } from './mod-main/containers';

import { rootReducers, metaReducers } from './state';

import { environment } from 'src/environments/environment';

@NgModule({
  imports: [
    BrowserModule.withServerTransition({ appId: 'ng-cli-universal' }),
    BrowserAnimationsModule,
    HttpClientModule,
    FormsModule,

    MainModule.forRoot(),
    UiModule,
    AppRoutingModule,
    
    StoreModule.forRoot(rootReducers, { metaReducers }),
    environment.production 
    ? [] 
    : StoreDevtoolsModule.instrument({
      maxAge: 25,
      logOnly: environment.production,
    }),
    EffectsModule.forRoot([]),
  ],
  declarations: [
  ],
  exports: [
  ],
  providers: 
  [
    { provide: HTTP_INTERCEPTORS, useClass: AuthorizeInterceptor, multi: true }    
  ],
  bootstrap: [AppRootComponent]
})
export class AppModule { }