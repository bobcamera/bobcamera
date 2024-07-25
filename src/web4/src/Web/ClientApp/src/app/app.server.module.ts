import { NgModule } from '@angular/core';
import { ServerModule } from '@angular/platform-server';
import { AppRootComponent } from './mod-main/containers/';
import { AppModule } from './app.module';

@NgModule({
    imports: [AppModule, ServerModule],
    bootstrap: [AppRootComponent]
})
export class AppServerModule { }
