import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { NotFoundPageComponent, AccessDeniedPageComponent } from './core/containers';

const routes: Routes = [
  { path: '', redirectTo: 'vision',  pathMatch:'full' },  
  { path: 'vision', loadChildren: () => import('./features/vision/vision.module').then(m => m.VisionModule) },
  { path: 'playback', loadChildren: () => import('./features/playback/playback.module').then(m => m.PlaybackModule) },
  { path: 'about', loadChildren: () => import('./features/about/about.module').then((m) => m.AboutModule) },
  { path: 'settings', loadChildren: () => import('./features/settings/settings.module').then((m) => m.SettingsModule) },  
  { path: 'access-denied', component: AccessDeniedPageComponent, data: { title: 'Access Denied' } },
  { path: '**', component: NotFoundPageComponent, data: { title: 'Page Not Found' } },
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }