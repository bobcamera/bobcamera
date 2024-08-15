import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { NotFoundPageComponent, AccessDeniedPageComponent } from './core/containers';

const routes: Routes = [
  { path: '', redirectTo: 'vision',  pathMatch:'full' },
  { path: 'vision', loadChildren: () => import('./features/vision/vision.module').then(m => m.VisionModule) },
  { path: 'playback', loadChildren: () => import('./features/playback/playback.module').then(m => m.PlaybackModule) },
  { path: 'access-denied', component: AccessDeniedPageComponent, data: { title: 'Access Denied' } },
  { path: '**', component: NotFoundPageComponent, data: { title: 'Page Not Found' } },
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }