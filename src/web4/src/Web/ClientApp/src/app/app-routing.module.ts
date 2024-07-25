import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { NotFoundPageComponent, AccessDeniedPageComponent } from './mod-main/containers';

const routes: Routes = [
  { path: '', redirectTo: 'vision',  pathMatch:'full' },
  { path: 'vision', loadChildren: () => import('./mod-vision/vision.module').then(m => m.VisionModule) },    
  { path: 'access-denied', component: AccessDeniedPageComponent, data: { title: 'Access Denied' } },
  { path: '**', component: NotFoundPageComponent, data: { title: 'Page Not Found' } },
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }