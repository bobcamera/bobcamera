import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { VisionIndexComponent } from './containers';

const routes: Routes = [
  { path: '', redirectTo: 'annotated',  pathMatch:'full' },
  { path:':type', component:VisionIndexComponent, data: { title: 'BOB Universal Object Tracker' } }
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class VisionRoutingModule { }