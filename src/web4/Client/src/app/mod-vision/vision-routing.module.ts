import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { VisionIndexComponent } from './containers';

import { TestComponentComponent } from './components';

const routes: Routes = [
  { path:'', component:VisionIndexComponent, data: { title: 'BOB Universal Object Tracker' } ,
    children: [
      { path: '', redirectTo: 'test/annotated',  pathMatch:'full' },
      { path: 'test/:type', component: TestComponentComponent, data: { title: 'Test Component' } },
    ]   
  }
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class VisionRoutingModule { }