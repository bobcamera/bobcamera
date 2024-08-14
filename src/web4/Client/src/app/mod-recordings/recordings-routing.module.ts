import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { RecordingIndexComponent } from './containers';

import { TestComponentComponent } from './components';

const routes: Routes = [
  { path:'', component:RecordingIndexComponent, data: { title: 'BOB Universal Object Tracker' } ,
    children: [
      { path: '', redirectTo: 'test',  pathMatch:'full' },
      { path: 'test', component: TestComponentComponent, data: { title: 'Test Component' } },
    ]   
  }
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class RecordingRoutingModule { }