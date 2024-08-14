import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../mod-ui';
import { MainModule } from '../mod-main';

import { RecordingRoutingModule } from './recordings-routing.module';

import { TestComponentComponent } from './components';

import { RecordingIndexComponent } from './containers';

import { RecordingService } from './services';

import * as fromRecording from './state';

export const CONTAINERS = [RecordingIndexComponent];

export const COMPONENTS = [TestComponentComponent];

export const ENTRY_COMPONENTS = [];

export const PROVIDERS = [RecordingService];

export const EFFECTS = [fromRecording.RecordingEffects];

//export const PIPES = [];

@NgModule({
  imports: [
    CommonModule,
    StoreModule.forFeature(fromRecording.featureKey, fromRecording.recordingReducer),
    EffectsModule.forFeature(EFFECTS),
    UiModule,
    MainModule,
    RecordingRoutingModule,
  ],
  declarations: [COMPONENTS, CONTAINERS, ENTRY_COMPONENTS],
  providers: [PROVIDERS],
  //exports: [COMPONENTS, CONTAINERS],
})
export class RecordingModule { }