import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../../ui';
import { CoreModule } from '../../core';

import { PlaybackRoutingModule } from './playback-routing.module';

import { TestComponentComponent } from './components';

import { RecordingIndexComponent } from './containers';

import { PlaybackService } from './services';

import * as fromRecording from './state';

export const CONTAINERS = [RecordingIndexComponent];

export const COMPONENTS = [TestComponentComponent];

export const ENTRY_COMPONENTS = [];

export const PROVIDERS = [PlaybackService];

export const EFFECTS = [fromRecording.RecordingEffects];

//export const PIPES = [];

@NgModule({
  imports: [
    CommonModule,
    StoreModule.forFeature(fromRecording.featureKey, fromRecording.recordingReducer),
    EffectsModule.forFeature(EFFECTS),
    UiModule,
    //CoreModule,
    PlaybackRoutingModule,
  ],
  declarations: [COMPONENTS, CONTAINERS, ENTRY_COMPONENTS],
  providers: [PROVIDERS],
  //exports: [COMPONENTS, CONTAINERS],
})
export class PlaybackModule { }