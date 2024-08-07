import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../mod-ui';
import { MainModule } from '../mod-main';

import { VisionRoutingModule } from './vision-routing.module';

import { VisionService } from './services';

import { TestComponentComponent, MaskControlsComponent, StreamDisplayComponent, BobStateComponent, BobInfoComponent } from './components';

import { VisionIndexComponent} from './containers';

import * as fromVision from './state';

export const CONTAINERS = [VisionIndexComponent];

export const COMPONENTS = [TestComponentComponent, MaskControlsComponent, StreamDisplayComponent, BobStateComponent, BobInfoComponent];

export const ENTRY_COMPONENTS = [];

export const PROVIDERS = [VisionService];

export const EFFECTS = [fromVision.VisionEffects];

//export const PIPES = [];

@NgModule({
  imports: [
    CommonModule,
    StoreModule.forFeature(fromVision.featureKey, fromVision.visionReducer),
    EffectsModule.forFeature(EFFECTS),
    UiModule,
    MainModule,
    VisionRoutingModule,
  ],
  declarations: [COMPONENTS, CONTAINERS, ENTRY_COMPONENTS, BobInfoComponent],
  providers: [PROVIDERS],
  //exports: [COMPONENTS, CONTAINERS],
})
export class VisionModule { }