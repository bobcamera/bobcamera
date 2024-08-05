import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../mod-ui';
import { MainModule } from '../mod-main';

import { VisionRoutingModule } from './vision-routing.module';

import { VisionService, BobRosService } from './services';

import { TestComponentComponent } from './components';

import { VisionIndexComponent} from './containers';

import * as fromVision from './state';

export const CONTAINERS = [VisionIndexComponent];

export const COMPONENTS = [TestComponentComponent];

export const ENTRY_COMPONENTS = [];

export const PROVIDERS = [VisionService, BobRosService];

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
  declarations: [COMPONENTS, CONTAINERS, ENTRY_COMPONENTS],
  providers: [PROVIDERS],
  //exports: [COMPONENTS, CONTAINERS],
})
export class VisionModule { }