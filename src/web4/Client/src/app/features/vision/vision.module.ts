import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { StoreModule } from '@ngrx/store';
import { EffectsModule } from '@ngrx/effects';

import { UiModule } from '../../ui';
import { CoreModule } from '../../core';

import { VisionRoutingModule } from './vision-routing.module';

import { VisionService } from './services';

import { MaskControlsComponent, StreamDisplayComponent, BobStateComponent, BobInfoComponent, 
  MaskCreationComponent, MaskCreationSvgComponent } from './components';

import { VisionIndexComponent} from './containers';

import * as fromVision from './state';

export const CONTAINERS = [VisionIndexComponent];

export const COMPONENTS = [MaskControlsComponent, StreamDisplayComponent, BobStateComponent, BobInfoComponent, 
  MaskCreationComponent, MaskCreationSvgComponent
];

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
    //CoreModule,
    VisionRoutingModule,
  ],
  declarations: [COMPONENTS, CONTAINERS, ENTRY_COMPONENTS],
  providers: [PROVIDERS],
  //exports: [COMPONENTS, CONTAINERS],
})
export class VisionModule { }