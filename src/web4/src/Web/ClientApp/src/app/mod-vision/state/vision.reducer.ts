import { createFeatureSelector, createReducer, createSelector, on } from '@ngrx/store';

import * as fromRoot from '../../state'
import * as VisionActions from './vision.actions'

export const featureKey = 'vision';

export interface VisionState {
  heading: string;
  navPanelExpanded: boolean;
  message: string;
}

export interface State extends fromRoot.State {
  [featureKey]: VisionState;
}

const initialState: VisionState = {
  heading: 'Vision Component',
  navPanelExpanded: true,
  message: '',
};


export const visionReducer = createReducer<VisionState>(
  initialState,

  on(VisionActions.setHeading, (state, action): VisionState => { return { ...state, heading: action.heading }; }),

  on(VisionActions.navPanelExpanded, (state, action): VisionState => { return { ...state, navPanelExpanded: action.expanded }; }),

  on(VisionActions.setMessage, (state, action): VisionState => { return { ...state, message: action.message }; }),
  on(VisionActions.clearMessage, (state, action): VisionState => { return { ...state, message: '' }; })  
)

// Selectors
const getVisionState = createFeatureSelector<VisionState>(featureKey);

export const getVisionHeading = createSelector(
  getVisionState,
  state => state.heading
);

export const getVisionNavPanelExpanded = createSelector(
  getVisionState,
  state => state.navPanelExpanded
);

export const getVisionMessage = createSelector(
  getVisionState,
  state => state.message
);
