import { createFeatureSelector, createReducer, createSelector, on } from '@ngrx/store';

import * as fromRoot from '../../state'
import * as RecordingActions from './recording.actions'

export const featureKey = 'recording';

export interface RecordingState {
  heading: string;
  navPanelExpanded: boolean;
  message: string;
}

export interface State extends fromRoot.State {
  [featureKey]: RecordingState;
}

const initialState: RecordingState = {
  heading: 'Recording Component',
  navPanelExpanded: true,
  message: '',
};


export const recordingReducer = createReducer<RecordingState>(
  initialState,

  on(RecordingActions.setHeading, (state, action): RecordingState => { return { ...state, heading: action.heading }; }),

  on(RecordingActions.navPanelExpanded, (state, action): RecordingState => { return { ...state, navPanelExpanded: action.expanded }; }),

  on(RecordingActions.setMessage, (state, action): RecordingState => { return { ...state, message: action.message }; }),
  on(RecordingActions.clearMessage, (state, action): RecordingState => { return { ...state, message: '' }; })  
)

// Selectors
const getRecordingState = createFeatureSelector<RecordingState>(featureKey);

export const getRecordingHeading = createSelector(
  getRecordingState,
  state => state.heading
);

export const getRecordingNavPanelExpanded = createSelector(
  getRecordingState,
  state => state.navPanelExpanded
);

export const getRecordingMessage = createSelector(
  getRecordingState,
  state => state.message
);
