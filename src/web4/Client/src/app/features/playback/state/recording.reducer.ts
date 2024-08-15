import { createFeatureSelector, createReducer, createSelector, on } from '@ngrx/store';
import { HttpErrorResponse } from '@angular/common/http';

import * as fromRoot from '../../../state'
import * as RecordingActions from './recording.actions'

import { LoadingModel, RecordingQuery, RecordingDto } from '../models';

export const featureKey = 'recording';

export interface RecordingState {
  heading: string;
  menuPanelExpanded: boolean;
  contextPanelExpanded: boolean;
  message: string;

  error: HttpErrorResponse;
  loading: boolean;
  loadingMessage: string;
  recordings: RecordingDto[]
}

export interface State extends fromRoot.State {
  [featureKey]: RecordingState;
}

const initialState: RecordingState = {
  heading: 'Recording Component',
  menuPanelExpanded: false,
  contextPanelExpanded: false,
  message: '',

  error: null,
  loading: false,
  loadingMessage: '',
  recordings: [],
};


export const recordingReducer = createReducer<RecordingState>(
  initialState,

  on(RecordingActions.setHeading, (state, action): RecordingState => { return { ...state, heading: action.heading }; }),

  on(RecordingActions.menuPanelToggle, (state, action): RecordingState => { return { ...state, menuPanelExpanded: !state.menuPanelExpanded }; }),
  on(RecordingActions.contextPanelToggle, (state, action): RecordingState => { return { ...state, contextPanelExpanded: !state.contextPanelExpanded }; }),

  on(RecordingActions.setMessage, (state, action): RecordingState => { return { ...state, message: action.message }; }),
  on(RecordingActions.clearMessage, (state, action): RecordingState => { return { ...state, message: '' }; }),

  on(RecordingActions.getRecordings, (state, action): RecordingState => { return { ...state, loading: true, loadingMessage: 'Loading Question Items', recordings: [] }; }),
  on(RecordingActions.getRecordingsSuccess, (state, action): RecordingState => { return { ...state, loading: false, error: null, loadingMessage: null, recordings: action.data }; }),  

  on(RecordingActions.deleteRecording, (state, action): RecordingState => { return { ...state, loading: true, loadingMessage: 'Deleting Question Item' }; }),
  on(RecordingActions.deleteRecordingSuccess, (state, action): RecordingState => {

    console.log(`Deleteting file ${action.data.file}`);

    return { 
      ...state, 
      loading: false, 
      error: null,
      loadingMessage: null, 
      recordings : state.recordings.filter((q: RecordingDto) => q.id !== action.data.id),
    }; 
  }),  
)

// Selectors
const getRecordingState = createFeatureSelector<RecordingState>(featureKey);

export const getRecordingHeading = createSelector(
  getRecordingState,
  state => state.heading
);

export const getRecordingMenuPanelExpanded = createSelector(
  getRecordingState,
  state => state.menuPanelExpanded
);

export const getRecordingContextPanelExpanded = createSelector(
  getRecordingState,
  state => state.contextPanelExpanded
);

export const getRecordingMessage = createSelector(
  getRecordingState,
  state => state.message
);

export const getRecordingError = createSelector(
  getRecordingState,
  state => state.error
);

export const getRecordingLoading = createSelector(
  getRecordingState,
  state => <LoadingModel>{loading: state.loading, message: state.loadingMessage}
);

export const getRecordingItems = createSelector(
  getRecordingState,
  state => state.recordings
);