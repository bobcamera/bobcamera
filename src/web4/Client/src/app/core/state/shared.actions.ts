import { HttpErrorResponse } from '@angular/common/http';
import { props, createAction } from '@ngrx/store';

import { NotificationModel } from '../models'

export const ClearState = createAction('[Shared] Clear State');

export const Notification = createAction('[Shared] Notification', props<{ notification: NotificationModel }>());
export const ClearNotification = createAction('[Shared] Clear Notification');

export const Error = createAction('[Shared] Error', props<{ error: HttpErrorResponse }>());
export const ClearError = createAction('[Shared] Clear Error');