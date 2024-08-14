import { HttpErrorResponse } from '@angular/common/http';
import { props, createAction } from '@ngrx/store';

import { NotificationModel } from '../models'

export const ClearState = createAction('[Main] Clear State');

export const Notification = createAction('[Main] Notification', props<{ notification: NotificationModel }>());
export const ClearNotification = createAction('[Main] Clear Notification');

export const MenuDrawerToggle = createAction('[Main] MenuDrawerToggle');

export const Error = createAction('[Main] Error', props<{ error: HttpErrorResponse }>());
export const ClearError = createAction('[Main] Clear Error');