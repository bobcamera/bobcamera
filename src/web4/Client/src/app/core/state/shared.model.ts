import { HttpErrorResponse } from '@angular/common/http';

import * as fromRoot from '../../state';

import { NotificationModel } from '../models'

export interface SharedState {
    error: HttpErrorResponse;
    notification: NotificationModel;
}

/*export interface State extends fromRoot.State {
  shared: SharedState;
}*/