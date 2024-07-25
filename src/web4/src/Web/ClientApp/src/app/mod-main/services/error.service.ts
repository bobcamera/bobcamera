import { Injectable } from '@angular/core';
import { HttpErrorResponse } from '@angular/common/http';

import { NotificationPublisher } from '.';
import { NotificationType } from '../models';

@Injectable({
  providedIn: 'root'
})
export class ErrorService {

  constructor(private notificationPublisher: NotificationPublisher) {
  }

  HandleError(message: string, error: HttpErrorResponse) {   
    console.error(`BOB error handler - Message: ${message}`);
    console.error('BOB error handler - Error:', error);
    this.notificationPublisher.Publish({ type: NotificationType.Error, message: error.message });
  }
}