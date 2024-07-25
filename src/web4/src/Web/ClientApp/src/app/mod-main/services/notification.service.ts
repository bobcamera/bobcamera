import { Injectable } from '@angular/core';
import { MatSnackBar, MatSnackBarConfig } from '@angular/material/snack-bar';
import { Store } from '@ngrx/store';
import { filter } from 'rxjs/operators';

import { MainState } from '../state/main.reducer';
import { Notification } from '../state/main.actions';
import { NotificationType, NotificationModel } from '../models';
import { environment } from '../../../environments/environment';

@Injectable({
  providedIn: 'root'
})
export class NotificationPublisher {

  constructor(private store: Store<MainState>) {
  }

  PublishInfo(message: string) {
    this.Publish({ type: NotificationType.Information, message: message });
  }

  PublishWarn(message: string) {
    this.Publish({ type: NotificationType.Warning, message: message });
  }

  PublishError(message: string) {
    this.Publish({ type: NotificationType.Error, message: message });
  }

  Publish(notification: NotificationModel) {
    this.store.dispatch(Notification({ notification: notification }));
  }
}

@Injectable({
  providedIn: 'root'
})
export class NotificationHandler {

  constructor(private store: Store<MainState>, private snackBar: MatSnackBar) {
    /*this.store.select(getNotification)
    .pipe(filter(notification => !!notification))
    .subscribe((notification: NotificationModel) => this.handleNotification(notification));*/
  }

  DisplayInfo(content: string) {
    const config = new MatSnackBarConfig();
    config.panelClass = ['info-snackbar'];
    config.duration = environment.settings.serices.notification.displayDuration.info;

    this.snackBar.open(content, 'Close', config);
  }

  DisplayWarn(content: string) {
    const config = new MatSnackBarConfig();
    config.panelClass = ['warn-snackbar'];
    config.duration = environment.settings.serices.notification.displayDuration.warn;

    this.snackBar.open(content, 'Warn - Close', config);
  }

  DisplayError(content: string) {
    const config = new MatSnackBarConfig();
    config.panelClass = ['error-snackbar'];
    config.duration = environment.settings.serices.notification.displayDuration.error

    this.snackBar.open(content, 'Error - Close', config);
  }
}