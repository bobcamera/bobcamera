import { Injectable, NgZone } from '@angular/core';
import { MatSnackBar, MatSnackBarConfig } from '@angular/material/snack-bar';
import { Store } from '@ngrx/store';
import { filter } from 'rxjs/operators';

import { CoreState } from '../state/';
import { Notification } from '../state/shared.actions';
import { NotificationType, NotificationModel } from '../models';

@Injectable({
  providedIn: 'root'
})
export class NotificationPublisher {

  constructor(private store: Store<CoreState>) {
  }

  default(message: string) {
    this.publish({ type: NotificationType.Default, message: message });
  }  

  info(message: string) {
    this.publish({ type: NotificationType.Information, message: message });
  }

  success(message: string) {
    this.publish({ type: NotificationType.Success, message: message });
  }

  warn(message: string) {
    this.publish({ type: NotificationType.Warning, message: message });
  }

  error(message: string) {
    this.publish({ type: NotificationType.Error, message: message });
  }

  private publish(notification: NotificationModel) {
    this.store.dispatch(Notification({ notification: notification }));
  }
}

@Injectable({
  providedIn: 'root'
})
export class NotificationHandler {
  constructor(
    private readonly snackBar: MatSnackBar,
    private readonly zone: NgZone
  ) {}

  default(message: string) {
    this.show(message, {
      duration: 2000,
      panelClass: 'default-notification-overlay'
    });
  }

  info(message: string) {
    this.show(message, {
      duration: 2000,
      panelClass: 'info-notification-overlay'
    });
  }

  success(message: string) {
    this.show(message, {
      duration: 2000,
      panelClass: 'success-notification-overlay'
    });
  }

  warn(message: string) {
    this.show(message, {
      duration: 2500,
      panelClass: 'warning-notification-overlay'
    });
  }

  error(message: string) {
    this.show(message, {
      duration: 3000,
      panelClass: 'error-notification-overlay'
    });
  }

  private show(message: string, configuration: MatSnackBarConfig) {
    // Need to open snackBar from Angular zone to prevent issues with its position per
    // https://stackoverflow.com/questions/50101912/snackbar-position-wrong-when-use-errorhandler-in-angular-5-and-material
    this.zone.run(() => this.snackBar.open(message, undefined, configuration));
  }
}