export enum NotificationType {
    Information,
    Warning,
    Error
}

export interface NotificationModel {
    type: NotificationType;
    message: string;
}