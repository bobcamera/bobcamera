export enum NotificationType {
    Default,
    Information,
    Success,
    Warning,
    Error
}

export interface NotificationModel {
    type: NotificationType;
    message: string;
}