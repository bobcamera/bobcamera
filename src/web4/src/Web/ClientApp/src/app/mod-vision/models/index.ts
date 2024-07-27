export interface LoadingModel {
    loading: boolean;
    message: string;
}

export interface CameraQuery {
    cameraName: string;
}

export interface CameraDto {
    cameraName: string;
    fps: number;
    recording: boolean;
    randomNumber: number;
}
