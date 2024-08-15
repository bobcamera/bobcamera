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

export interface AppInfoDto {
    version: string;
    frame_width: number;
    frame_height: number;
    video_fps: number;
}

export interface AppStateDto {
    trackable: number;
    alive: number;
    started: number;
    ended: number;
    sensitivity: string;
    max_blobs_reached: boolean;
    recording: boolean;
    percentage_cloud_cover: number;
    unimodal_cloud_cover: boolean;
    day_night_enum: number;
    avg_brightness: number;
}

export interface Point {
    x: number; 
    y: number
};

export enum ImageStreamTypeEnum {
    Annotated,
    ForegroundMask,
    DetectionMask,
    PrivacyMask,
}