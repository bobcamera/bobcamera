import { Injectable, Inject } from "@angular/core";
import { catchError, Observable, Subject } from "rxjs";
import { map, switchMap } from 'rxjs/operators';

import * as ROSLIB from 'roslib';

import { RosLibServiceBase } from '../../mod-main/services';

import { AppStateDto } from '../models';

export enum SubscriptionType {
    Annotated,
    ForegroundMask,
    DetectionMask,
    PrivacyMask,
}

@Injectable({
  providedIn: 'root',
})
export class BobRosService extends RosLibServiceBase {    

    // https://stackoverflow.com/questions/64647388/roslib-with-angular-10
    constructor(@Inject('BASE_URL') baseUrl: string) {
        super(baseUrl);        
    }

    private _types = {
        [SubscriptionType.Annotated]: "/bob/frames/annotated/resized/compressed",
        [SubscriptionType.ForegroundMask]: "/bob/frames/foreground_mask/resized/compressed",
        [SubscriptionType.DetectionMask]: "/bob/frames/allsky/original/resized/compressed",
        [SubscriptionType.PrivacyMask]: "/bob/frames/allsky/original/resized/compressed"
    };

    private _videoStream = new Subject<any>();
    private _appState = new Subject<any>();

    private _videoStreamListener: ROSLIB.Topic;
    private _appStateListener: ROSLIB.Topic;

    public videoStream(type: SubscriptionType): Observable<string>{

        this.internal_videoStream(type);

        return this._videoStream.pipe(
            map((message: any) => this.transformImage(message))
          );        
    }

    public appState(subscribe: boolean): Observable<AppStateDto> {

        if (subscribe) {
            if (!this._appStateListener) {
                this.internal_appState();
            }
        } else {
            super.unsubscribeTopic(this._appStateListener);
            this._appStateListener = null;
        }

        return this._appState.pipe(
            map((message: any) => this.transformStatus(message))
          );
    }

    private internal_videoStream(type: SubscriptionType) {

        if (this._videoStreamListener) {
            super.unsubscribeTopic(this._videoStreamListener);
        }

        const url = this._types[type];

        this._videoStreamListener = super.subscribeTopic<string>(
            url, 
            'sensor_msgs/msg/CompressedImage', 
            (msg: any) => { this._videoStream.next(msg);}
        );
    }

    private internal_appState() {
        this._appStateListener = super.subscribeTopic<string>(
            '/bob/monitoring/status', 
            'bob_interfaces/msg/MonitoringStatus', 
            (msg: any) => { this._appState.next(msg);}
        );
    }

    private transformImage(message: any): string {
        return `data:image/jpeg;base64,${message.data}`;
    }

    private transformStatus(message: any): AppStateDto {
        return {
            trackable: message.trackable,
            alive: message.alive,
            started: message.started,
            ended: message.ended,
            sensitivity: message.sensitivity,
            max_blobs_reached: message.max_blobs_reached,
            recording: message.recording,
            percentage_cloud_cover: message.percentage_cloud_cover,
            unimodal_cloud_cover: message.unimodal_cloud_cover,
            day_night_enum: message.day_night_enum,
            avg_brightness: message.avg_brightness
        };
    }
}